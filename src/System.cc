/*
* This file is an implementation of "Keeping Less is More: Point Sparsification for Visual SLAM"
* Based on ORB-SLAM2 <https://github.com/raulmur/ORB_SLAM2>
*/
/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#include "System.h"
#include "Converter.h"
#include <thread>

namespace ORB_SLAM2
{
System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer)
    :mSensor(sensor), 
     mpViewer(static_cast<Viewer*>(NULL)), 
     mbActivateLocalizationMode(false),
     mbDeactivateLocalizationMode(false)
{
    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //Load ORB Vocabulary
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad) {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }

    cv::FileNode fnMapfile = fsSettings["Map.file"]; //saved map file (result of SLAM) for localization

    bool bReuseMap = false;
    if (!fnMapfile.empty()) {
        mapfile = (string)fnMapfile;
        if (LoadMap()) 
            bReuseMap = true;
        mbActivateLocalizationMode = true;
    } else{
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        mpMap = new Map();
    }

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    mpTracker = new Tracking(mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, bReuseMap);
    
    //Initialize the Viewer thread and launch
    if(bUseViewer) {
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker,strSettingsFile, bReuseMap);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode) {
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }

        if(mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mbDeactivateLocalizationMode = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode) {
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }

        if(mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mbDeactivateLocalizationMode = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode) {
            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }

        if(mbDeactivateLocalizationMode) {
            mpTracker->InformOnlyTracking(false);
            mbDeactivateLocalizationMode = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::Shutdown()
{
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }
}


bool System::LoadMap()
{
    unique_lock<mutex> MapPointGlobal(MapPoint::mGlobalMutex);

    std::ifstream loadMap(mapfile, std::ios_base::binary);
    if (!loadMap)
    {
        cerr  << "** Error: Cannot Open Mapfile: " << mapfile << endl;
        return false;
    }

    cout << "Loading Mapfile: " << mapfile << flush << endl;

    boost::archive::binary_iarchive ia(loadMap, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    cout << "...Done!" << endl;

    cout << "Map Reconstructing" << flush << endl;
    vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it : vpKFs)
    {
        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();
        if (it->mnFrameId > mnFrameId)
            mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << "...Done!" << endl;
    loadMap.close();

    return true;
}

////////////////////////////////////////////////////////////
// below is for logging

void System::SaveStats(const string &resultPath, const string &logPath)
{
    mpTracker->PrintTimeStats(resultPath, logPath);
}

void System::SaveTrajectoryTUM(const string &filename) //save whole traj on TUM format
{
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    if (!mpTracker->mbOnlyTracking){
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
            lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
        {
            if(*lbL) continue;

            KeyFrame* pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

            while(pKF->isBad())
            {
                Trw = Trw*pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw*pKF->GetPose()*Two;

            cv::Mat Tcw = (*lit)*Trw;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            f << setprecision(6) << *lT << " " <<  setprecision(9) 
            << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " 
            << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    } else { 
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();

        for(list<cv::Mat>::iterator lit=mpTracker->mlLocalizedPoses.begin(),
            lend=mpTracker->mlLocalizedPoses.end();lit!=lend;lit++,lT++)
        {
            
            cv::Mat Tcw = *lit;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            f << setprecision(6) << *lT << " " <<  setprecision(9) 
            << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " 
            << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        } //첫 프레임을 원점으로 맞춰줘야하나? 일단은 굳이.. 패스.
        //number of frames that tracking failed
        int LostCount=0;
        for(list<bool>::iterator lst=mpTracker->mlbLost.begin(), lend=mpTracker->mlbLost.end();lst!=lend;lst++)
        {
            if(*lst) LostCount++;
        }
        cout << "Lost count: " << LostCount << endl;
        }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename) //save traj of keyframes on TUM format
{
    int kf_num=0;
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad()) continue;
        kf_num++;
        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " 
          << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " 
          << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << filename.c_str() << endl;
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    if (!mpTracker->mbOnlyTracking){
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
        {
            ORB_SLAM2::KeyFrame* pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

            while(pKF->isBad())
            {
                Trw = Trw*pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw*pKF->GetPose()*Two;

            cv::Mat Tcw = (*lit)*Trw;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
                Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
                Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
        }
    } else {
        for(list<cv::Mat>::iterator lit=mpTracker->mlLocalizedPoses.begin(),
            lend=mpTracker->mlLocalizedPoses.end();lit!=lend;lit++)
        {   
            cv::Mat Tcw = *lit;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
                Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
                Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
        } //첫 프레임을 원점으로 맞춰줘야하나? 일단은 굳이.. 패스.
        //number of frames that tracking failed

        int LostCount=0;
        for(list<bool>::iterator lst=mpTracker->mlbLost.begin(), lend=mpTracker->mlbLost.end();lst!=lend;lst++)
        {
            if(*lst) LostCount++;
        }
        cout << "Lost count: " << LostCount << endl;
    }
    f.close();
}

void System::InsertRectTime(double& time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertTrackTime(double& time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}

void System::SaveMap(const string &logPath)
{
    unique_lock<mutex> MapPointGlobal(MapPoint::mGlobalMutex);

    std::stringstream SaveMapPath;
    SaveMapPath << logPath << "_map.bin";
    cout << SaveMapPath.str() << endl;
    std::ofstream saveMap(SaveMapPath.str().data(), std::ios_base::binary);
    if (!saveMap) {
        cerr << "** Error: Cannot Write to Mapfile: " << mapfile << endl;
        exit(-1);
    }

    cout << "Saving Mapfile: " << SaveMapPath.str() << flush << endl;
    boost::archive::binary_oarchive oa(saveMap, boost::archive::no_header);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    cout << "...one!" << endl;
    saveMap.close();
}

void System::SaveMapPoints(const string &logPath)
{
    unique_lock<mutex> MapPointGlobal(MapPoint::mGlobalMutex);

    std::ostringstream SaveMapPointsPath;
    SaveMapPointsPath << logPath << "_MapPoints.txt";

    std::ofstream SaveMapPointsFile(SaveMapPointsPath.str().data());
    if (!SaveMapPointsFile.is_open()) {
        cerr << "** Error: Cannot Write to MapPoints: " << SaveMapPointsPath.str() << endl;
        exit(-1);
    }

    cout << "\nSaving map points\n...\n";

    vector<MapPoint *> vpMap = mpMap->GetAllMapPoints();
    for (size_t i = 0; i < vpMap.size(); ++i)
    {
        MapPoint *pMap = vpMap[i];

        if (pMap->isBad()) 
            continue;

        cv::Mat worldPos = pMap->GetWorldPos();
        SaveMapPointsFile << setprecision(6) << worldPos.at<float>(0) << " " << worldPos.at<float>(1) << " " << worldPos.at<float>(2) << endl;
    }
    SaveMapPointsFile.close();
    cout << "Saved " << vpMap.size() << " map points in " << SaveMapPointsPath.str() << endl;
}
} //namespace ORB_SLAM

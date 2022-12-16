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
#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"
#include"Optimizer.h"
#include"PnPsolver.h"

#include "glog/logging.h"
#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{
Tracking::Tracking(ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, bool bReuseMap):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    mpLocalMapper= new LocalMapping(mpMap, mpKeyFrameDB, mpORBVocabulary, mSensor==System::MONOCULAR);
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fSettings["Camera.fx"];
    K.at<float>(1,1) = fSettings["Camera.fy"];
    K.at<float>(0,2) = fSettings["Camera.cx"];
    K.at<float>(1,2) = fSettings["Camera.cy"];
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0) fps=30;

    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << K.at<float>(0,0) << endl;
    cout << "- fy: " << K.at<float>(1,1) << endl;
    cout << "- cx: " << K.at<float>(0,2) << endl;
    cout << "- cy: " << K.at<float>(1,2) << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO || sensor==System::RGBD) {
        mThDepth = mbf*(float)fSettings["ThDepth"]/K.at<float>(0,0);
    }

    if(sensor==System::RGBD) {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5) mDepthMapFactor=1;
        else mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    if (bReuseMap)
        mState = LOST;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3) {
        if(mbRGB) {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        } else {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    } else if(mImGray.channels()==4) {
        if(mbRGB) {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        } else {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    vdStereoMatch_ms.push_back(mCurrentFrame.mTimeStereoMatch);
    
    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    imRGB.copyTo(mImGray);
    imDepth = imD;

    if(mImGray.channels()==3) {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    } else if(mImGray.channels()==4) {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    
    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3) {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    } else if(mImGray.channels()==4) {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    vdORBExtract_ms.push_back(mCurrentFrame.mTimeORB_Ext);
    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    // cout << "1" << endl;
    timeStartTrack = std::chrono::steady_clock::now();

    if(mState==NO_IMAGES_YET)
        mState = NOT_INITIALIZED;

    mLastProcessedState=mState;

    if(mState==NOT_INITIALIZED) {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    } else {
        std::chrono::steady_clock::time_point time_StartPosePred = std::chrono::steady_clock::now();
        bool bOK;
        if(!mbOnlyTracking) {
            if(mState==OK) {
                CheckReplacedInLastFrame();
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                    bOK = TrackReferenceKeyFrame();
                 else {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            } else
                bOK = Relocalization();
        } else {
            
            if(mState==LOST) {

                chrono::steady_clock::time_point time_localize_st = chrono::steady_clock::now();
                bOK = Relocalization();
                chrono::steady_clock::time_point time_localize_end = chrono::steady_clock::now();
                timeLocalize = chrono::duration_cast<chrono::duration<double,milli> >(time_localize_end - time_localize_st).count();
            } else {
                if(!mbVO) {
                    if(!mVelocity.empty())
                        bOK = TrackWithMotionModel();
                    else
                        bOK = TrackReferenceKeyFrame();
                } else {
                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty()) {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc) {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO) {
                            for(int i =0; i<mCurrentFrame.N; i++) {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                            }
                        }
                    } else if(bOKReloc)
                        mbVO = false;

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;
        std::chrono::steady_clock::time_point time_EndPosePred = std::chrono::steady_clock::now();
        double timePosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndPosePred - time_StartPosePred).count();
        vdPosePred_ms.push_back(timePosePred);

        if(!mbOnlyTracking) {
            if(bOK)
                bOK = TrackLocalMap();
        } else {
            if(bOK && !mbVO)
                bOK = TrackLocalMap(); }
        std::chrono::steady_clock::time_point time_EndLMTrack = std::chrono::steady_clock::now();
        double timeLMTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLMTrack - time_EndPosePred).count();
        vdLMTrack_ms.push_back(timeLMTrack);
        if(bOK)
            mState = OK;
        else
            mState=LOST;

        mpFrameDrawer->Update(this);

        if(bOK) {
            if(!mLastFrame.mTcw.empty()) {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            } else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            for(int i=0; i<mCurrentFrame.N; i++) {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1) {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++) {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            if(NeedNewKeyFrameCustom()) { 
            // if(NeedNewKeyFrame()) { //this one is the original version of ORB SLAM2
                CreateNewKeyFrame();
                
            } else {
                std::chrono::steady_clock::time_point tmp = std::chrono::steady_clock::now();
                double tmpp = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(tmp - timeStartTrack).count();
                vdTrackTotal_ms.push_back(tmpp);
            }
            
            for(int i=0; i<mCurrentFrame.N;i++) {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        if(mState==LOST) {
            if(mpMap->KeyFramesInMap()<=5) {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }
    if(!mCurrentFrame.mTcw.empty()) {
        if (mbOnlyTracking) 
            mlLocalizedPoses.push_back(mCurrentFrame.mTcw);

        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
    } else {
        if (mbOnlyTracking) 
            mlLocalizedPoses.push_back(mlLocalizedPoses.back());
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
    }
    mlbLost.push_back(mState==LOST);

}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500) {
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
        mpMap->AddKeyFrame(pKFini);

        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0) {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);

                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        mpLocalMapper->Run(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        mpMap->mvpKeyFrameOrigins.push_back(pKFini);
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{
    if(!mpInitializer) {
        if(mCurrentFrame.mvKeys.size()>100) {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    } else {
        if((int)mCurrentFrame.mvKeys.size()<=100) {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        if(nmatches<100) {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw;
        cv::Mat tcw;
        vector<bool> vbTriangulated;

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i]) {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    for(size_t i=0; i<mvIniMatches.size();i++) {
        if(mvIniMatches[i]<0)
            continue;

        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;
    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP]) {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->Run(pKFini);
    mpLocalMapper->Run(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++) {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP) {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
                mLastFrame.mvpMapPoints[i] = pRep;
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    mCurrentFrame.ComputeBoW();

    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++) {
        if(mCurrentFrame.mvpMapPoints[i]) {
            if(mCurrentFrame.mvbOutlier[i]) {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++) {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
            vDepthIdx.push_back(make_pair(z,i));
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
            bCreateNew = true;

        if(bCreateNew) {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        } else
            nPoints++;

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    UpdateLastFrame();
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    if(nmatches<20) {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    Optimizer::PoseOptimization(&mCurrentFrame);

    int nmatchesMap = 0;
    vector<MapPoint*> nowViewingMPs;
    for(int i =0; i<mCurrentFrame.N; i++) {
        if(mCurrentFrame.mvpMapPoints[i]) {
            if(mCurrentFrame.mvbOutlier[i]) {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0) {
                nmatchesMap++;
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                nowViewingMPs.push_back(pMP);
            }
        }
    }
    
    mpMap->SetViewingMapPoints(nowViewingMPs);    

    if(mbOnlyTracking) {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    UpdateLocalMap();
    SearchLocalPoints();

    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    for(int i=0; i<mCurrentFrame.N; i++) {
        if(mCurrentFrame.mvpMapPoints[i]) {
            if(!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking) {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                } else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}

bool Tracking::NeedNewKeyFrame()
{
    std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();

    if(mbOnlyTracking) 
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames) return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2) nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR) {
        for(int i =0; i<mCurrentFrame.N; i++) {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth) {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames;
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();
    double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndNewKF - time_StartNewKF).count();
    vdNewKF_ms.push_back(timeNewKF);

    if((c1a||c1b||c1c)&&c2) return true;
    else return false;
}

/* 
deterministic keyframe insertion criterion that depends on the amount of change
in translation and rotation is used to measure the impact of the proposed point sparsification. 
*/
bool Tracking::NeedNewKeyFrameCustom()
{
    std::chrono::steady_clock::time_point time_StartNewKF = std::chrono::steady_clock::now();

    if(mbOnlyTracking)
        return false;

    cv::Mat Tcr = mCurrentFrame.mTcw*mpLastKeyFrame->GetPoseInverse();
    //condition1: translational difference is larger than certain threshold
    Tdiff = cv::norm(Tcr.rowRange(0,3).col(3));

    //condition2: rotational difference is larger than certain threshold
    Rdiff = cv::norm(Tcr.rowRange(0,3).colRange(0,3)-cv::Mat::eye(3,3, CV_32F));

    insertFlag=(Rdiff > theshR)||(Tdiff > theshT);
    
    std::chrono::steady_clock::time_point time_EndNewKF = std::chrono::steady_clock::now();
    double timeNewKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndNewKF - time_StartNewKF).count();
    vdNewKF_ms.push_back(timeNewKF);
    
    return insertFlag;
}

void Tracking::CreateNewKeyFrame()
{
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR) {
        mCurrentFrame.UpdatePoseMatrices();
        
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
                vDepthIdx.push_back(make_pair(z,i));
        }

        if(!vDepthIdx.empty()) {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++) {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1) {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }
                
                if(bCreateNew) {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);

                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                } else
                    nPoints++;

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }
    
    std::chrono::steady_clock::time_point tt = std::chrono::steady_clock::now();
    double tmpp = chrono::duration_cast<chrono::duration<double,std::milli> >(tt - timeStartTrack).count();
    vdTrackTotal_ms.push_back(tmpp);

    mpLocalMapper->Run(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++) {
        MapPoint* pMP = *vit;
        if(pMP) {
            if(pMP->isBad())
                *vit = static_cast<MapPoint*>(NULL);
            else {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        if(mCurrentFrame.isInFrustum(pMP,0.5)) {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0) {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++) {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++) {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad()) {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++) {
        if(mCurrentFrame.mvpMapPoints[i]) {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad()) {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            } else
                mCurrentFrame.mvpMapPoints[i]=NULL;
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++) {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max) {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++) {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++) {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad()) {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++) {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad()) {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent) {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId) {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax) {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    mCurrentFrame.ComputeBoW();
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++) {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);

            if(nmatches<15) {
                vbDiscarded[i] = true;
                continue;
            } else {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++) {
            if(vbDiscarded[i])
                continue;

            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);
            // cv::Mat Tcw = pSolver->iterate(50,bNoMore,vbInliers,nInliers);

            if(bNoMore) {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            if(!Tcw.empty()) {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++) {
                    if(vbInliers[j]) {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50) {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50) {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50) {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50) {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }
                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
        return false;
    else {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{
    cout << "System Reseting" << endl;
    if(mpViewer) {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    mpLocalMapper->Reset();

    mpKeyFrameDB->clear();
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer) {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

////////////////////////////////////////////////////////////
// below is for logging

double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<int> v_values, double average)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

void Tracking::Stats2File()
{
    ofstream f;
    f.open((mlogPath+"Mapping_Stats.txt").c_str());
    f << fixed << setprecision(6);
    f << "#KF insert[ms], MP creation[ms], LBA[ms], Total[ms]" << endl;
    for(int i=0; i<mpLocalMapper->vdLBA_ms.size(); ++i)
    {
        f << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCreation_ms[i] 
          << "," << mpLocalMapper->vdLBA_ms[i] << "," << "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    f.open((mlogPath+"LBA_Stats.txt").c_str());
    f << fixed << setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for(int i=0; i<Optimizer::getLBA_KFopt().size(); ++i)
    {
        f << Optimizer::getLBA_KFopt()[i] << "," << Optimizer::getLBA_KFfixed()[i] << ","
          << Optimizer::getLBA_MPs()[i] << "," << Optimizer::getLBA_edges()[i] << endl;
    }

    f.close();

    f.open((mlogPath+"GBA_Stats.txt").c_str());
    f << fixed << setprecision(6);
    f << "#GBA time[ms], update map[ms], total[ms] KF[#], MP[#]" << endl;
    for(int i=0; i<mpLocalMapper->mpLoopCloser->vdGBA_ms.size(); ++i)
    {
        f << mpLocalMapper->mpLoopCloser->vdGBA_ms[i] << "," << mpLocalMapper->mpLoopCloser->vdUpdateMap_ms[i] << ","
          << mpLocalMapper->mpLoopCloser->vdFGBATotal_ms[i] << "," << mpLocalMapper->mpLoopCloser->vnGBAKFs[i] << "," 
          << mpLocalMapper->mpLoopCloser->vnGBAMPs[i] << endl;
    }

    f.close();

    f.open((mlogPath+"Time_Check.txt").c_str());
    f << fixed << setprecision(6);

    f << "#Image Rect[ms], ORB ext[ms], Stereo match[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;


    LOG(INFO) << "-------SIZE CHECK-----------" << vdStereoMatch_ms.size() << ", " << vdORBExtract_ms.size() << ", "
              << vdPosePred_ms.size() << ", "  << vdLMTrack_ms.size() << ", "  << vdNewKF_ms.size() << ", " << vdTrackTotal_ms.size();
    for(int i=0; i<vdTrackTotal_ms.size(); ++i)
    {
        double stereo_rect = 0.0;
        if(!vdRectStereo_ms.empty())
        {
            stereo_rect = vdRectStereo_ms[i];
        }

        double stereo_match = 0.0;
        if(!vdStereoMatch_ms.empty())
        {
            stereo_match = vdStereoMatch_ms[i];
        }

        f << stereo_rect <<  "," << vdORBExtract_ms[i] << "," << stereo_match << ","
          << vdPosePred_ms[i] <<  "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << "," << vdTrackTotal_ms[i] << endl;
    }

    f.close();

    f.open((mlogPath+"connect_Stats.txt").c_str());
    for (const auto& e: connection_arr)
    {
        f << e << "\n";
    }

    f.close();

    f.open((mlogPath+"timeD_Stats.txt").c_str());
    for (const auto& e: diff_time_arr)
    {
        f << e << "\n";
    }
    f.close();
}


void Tracking::PrintTimeStats(const string &resultPath, const string &logPath)
{
    if (mbOnlyTracking)
        return;

    mlogPath = logPath;
    vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    int num_mapPoints = vpMPs.size();
    int num_keyFrames = vpKFs.size();

    connection_arr.reserve(num_mapPoints);
    diff_time_arr.reserve(num_mapPoints);
    spatial_div_arr.reserve(num_keyFrames);
    
    for (auto mp : vpMPs)
    {
        map<KeyFrame*, size_t> observations = mp->GetObservations();

        connection_arr.push_back(observations.size());
        int max_frameId=0;
        double min_frameId=9e+10;
        
        for (map<KeyFrame*, size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* mkf = mit->first;
            if(mkf->mnFrameId > max_frameId) max_frameId = mkf->mnFrameId;
            if(mkf->mnFrameId < min_frameId) min_frameId = mkf->mnFrameId;
        }

        diff_time_arr.push_back(max_frameId - min_frameId);
    }

    // spatial diversity (calculated by occupied cell number of grid)
    for (auto& kf: vpKFs)
    {
        // cout << "occupied num: " << kf->CountOccupiedGrid() << endl;
        double rate=static_cast<double>(kf->CountOccupiedGrid())/static_cast<double>(kf->getGridNum())*100;
        spatial_div_arr.push_back(rate);
    }

    Stats2File();

    ofstream f;
    f.open(resultPath.c_str(), ios::app | ios::ate);
    f << fixed;
    
    //Tracking
    double average, deviation;
    if(!vdRectStereo_ms.empty()) {
        average = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        f << average << "(" << deviation << ")" << ",";
    } else{
        f << 0 << "(" << 0 << ")" << ",";
    }

    average = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    f << average << "(" << deviation << ")" << ",";

    if(!vdStereoMatch_ms.empty()) {
        average = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        f << average << "(" << deviation << ")" << ",";
    } else {
        f << 0 << "(" << 0 << ")" << ",";
    }

    average = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    f << average << "(" << deviation << ")" << ",";

   //Mapping
    average = calcAverage(mpLocalMapper->vdKFInsert_ms);
    deviation = calcDeviation(mpLocalMapper->vdKFInsert_ms, average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(mpLocalMapper->vdMPCreation_ms);
    deviation = calcDeviation(mpLocalMapper->vdMPCreation_ms, average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(mpLocalMapper->vdLBA_ms);
    deviation = calcDeviation(mpLocalMapper->vdLBA_ms, average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(mpLocalMapper->vdLMTotal_ms);
    deviation = calcDeviation(mpLocalMapper->vdLMTotal_ms, average);
    f << average << "(" << deviation << ")" << ",";

    f << mpLocalMapper->vdLBA_ms.size() << ",";

    // LBA
    average = calcAverage(Optimizer::getLBA_edges());
    deviation = calcDeviation(Optimizer::getLBA_edges(), average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(Optimizer::getLBA_KFopt());
    deviation = calcDeviation(Optimizer::getLBA_KFopt(), average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(Optimizer::getLBA_KFfixed());
    deviation = calcDeviation(Optimizer::getLBA_KFfixed(), average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(Optimizer::getLBA_MPs());
    deviation = calcDeviation(Optimizer::getLBA_MPs(), average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(Optimizer::getcullTime());
    deviation = calcDeviation(Optimizer::getcullTime(), average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(Optimizer::getcullRatioStack());
    deviation = calcDeviation(Optimizer::getcullRatioStack(), average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(Optimizer::getLBATime());
    deviation = calcDeviation(Optimizer::getLBATime(), average);
    f << average << "(" << deviation << ")" << ",";

    // 추가. ratio w.r.t number of keyframe
    average = calcAverage(connection_arr);
    deviation = calcDeviation(connection_arr, average);
    f << average/num_keyFrames*100 << "(" << deviation/num_keyFrames*100 << ")" << ",";

    average = calcAverage(diff_time_arr);
    deviation = calcDeviation(diff_time_arr, average);
    f << average << "(" << deviation << ")" << ",";

    average = calcAverage(spatial_div_arr);
    deviation = calcDeviation(spatial_div_arr, average);
    f << average << "(" << deviation << ")" << ",";

    // Map Size
    f << num_keyFrames << ",";
    f << num_mapPoints << ",";
    
    //Loop
    average = calcAverage(mpLocalMapper->mpLoopCloser->vdLoopFusion_ms);
    deviation = calcDeviation(mpLocalMapper->mpLoopCloser->vdLoopFusion_ms, average);
    f << average << "(" << deviation << ")" << ",";
    
    average = calcAverage(mpLocalMapper->mpLoopCloser->vdLoopOptEss_ms);
    deviation = calcDeviation(mpLocalMapper->mpLoopCloser->vdLoopOptEss_ms, average);
    f << average << "(" << deviation << ")" << ",";
    
    average = calcAverage(mpLocalMapper->mpLoopCloser->vdLoopTotal_ms);
    deviation = calcDeviation(mpLocalMapper->mpLoopCloser->vdLoopTotal_ms, average);
    f << average << "(" << deviation << ")" << ",";

    //Loop info
    f << mpLocalMapper->mpLoopCloser->vdLoopTotal_ms.size() << ",";
    
    average = calcAverage(mpLocalMapper->mpLoopCloser->vnLoopKFs);
    deviation = calcDeviation(mpLocalMapper->mpLoopCloser->vnLoopKFs, average);
    f << average << "(" << deviation << ")" << ",";

    //GBA
    average = calcAverage(mpLocalMapper->mpLoopCloser->vdGBA_ms);
    deviation = calcDeviation(mpLocalMapper->mpLoopCloser->vdGBA_ms, average);
    f << average << "(" << deviation << ")" << ",";
    
    average = calcAverage(mpLocalMapper->mpLoopCloser->vdUpdateMap_ms);
    deviation = calcDeviation(mpLocalMapper->mpLoopCloser->vdUpdateMap_ms, average);
    f << average << "(" << deviation << ")" << ",";
    
    average = calcAverage(mpLocalMapper->mpLoopCloser->vdFGBATotal_ms);
    deviation = calcDeviation(mpLocalMapper->mpLoopCloser->vdFGBATotal_ms, average);
    f << average << "(" << deviation << ")" << ",";

    f << mpLocalMapper->mpLoopCloser->vdFGBATotal_ms.size() << ",";
    
    average = calcAverage(mpLocalMapper->mpLoopCloser->vnGBAKFs);
    deviation = calcDeviation(mpLocalMapper->mpLoopCloser->vnGBAKFs, average);
    f << average << "(" << deviation << ")" << ",";
    
    average = calcAverage(mpLocalMapper->mpLoopCloser->vnGBAMPs);
    deviation = calcDeviation(mpLocalMapper->mpLoopCloser->vnGBAMPs, average);
    f << average << "(" << deviation << ")" << ", ";

    //number of frames that tracking failed
    int LostCount=0;
    for(list<bool>::iterator lst=mlbLost.begin(), lend=mlbLost.end();lst!=lend;lst++)
    {
        if(*lst) LostCount++;
    }
    // f << LostCount << endl;
    f << LostCount << ", ";

    //to check culled mappoint num- ratio
    f << mpMap->GetCulledMapPoints().size() << endl;

    

    f.close();
}

// #endif

} //namespace ORB_SLAM


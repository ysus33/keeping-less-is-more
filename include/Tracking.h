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
#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class System;

class Tracking
{  

public:
    Tracking(ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, bool bReuseMap);

    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetViewer(Viewer* pViewer);
    void InformOnlyTracking(const bool &flag);
    string mlogPath;

public:
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    int mSensor;
    static const float &theshR;
    static const float &theshT;

    double Rdiff;
    double Tdiff;
    bool insertFlag;

    Frame mCurrentFrame;
    cv::Mat mImGray;
    cv::Mat imDepth;


    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    list<cv::Mat> mlRelativeFramePoses;
    list<cv::Mat> mlLocalizedPoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    bool mbOnlyTracking;

    void Reset();

    // for statistics
    void Stats2File();
    void PrintTimeStats(const string &resultPath, const string &logPath);
    vector<double> vdRectStereo_ms;
    vector<double> vdORBExtract_ms; 
    vector<double> vdStereoMatch_ms; 
    vector<double> vdPosePred_ms; 
    vector<double> vdLMTrack_ms; 
    vector<double> vdNewKF_ms; 
    vector<double> vdTrackTotal_ms;
    vector<int> connection_arr;
    vector<int> diff_time_arr;
    vector<double> spatial_div_arr;
    vector<double> vdTrackTotalwhenLBA_ms;
    double timeLocalize;
    chrono::steady_clock::time_point timeStartTrack;

protected:
    void Track();
    void StereoInitialization();
    void MonocularInitialization();
    void CreateInitialMapMonocular();
    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrameCustom();
    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    bool mbVO;

    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Map* mpMap;
    LocalMapping* mpLocalMapper;

    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    int mMinFrames;
    int mMaxFrames;

    float mThDepth;
    float mDepthMapFactor;
    int mnMatchesInliers;

    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    cv::Mat mVelocity;

    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};
} //namespace ORB_SLAM

#endif // TRACKING_H

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
#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"


namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;

class System
{
public:
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    void ActivateLocalizationMode();
    void DeactivateLocalizationMode();

    void Shutdown();

    void SaveStats(const string &resultPath, const string &logPath);

    void SaveTrajectoryTUM(const string &filename);
    void SaveKeyFrameTrajectoryTUM(const string &filename);
    void SaveTrajectoryKITTI(const string &filename);

    void InsertRectTime(double& time);
    void InsertTrackTime(double& time);

    bool LoadMap();
    void SaveMap(const string &logPath);
    void SaveMapPoints(const string &logPath);
    string mapfile;

private:
    eSensor mSensor;
    ORBVocabulary* mpVocabulary;
    KeyFrameDatabase* mpKeyFrameDatabase;
    Map* mpMap;
    Tracking* mpTracker;
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    std::thread* mptViewer;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H

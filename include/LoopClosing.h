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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "Map.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <mutex>

namespace ORB_SLAM2
{

class KeyFrameDatabase;

class LoopClosing
{
public:
    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

public:
    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void Run(KeyFrame *pKF);
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);
    void Reset();

    vector<float> GlobalBAtimes;

    vector<double> vdLoopFusion_ms; 
    vector<double> vdLoopOptEss_ms; 
    vector<double> vdLoopTotal_ms; 
    vector<int> vnLoopKFs; 

    vector<double> vdGBA_ms; 
    vector<double> vdUpdateMap_ms; 
    vector<double> vdFGBATotal_ms; 
    vector<int> vnGBAKFs; 
    vector<int> vnGBAMPs; 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    bool DetectLoop();
    bool ComputeSim3();
    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);
    void CorrectLoop();

    Map* mpMap;
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    bool mbFixScale;
    bool mnFullBAIdx;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H

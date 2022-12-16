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
#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "System.h"
#include "LoopClosing.h"
#include "MatchCuller.h"
#include "Converter.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;


class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
   
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, const unsigned long nLoopKF=0, const bool bRobust = true);
    
    void static LocalBundleAdjustment(KeyFrame *pKF, Map* pMap, int maxNum, bool flg);
    
    void static LocalBundleAdjustment(KeyFrame *pKF, Map* pMap, int maxNum, bool flg, int window);
    
    int static PoseOptimization(Frame* pFrame);

    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);


    // for statistics
    void static AddcullRatio(int ratio)
    {
        CullRatioStack.push_back(ratio);
    }
    std::vector<int> static getcullRatioStack()
    {
        return CullRatioStack;
    }

    void static AddcullTime(double rtime)
    {
        CullTimeStack.push_back(rtime);
    }
    std::vector<double> static getcullTime()
    {
        return CullTimeStack;
    }

    void static AddLBATime(double rtime)
    {
        LBATimeStack.push_back(rtime);
    }
    std::vector<double> static getLBATime()
    {
        return LBATimeStack;
    }

    void static AddLBA_edges(int num)
    {
        vnLBA_edges.push_back(num);
    }
    std::vector<int> static getLBA_edges()
    {
        return vnLBA_edges;
    }

    void static AddLBA_KFopt(int num)
    {
        vnLBA_KFopt.push_back(num);
    }
    std::vector<int> static getLBA_KFopt()
    {
        return vnLBA_KFopt;
    }

    void static AddLBA_KFfixed(int num)
    {
        vnLBA_KFfixed.push_back(num);
    }
    std::vector<int> static getLBA_KFfixed()
    {
        return vnLBA_KFfixed;
    }

    void static AddLBA_MPs(int num)
    {
        vnLBA_MPs.push_back(num);
    }
    std::vector<int> static getLBA_MPs()
    {
        return vnLBA_MPs;
    }

private:
    static std::vector<int> CullRatioStack;
    static std::vector<double> CullTimeStack;
    static std::vector<double> LBATimeStack;
    static std::vector<int> vnLBA_edges; 
    static std::vector<int> vnLBA_KFopt; 
    static std::vector<int> vnLBA_KFfixed; 
    static std::vector<int> vnLBA_MPs; 
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H

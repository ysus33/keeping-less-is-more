/*
* This file is an implementation of "Keeping Less is More: Point Sparsification for Visual SLAM"
* Based on ORB-SLAM2 <https://github.com/raulmur/ORB_SLAM2>
*/
#include "MatchCuller.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

using operations_research::ArcIndex;
using operations_research::CostValue;
using operations_research::FlowQuantity;
using operations_research::Graphs;
using operations_research::NodeIndex;
using operations_research::SimpleMinCostFlow;
using operations_research::MTRandom;


namespace ORB_SLAM2 {

template <typename T> 
MatchCuller<T>::MatchCuller(int max_points_per_pair)
    : max_points_per_pair_(max_points_per_pair),
      largest_point_size_(0),
      point_index_(0),
      next_node_(0),
      source_node_(-1),
      sink_node_(-1)
{
  CHECK_GT(max_points_per_pair_, 0);
}

template <typename T> 
MatchCuller<T>::~MatchCuller() 
{}

static FlowQuantity ComputeCapacityForSourcePoint(const int point_size) 
{
    const int pose_pair_size = point_size * (point_size - 1) / 2;
    return pose_pair_size;
}

static CostValue ComputeCostForPointSize(const int largest_point_size, const int point_size) 
{
  if (largest_point_size == point_size) 
    return 1;

  if (point_size < 2) 
    return numeric_limits<int>::max();

  CHECK(point_size < largest_point_size);
  CostValue cost_size_one = ComputeCostForPointSize(largest_point_size, point_size + 1);
  return ceil(static_cast<double>(point_size + 1) / (point_size - 1) * cost_size_one);
}

static CostValue ComputeCostForSourcePoint(const int largest_point_size, const int point_size) 
{
	return ComputeCostForPointSize(largest_point_size, point_size);
}

static CostValue ComputeCostForPointPosePair(MapPoint* mp, KeyFrame* pose_1, KeyFrame* pose_2) 
{
    int neighbor1 = mp->GetNeighbors(pose_1);
    int neighbor2 = mp->GetNeighbors(pose_2);

    int neighbor = neighbor1 * neighbor2; 
    int cost;
    if (neighbor < 100) 
        cost = 0;
    else if (neighbor < 500) 
        cost = 1;
    else if (neighbor < 1200) 
        cost = 2;
    else 
        cost = 3;

    return cost * 2;
}

static CostValue ComputeCostForPosePairSink(KeyFrame* pose_1, KeyFrame* pose_2)
{ 
    cv::Mat Ow1 = pose_1->GetCameraCenter();
    cv::Mat Ow2 = pose_2->GetCameraCenter();

    cv::Mat vBaseline = Ow2-Ow1;
    const float baseline = cv::norm(vBaseline);

    int cost;
    if (baseline < 10) cost = 10;
    else if (baseline < 20) cost = 9;
    else if (baseline < 30) cost = 8;
    else if (baseline < 40) cost = 7;
    else if (baseline < 50) cost = 6;
    else if (baseline < 60) cost = 5;
    else if (baseline < 70) cost = 4;
    else if (baseline < 80) cost = 3;
    else if (baseline < 90) cost = 2;
    else if (baseline < 100) cost = 1;
    else if (baseline < 200) cost = 0;
    
    // LOG(INFO) << cost;
    return cost * 10; 
}

template <typename T> 
NodeIndex MatchCuller<T>::AllocateNode() { return next_node_++;}

template <typename T> 
void MatchCuller<T>::cull(T MapPoints) 
{
    BuildProblem(MapPoints);

    SimpleMinCostFlow::Status status = min_cost_flow_.SolveMaxFlowWithMinCost();
    CHECK_EQ(status, SimpleMinCostFlow::OPTIMAL);

    CullPoints();
}

template <typename T> 
void MatchCuller<T>::CullPoints()
{
    afterNum=0;
    for (unordered_map<int, operations_research::ArcIndex>::iterator it=point_edge_index_.begin(), itend=point_edge_index_.end(); it!=itend; it++)
    {
        int idx = it->first;
        const ArcIndex edge_index = it->second;

        FlowQuantity capacity = min_cost_flow_.Capacity(edge_index);
        FlowQuantity flow = min_cost_flow_.Flow(edge_index);
        FlowQuantity threshold = capacity/2;

        if (flow > threshold)
          afterNum++;
        else
          point_ptr_index[idx]->SetCullFlag(); 
        //   point_ptr_index[idx]->SetBadFlag(); 
    }
}

template <typename T> 
void MatchCuller<T>::BuildProblem(const T& MapPoints) 
{
    source_node_ = AllocateNode();
    sink_node_ = AllocateNode();

    int idx = 0;
    for (auto &lit : MapPoints)
    {
        if(!lit) continue;
        if(lit->isBad()) continue;
        map<KeyFrame*, size_t> observations = lit->GetObservations();
        if (observations.size()>1) {
            point_ptr_index[idx] = lit;
            largest_point_size_ = max<int>(largest_point_size_, observations.size());
            idx ++;
        }
    }
    beforeNum = idx;


    const int num_points = point_ptr_index.size();
    const int max_point_edge = largest_point_size_ * (largest_point_size_ - 1) / 2;
    min_cost_flow_.SetNodeSupply(source_node_, num_points * max_point_edge);
    min_cost_flow_.SetNodeSupply(sink_node_, -num_points * max_point_edge);

    for (auto & pt: point_ptr_index)
    {
        AddEdgesForPoint(&pt);
    }
}

template <typename T> 
void MatchCuller<T>::AddEdgesForPoint(pair<const int, MapPoint *> *ptr_idx)
{
    map<KeyFrame*, size_t> observations = ptr_idx->second->GetObservations();

    int NconnectedKF = observations.size();
    const NodeIndex Point_node = AllocateNode();
    const ArcIndex Source_Point_edge 
        = min_cost_flow_.AddArcWithCapacityAndUnitCost(source_node_,
                                                       Point_node,
                                                       ComputeCapacityForSourcePoint(NconnectedKF), 
                                                       ComputeCostForSourcePoint(largest_point_size_, NconnectedKF));
    
    gtl::InsertOrDie(&point_edge_index_, ptr_idx->first, Source_Point_edge);

    for (map<KeyFrame*, size_t>::iterator ob1=observations.begin(), ob1end=observations.end(); ob1!=ob1end; ob1++)
    {
        KeyFrame* p1 = ob1->first;
        for (map<KeyFrame*, size_t>::iterator ob2=observations.begin(), ob2end=observations.end(); ob2!=ob2end; ob2++)
        {
            KeyFrame* p2 = ob2->first;
            if (p1->mnFrameId <= p2->mnFrameId) continue; //p1>p2 (frame id)

            pair<int, int> pose_pair = make_pair(p1->mnFrameId, p2->mnFrameId);

            NodeIndex PosePair_node;
            if (!gtl::ContainsKey(pose_pair_node_index_, pose_pair)) {
                PosePair_node = AllocateNode();
                pose_pair_node_index_[pose_pair] = PosePair_node;
                min_cost_flow_.AddArcWithCapacityAndUnitCost(PosePair_node, 
                                                             sink_node_,
                                                             max_points_per_pair_,
                                                             ComputeCostForPosePairSink(p1, p2));
            } else {
                PosePair_node = gtl::FindOrDieNoPrint(pose_pair_node_index_, pose_pair);
            }

            min_cost_flow_.AddArcWithCapacityAndUnitCost(Point_node, 
                                                         PosePair_node, 
                                                         1,
                                                         ComputeCostForPointPosePair(ptr_idx->second, p1, p2));
        }
    }
}

template class MatchCuller<vector<MapPoint *>>;
template class MatchCuller<list<MapPoint *>>;

}

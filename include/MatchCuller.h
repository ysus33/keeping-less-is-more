/*
* This file is an implementation of "Keeping Less is More: Point Sparsification for Visual SLAM"
* Based on ORB-SLAM2 <https://github.com/raulmur/ORB_SLAM2>
*/

#ifndef MATCH_CULLER_H_
#define MATCH_CULLER_H_

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Map.h"

#include "ortools/base/macros.h"
#include "ortools/base/map_util.h"
#include "ortools/base/hash.h"
#include "ortools/base/random.h"
#include "ortools/graph/graphs.h"
#include "ortools/graph/ebert_graph.h"
#include "ortools/graph/min_cost_flow.h"

using namespace std;

namespace ORB_SLAM2 
{
template <typename T>
class MatchCuller
{
public:
  explicit MatchCuller(int max_points_per_pair);
  ~MatchCuller();

  void cull(T MapPoints);

  int getCullRatio()
  { 
    return static_cast<float>(afterNum)/static_cast<float>(beforeNum) * 100;
  };

private:
  void BuildProblem(const T& MapPoints);
  void AddEdgesForPoint(pair<const int, MapPoint *> *ptr_idx);
  void CullPoints();

  unordered_map<int, MapPoint*> point_ptr_index;
  unordered_map<int, operations_research::ArcIndex> point_edge_index_;
  unordered_map<pair<int, int>, operations_research::NodeIndex> pose_pair_node_index_;

  operations_research::NodeIndex AllocateNode();
  operations_research::NodeIndex next_node_;
  operations_research::NodeIndex source_node_;
  operations_research::NodeIndex sink_node_;
  operations_research::SimpleMinCostFlow min_cost_flow_;

  int max_points_per_pair_;
  int largest_point_size_;
  int point_index_;
  int beforeNum;
  int afterNum;
};

}  // namespace ORB_SLAM2

#endif MATCH_CULLER_H_
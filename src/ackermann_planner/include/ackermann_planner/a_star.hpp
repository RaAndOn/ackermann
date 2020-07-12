#pragma once

#include <boost/functional/hash.hpp>
#include <boost/optional.hpp>
#include <functional>
#include <iostream>
#include <queue> // std::priority_queue
#include <unordered_map>
#include <unordered_set>
// #include <utility> // std::pair
#include <vector> // std::vector

#include <ros/ros.h>

#include <ackermann_planner/motion_primitive.hpp>
#include <ackermann_planner/planner_utils.hpp>

using FCost = double;
using NodeIndex = unsigned;

struct Node {
  State m_state;
  boost::optional<NodeIndex> m_parentIndex;
  double m_gCost;
  bool m_closed;
  NodeIndex m_index;
  Node(const State &state, const NodeIndex index,
       const boost::optional<NodeIndex> parentIndex, const double gCost)
      : m_state{state}, m_index{index},
        m_parentIndex{parentIndex}, m_gCost{gCost}, m_closed{false} {}
};

using Path = std::vector<State>;
using NodeRank = std::pair<FCost, NodeIndex>; // FCost and NodeIndex
using OpenList =
    std::priority_queue<NodeRank, std::vector<NodeRank>,
                        std::greater<NodeRank>>;  // Min priority queue
using ClosedList = std::unordered_set<NodeIndex>; // Constant time lookup
using Graph = std::unordered_map<NodeIndex, Node>;
using Heuristic = std::function<double(const State &state)>;
using EdgeCost = std::function<double(const Primitive &primitive)>;
// using Hash = std::function<NodeIndex(const State &state)>;

class AStar {
public:
  /// @brief Implementation of A* planner for an ackermann vehicle with X,Y
  /// position, steering angle of Theta.

  /// It is assumed in the design of this class that the 2D array overwhich this
  /// operates is 0 indexed and always positive
  AStar(const std::vector<Primitive> &primitives,
        const double distanceThresholdMeters,
        const double angularThresholdDegrees,
        const double distanceResolutionMeters,
        const double angularResolutionDegrees);

  ~AStar();

  /// @brief This function performs an A* search using the given parameters to
  /// define the search parameters
  boost::optional<Path> astar(const State &startState, const State &goalState,
                              const int epsilon,
                              const std::string &heuristicFunction,
                              const std::string &edgeCostFunction);

  /// @brief This is a modified version of the Szudzik Pairing algorithm which
  /// takes a pair of integers and makes them into a unique integer.
  /// https://www.vertexfragment.com/ramblings/cantor-szudzik-pairing-functions/
  inline NodeIndex signedSzudzikPair(const int x, const int y) {
    // Modification to Sudzik Pairing algorithm for negative numbers
    const int a = (x >= 0.0 ? 2.0 * x : (-2.0 * x) - 1.0);
    const int b = (y >= 0.0 ? 2.0 * y : (-2.0 * y) - 1.0);
    // Szudzik Pairing Algorithm
    return (a >= b ? (a * a) + a + b : (b * b) + a);
    // * 0.5 <- Removed to ensure numbers are ints
  }

  /// @brief This function computes a unique index for a state
  /// The state is rounded based on the resolution
  inline NodeIndex hashFunction(const State &state) {
    const int theta{static_cast<int>(state.m_theta / m_angularResolution)};
    const int x{static_cast<int>(state.m_x / m_distanceResolution)};
    const int y{static_cast<int>(state.m_y / m_distanceResolution)};

    const NodeIndex index1{signedSzudzikPair(x, y)};
    const NodeIndex index2{signedSzudzikPair(index1, theta)};
    if (index2 < 0) {
      ROS_ERROR("ERROR: hashFunction Calculated "
                "Node index is negative, "
                "must be positive");
      throw "";
    }
    return index2;
  };

private:
  OpenList m_openList;
  Graph m_nodeGraph;
  // boost::optional<Graph> m_heuristicGraph;
  int m_epsilon;
  int m_collisionThresh;
  Heuristic m_heuristicFunction;
  EdgeCost m_edgeCostFunction;
  // Hash m_hashFunction;

  boost::optional<State> m_goalState;
  NodeIndex m_startIndex;

  int m_direction;

  double m_distanceThreshold;  // Meters
  double m_angularThreshold;   // Radians
  double m_distanceResolution; // Meters
  double m_angularResolution;  // Radians

  const std::vector<Primitive> m_primitives;

  /// @brief Given a node, this function finds any legitimate successors and
  /// adds them to the node graph
  void getSuccessors(const Node &currentNode);

  bool checkIfSameState(const State &state1, const State &state2);

  /// @brief The function will find and return a the node in the graph
  /// correspinding to an index
  boost::optional<Node &> getNode(const NodeIndex index);

  boost::optional<Path> getPath(const NodeIndex &goalIndex);

  void setHeuristicFunction(const std::string &heuristicFunction);

  void setEdgeCostFunction(const std::string &edgeCostFunction);

  /// @brief Function to safely calculate fCost, avoiding INT rollover
  inline FCost addFCost(const Node &node) {
    const FCost fCost{node.m_gCost +
                      m_heuristicFunction(node.m_state) * m_epsilon};
    if (fCost < 0) {
      ROS_ERROR("ERROR: FCost was negative throwing");
      throw "";
    }
    return fCost;
  }
};
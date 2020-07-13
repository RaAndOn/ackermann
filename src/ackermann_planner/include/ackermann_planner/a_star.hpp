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
using NodeIndex = long long int;

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
                        std::greater<NodeRank>>; // Min priority queue
using Graph = std::unordered_map<NodeIndex, Node>;
using Heuristic = std::function<double(const State &state)>;
using EdgeCost = std::function<double(const Primitive &primitive)>;

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

  /// @brief This is the Szudzik Pairing algorithm which takes a pair of
  /// positive integers and makes them into a unique positive integer.
  /// https://www.vertexfragment.com/ramblings/cantor-szudzik-pairing-functions/
  inline NodeIndex szudzikPair(const NodeIndex x, const NodeIndex y) const {
    return (x >= y ? (x * x) + x + y : (y * y) + x);
  }

  /// @brief This is a modified version of the Szudzik Pairing algorithm which
  /// takes a pair of signed integers and makes them into a unique positive
  /// integer.
  /// https://www.vertexfragment.com/ramblings/cantor-szudzik-pairing-functions/
  inline NodeIndex signedSzudzikPair(const NodeIndex x,
                                     const NodeIndex y) const {
    // Modification to Sudzik Pairing algorithm for negative numbers
    const NodeIndex a = (x >= 0.0 ? 2.0 * x : (-2.0 * x) - 1.0);
    const NodeIndex b = (y >= 0.0 ? 2.0 * y : (-2.0 * y) - 1.0);
    // Szudzik Pairing Algorithm

    return szudzikPair(a, b);
    // * 0.5 <- Removed to ensure numbers are ints
  }

  /// @brief This function computes a unique index for a state
  /// The state is rounded based on the resolution
  inline NodeIndex hashFunction(const State &state) const {
    // Make theta positive so we can use szudzik and get a tighter packing
    const NodeIndex theta{
        static_cast<int>((state.m_theta + M_PI) / m_angularResolution)};
    if (theta < 0) {
      ROS_ERROR("Theta is negative");
      throw "";
    }
    const int x{static_cast<int>(state.m_x / m_distanceResolution)};
    const int y{static_cast<int>(state.m_y / m_distanceResolution)};

    const NodeIndex index1{signedSzudzikPair(x, y)};
    if (index1 < 0) {
      ROS_ERROR("index1 is negative");
      throw "";
    }
    const NodeIndex index2{szudzikPair(index1, theta)};
    if (index2 < 0) {
      ROS_ERROR("ERROR: hashFunction Calculated "
                "Node index is negative, "
                "must be positive");
      throw "";
    }
    return (state.m_gear == Gear::FORWARD) ? index2 : -index2;
  };

private:
  OpenList m_openList;
  Graph m_nodeGraph;
  int m_epsilon;
  int m_collisionThresh;
  Heuristic m_heuristicFunction;
  EdgeCost m_edgeCostFunction;

  boost::optional<State> m_goalState;
  NodeIndex m_startIndex;

  // Thresholds for determining if states are the same
  double m_distanceThresholdSquared; // Meters
  double m_angularThreshold;         // Radians

  // Resolution for discretizing state into integers
  double m_distanceResolution; // Meters
  double m_angularResolution;  // Radians

  const std::vector<Primitive> m_primitives;

  /// @brief Given a node, this function finds any legitimate successors and
  /// adds them to the node graph
  /// @param currentNode node whose successors should be gotten
  void getSuccessors(const Node &currentNode);

  /// @brief Given a two states compare them and see if they are within the
  /// threshold to be the same states
  /// @param state1 The first state to be compared
  /// @param state2 The second state to be compared
  /// @return Boolean indicating whether two states are within in the threshold
  /// to be considered the same
  bool checkIfSameState(const State &state1, const State &state2);

  /// @brief The function will find and return a the node in the graph
  /// correspinding to an index
  /// @param index the index of the node wanted
  /// @return The node associated index, or boost::none if the node is not in
  /// the graph
  boost::optional<Node &> getNode(const NodeIndex index);

  /// @brief Get the path from the start state to the end index
  /// @param endIndex is the index of the node at the end of the path
  /// @return the path from the start state to the end of the path, returning
  /// boost::none if there is no path
  boost::optional<Path> getPath(const NodeIndex &endIndex);

  /// @brief Set a heuristic lambda function variable to the indicated lambda
  /// function
  /// @param heuristicFunction indication of which heuristic function to use
  void setHeuristicFunction(const std::string &heuristicFunction);

  /// @brief Set a edge cost lambda function variable to the indicated lambda
  /// function
  /// @param edgeCostFunction indication of which edge cost function to use
  void setEdgeCostFunction(const std::string &edgeCostFunction);

  /// @brief Function to safely calculate F-cost, avoiding INT rollover
  /// @param node the predicted F-cost of the node
  /// @return Calculated F-cost
  inline FCost addFCost(const Node &node) const {
    const FCost fCost{node.m_gCost +
                      m_heuristicFunction(node.m_state) * m_epsilon};
    if (fCost < 0) {
      ROS_ERROR("ERROR: FCost was negative throwing");
      throw "";
    }
    return fCost;
  }
};
#pragma once

#include <functional>
#include <iostream>
#include <queue> // std::priority_queue
#include <unordered_set>

#include <ros/ros.h>

#include <ackermann_planner/motion_primitive.hpp>
#include <ackermann_planner/search_class.hpp>
#include <ackermann_project/planner_utils.hpp>

using NodeRank = std::pair<Cost, NodeIndex>; // Cost and NodeIndex
using OpenList =
    std::priority_queue<NodeRank, std::vector<NodeRank>,
                        std::greater<NodeRank>>; // Min priority queue
using ClosedList = std::unordered_set<NodeIndex>;
using Heuristic = std::function<double(const State &state)>;
using EdgeCost = std::function<double(const Primitive &primitive)>;

class AStar : public SearchClass {
public:
  /// @brief Implementation of A* planner for an ackermann vehicle with X,Y
  /// position, steering angle of Theta.
  /// @param primitives vector of the primitives which are used for node
  /// expansion
  /// @param distanceResolutionMeters linear resolution used for discretizing
  /// and indexing states
  /// @param angularResolutionDegrees angular resolution used for discretizing
  /// and indexing states
  /// @param turningRadius
  /// @param epsilon Amount to weight the heuristic
  /// @param heuristicFunction Name of the heuristic function, which will be
  /// mapped to a lambda variable
  /// @param edgeCostFunction Name of the edge cost function, which will be
  /// mapped to a lambda variable
  /// @param debug Flag to indicate whether to use certain print statements
  AStar(std::vector<Primitive> primitives,
        const double distanceResolutionMeters,
        const double angularResolutionDegrees, const double turningRadius,
        const double epsilon, const std::string &heuristicFunction,
        const std::string &edgeCostFunction, const bool debug = true);

  ~AStar();

  /// @brief This public function performs a search from the start state to the
  /// goal state. The search it performs is based on the parameters provided at
  /// class initialization
  /// @param startState State from which the search is starting
  /// @param goalState State to which the search is planning
  /// @return Path from the start to goal state
  boost::optional<Path> search(const State &startState, const State &goalState);

  /// @brief Returns the graph created in the search
  /// @return The node graph
  Graph getGraph() const { return m_nodeGraph; }

  /// @brief Returns the size of the graph created in the search
  /// @return The size of the graph created in the search
  size_t getGraphSize() const { return m_nodeGraph.size(); }

  /// @brief Returns the time in seconds taken to perform latest search
  /// @return The time in seconds taken to perform latest search
  double getLatestSearchTime() const { return m_latestSearchTime; }

protected:
  /// @brief Node graph containing all nodes and their relationships
  Graph m_nodeGraph;

  /// @brief Open list for tracking which state is to be expanded next
  OpenList m_openList;

  /// @brief Closed list to indicate which states have already been expanded
  ClosedList m_closedList;

  /// @brief Weight on the heuristic when calculating the F cost
  double m_epsilon;

  /// @brief UNUSED
  int m_collisionThresh;

  /// @brief Lambda function representing the admissable heuristic function
  Heuristic m_heuristicFunction;

  /// @brief Lambda function holding the chosen edge cost function
  EdgeCost m_edgeCostFunction;

  /// @brief Goal state which is being planned towards
  boost::optional<State> m_goalState;

  /// @brief Index of the state which planning is starting from
  NodeIndex m_startIndex{};

  /// @brief Linear resolution for discretizing state into integers (meters)
  double m_distanceResolution;

  /// @brief Angular resolution for discretizing state into integers (radians)
  double m_angularResolution;

  /// @brief Vector of the primitives which are used for node expansion
  const std::vector<Primitive> m_primitives;

  /// @brief The amount of time taken to perform the latest search
  double m_latestSearchTime{};

  /// @brief Given a node, this function finds any legitimate successors and
  /// adds them to the node graph
  /// @param currentNode node whose successors should be gotten
  void getSuccessors(const Node &currentNode, const Heuristic &heuristic);

  /// @brief A* search (Primary function of the class)
  /// @param goalNode Node to which A* is trying to find a path
  bool aStar(const Node &goalNode);

  /// @brief Adds node to the open list (Assumes it is not in the closed list
  /// already)
  /// @param node Node to add to the open lists
  virtual void addNodeToOpenList(const Node &node) {
    m_openList.emplace(addFCost(node, m_heuristicFunction), node.m_index);
  }

  /// @brief Looks in the closed list for a node
  /// @param nodeIndex Index of the node which I wish to find in the closed list
  /// @param anchor Unused here, used in derived class
  /// @return True if the node is in the closed list, false otherwise
  virtual bool checkClosedList(const NodeIndex &nodeIndex,
                               const bool anchor = true) {
    if (m_closedList.find(nodeIndex) == m_closedList.end()) {
      return false;
    }
    return true;
  }

  /// @brief The function will find and return a the node in the graph
  /// correspinding to an index
  /// @param index the index of the node wanted
  /// @return The node associated index, or boost::none if the node is not
  /// in the graph
  boost::optional<Node &> getNode(const NodeIndex &index);

  /// @brief Get the path from the start state to the end index
  /// @param endIndex is the index of the node at the end of the path
  /// @return the path from the start state to the end of the path, returning
  /// boost::none if there is no path
  boost::optional<Path> getPath(const NodeIndex &endIndex);

  /// @brief Set a heuristic lambda function variable to the indicated lambda
  /// function
  /// @param heuristicVector vector of heuristic functions to use
  void setHeuristicLambdaFunctions(Heuristic *heuristicLambda,
                                   const std::string &functionName,
                                   const bool debug);

  /// @brief Set a edge cost lambda function variable to the indicated lambda
  /// function
  /// @param edgeCostFunction indication of which edge cost function to use
  void setEdgeCostLambdaFunction(const std::string &edgeCostFunction);

  /// @brief Function to safely calculate F-cost, avoiding INT rollover
  /// @param node the predicted F-cost of the node
  /// @return Calculated F-cost
  inline Cost addFCost(const Node &node, const Heuristic &heuristic) const {
    const Cost fCost{node.m_gCost + heuristic(node.m_state) * m_epsilon};
    if (fCost < 0) {
      ROS_ERROR("ERROR: fCost was negative throwing");
      throw "";
    }
    return fCost;
  }

  /// @brief Determine the gear of the vehicle based on its movement along its
  /// x-axis
  /// @param directionOfMovement The direction of the vehicles movement along
  /// its x-axis
  /// @return The gear indicated by the direction of movement
  Gear getGear(const double directionOfMovement);

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
  inline NodeIndex signedSzudzikPair(const int x, const int y) const {
    // Modification to Sudzik Pairing algorithm for negative numbers

    // @NOTE I'm leaving a and b as ints, because it is unlikely to reach an
    // overflow size naturally, and this allows for easy error checking
    const int a = (x >= 0 ? 2 * x : (-2 * x) - 1);
    if (a < 0) {
      ROS_ERROR("ERROR: a is negative. a: %s | x: %s",
                std::to_string(a).c_str(), std::to_string(x).c_str());
    }
    const int b = (y >= 0 ? 2 * y : (-2 * y) - 1);
    if (b < 0) {
      ROS_ERROR("ERROR: b is negative. b: %s | y: %s",
                std::to_string(b).c_str(), std::to_string(y).c_str());
    }
    // Szudzik Pairing Algorithm
    return szudzikPair(a, b);
    // * 0.5 <- Removed to ensure numbers are ints
  }

  /// @brief This function computes a unique index for a state
  /// The state is rounded based on the resolution
  /// @param state which is being hashed
  /// @return the index for the graph
  inline NodeIndex hashFunction(const State &state) const {
    // Make theta positive so we can use szudzik and get a tighter packing
    const double thetaPositive{state.m_theta + M_PI};
    // wrapToPi makes theta -Pi to Pi so we want to make it 0 to 2PI
    if (thetaPositive < 0) {
      ROS_ERROR("Theta is negative");
      throw "";
    }
    const NodeIndex theta{
        static_cast<size_t>(thetaPositive / m_angularResolution)};

    const int x{static_cast<int>(state.m_x / m_distanceResolution)};
    const int y{static_cast<int>(state.m_y / m_distanceResolution)};

    const NodeIndex index1{signedSzudzikPair(x, y)};
    const NodeIndex index2{szudzikPair(index1, theta)};
    if (index2 < index1 or index2 < theta) {
      ROS_ERROR(
          "ERROR: Likely overflow. index2 is less than one of its components");
      ROS_ERROR("index2: %s | index1: %s | theta: %s",
                std::to_string(index2).c_str(), std::to_string(index1).c_str(),
                std::to_string(theta).c_str());
      throw "";
    }
    const NodeIndex index3{szudzikPair(index2, state.m_gear)};
    if (index3 < index2 or index3 < state.m_gear) {
      ROS_ERROR(
          "ERROR: Likely overflow. index3 is less than one of its components");
      ROS_ERROR("index2: %s | index2: %s | index3: %s",
                std::to_string(index3).c_str(), std::to_string(index2).c_str(),
                std::to_string(state.m_gear).c_str());
      throw "";
    }
    return index3;
  };

private:
  /// @brief Flag to indicate whether to use certain print statements
  const bool m_debug;

  const double m_angularCostScaling;
};
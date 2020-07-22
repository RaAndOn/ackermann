#pragma once

#include <ackermann_planner/a_star.hpp>

class MHAStar : public AStar {
public:
  /// @brief Implementation of A* planner for an ackermann vehicle with X,Y
  /// position, steering angle of Theta.
  /// @param primitives vector of the primitives which are used for node
  /// expansion
  /// @param distanceResolutionMeters linear resolution used for discretizing
  /// and indexing states
  /// @param angularResolutionDegrees angular resolution used for discretizing
  /// and indexing states
  /// @param epsilon Amount to weight the heuristic
  /// @param weight Amount to weight the admissable FCost
  /// @param admissableFunction String indicating which admissible heuristic to
  /// use
  /// @param inadmissableFunctions String indicating which inadmissable
  /// heuristics to use
  /// @param edgeCostFunction Name of the edge cost function, which will be
  /// mapped to a lambda variable
  /// @param debug Flag to indicate whether to use certain print statements
  MHAStar(const std::vector<Primitive> &primitives,
          const double distanceResolutionMeters,
          const double angularResolutionDegrees, const double epsilon,
          const double weight, const std::string &admissableFunction,
          const std::string &inadmissableFunctions,
          const std::string &edgeCostFunction, const bool debug = true);

  ~MHAStar();

  /// @brief This public function performs a search from the start state to the
  /// goal state. The search it performs is based on the parameters provided at
  /// class initialization
  /// @param startState State from which the search is starting
  /// @param goalState State to which the search is planning
  /// @return Path from the start to goal state
  boost::optional<Path> search(const State &startState, const State &goalState);

private:
  /// @brief Vector of open lists for tracking which state is to be expanded
  /// next, one list per inadmissable heuristic
  std::vector<OpenList> m_inadmissableOpenVector;

  /// @brief Shared closed list for indadmissable heuristics indicating the node
  /// has been expanded by the inadmissable heuristics
  ClosedList m_inadmissableClosedList;

  /// @brief Vector of lambda functions representing inadmissable heuristics for
  /// MHA*
  std::vector<Heuristic> m_inadmissableHeuristics;

  /// @brief Amount to weight the admissable FCost
  const double m_weight;

  /// @brief Flag to indicate whether to use certain print statements
  const bool m_debug;

  /// @brief Multi-Heuristic A* search (Primary function of the class)
  /// @param goalNode Node to which Multi-Heuristic A* is trying to find a path
  bool mhaStar(const Node &goalNode);

  /// @brief Adds node to each open list, if it is not already in the respective
  /// closed list
  /// @param node Node to add to the open lists
  void addNodeToOpenList(const Node &node);

  /// @brief Checks to see if a node is in a particular closed list
  /// @param nodeIndex Index of the node which I wish to find in the closed list
  /// @param anchor Flag to indicate whether to look in the anchor closed list
  /// or the inadmissable closed list
  /// @return True if the node is in the closed list, false otherwise
  bool checkClosedList(const NodeIndex &nodeIndex, const bool anchor);

  /// @brief Add all of the strings in the vector to the vector of inadmissable
  /// heuristic lambda function variable
  /// @param heuristicVector vector of heuristic functions to use
  void setInadmissableHeuristics(std::vector<std::string> heuristicVector);
};
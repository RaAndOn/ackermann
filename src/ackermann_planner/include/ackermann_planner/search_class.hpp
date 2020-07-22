#pragma once

#include <boost/optional.hpp>
#include <unordered_map>
#include <vector> // std::vector

#include <ackermann_project/planner_utils.hpp>

using Cost = double;
using NodeIndex = size_t;

struct Node {
  size_t m_expansionOrder;
  State m_state;
  boost::optional<NodeIndex> m_parentIndex;
  Cost m_gCost;
  NodeIndex m_index;
  Node(const size_t expansionOrder, const State &state, const NodeIndex index,
       const boost::optional<NodeIndex> parentIndex, const Cost gCost)
      : m_expansionOrder{expansionOrder}, m_state{state}, m_index{index},
        m_parentIndex{parentIndex}, m_gCost{gCost} {}
};

using Path = std::vector<State>;
using Graph = std::unordered_map<NodeIndex, Node>;

class SearchClass {
public:
  /// @brief This public function performs a search from the start state to the
  /// goal state. The search it performs is based on the parameters provided at
  /// class initialization
  /// @param startState State from which the search is starting
  /// @param goalState State to which the search is planning
  /// @return Path from the start to goal state
  virtual boost::optional<Path> search(const State &startState,
                                       const State &goalState) {
    ROS_WARN("%s", m_warning.c_str());
    return boost::none;
  };

  /// @brief Returns the graph created in the search
  /// @return The node graph
  virtual Graph getGraph() const {
    ROS_WARN("%s", m_warning.c_str());
    return Graph{};
  }

  /// @brief Returns the size of the graph created in the search
  /// @return The size of the graph created in the search
  virtual size_t getGraphSize() const {
    ROS_WARN("%s", m_warning.c_str());
    return 0;
  }

  /// @brief Returns the time in seconds taken to perform latest search
  /// @return The time in seconds taken to perform latest search
  virtual double getLatestSearchTime() const {
    ROS_WARN("%s", m_warning.c_str());
    return 0;
  }

private:
  std::string m_warning = "Failed to select specific derived SearchClass";
};
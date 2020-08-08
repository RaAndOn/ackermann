#include <algorithm> // max and reverese
#include <cmath>
#include <iterator>
#include <sstream>

#include <ackermann_planner/mha_star.hpp>

MHAStar::MHAStar(const std::vector<Primitive> &primitives,
                 const double distanceResolutionMeters,
                 const double angularResolutionDegrees,
                 const double turningRadius, const double epsilon,
                 const double weight, const std::string &admissableFunction,
                 const std::string &inadmissableFunctions,
                 const std::string &edgeCostFunction, const bool debug)
    : AStar{primitives,
            distanceResolutionMeters,
            angularResolutionDegrees,
            turningRadius,
            epsilon,
            admissableFunction,
            edgeCostFunction,
            false},
      m_weight{weight}, m_debug{debug} {
  ROS_INFO_COND(m_debug, "Initializing MHAStar search class");

  // Split heuristic function string into vector
  std::vector<std::string> inadmissableVector;
  std::stringstream inadmissableStream(inadmissableFunctions);
  while (inadmissableStream.good()) {
    std::string heuristic;
    // get first string delimited by comma
    getline(inadmissableStream, heuristic, ',');
    inadmissableVector.push_back(heuristic);
  }

  // Set heuristic and edge cost functions
  ROS_INFO_COND(m_debug, "Admissable Heuristic:");
  setHeuristicLambdaFunctions(&m_heuristicFunction, admissableFunction,
                              m_debug);
  ROS_INFO_COND(m_debug, "Inadmissable Heuristic:");
  setInadmissableHeuristics(inadmissableVector);
  setEdgeCostLambdaFunction(edgeCostFunction);
}

MHAStar::~MHAStar() {
  // Clean up inadmissable heuristic pointers
  for (auto &heuristic : m_inadmissableHeuristics) {
    delete &heuristic;
  }
}

boost::optional<Path> MHAStar::search(const State &startState,
                                      const State &goalState) {
  // Mark the start time of the search
  const double begin = ros::Time::now().toSec();
  // Clear Data Structures
  m_nodeGraph.clear();
  m_openList = OpenList();
  m_closedList = ClosedList();
  m_inadmissableOpenVector =
      std::vector<OpenList>{m_inadmissableHeuristics.size()};
  m_inadmissableClosedList = ClosedList();
  // Set Variables
  m_goalState = goalState;
  // Add start node to the graph and open list
  m_startIndex = hashFunction(startState);
  const auto startNodeIt =
      m_nodeGraph.emplace(m_startIndex, Node{m_nodeGraph.size(), startState,
                                             m_startIndex, boost::none, 0});
  addNodeToOpenList(startNodeIt.first->second);

  // Add goal node to the graph
  auto goalIndex = hashFunction(m_goalState.get());
  const auto goalNodeIt = m_nodeGraph.emplace(
      goalIndex, Node{m_nodeGraph.size(), m_goalState.get(), goalIndex,
                      boost::none, std::numeric_limits<Cost>::max()});

  // Perform search
  ROS_INFO_COND(false, "Performing Multi-Heuristic A* Search");
  const bool foundPath = mhaStar(goalNodeIt.first->second);

  // Record how long it took to perform search
  const double end = ros::Time::now().toSec();
  m_latestSearchTime = end - begin;

  // Return the path
  if (foundPath) {
    return getPath(goalIndex);
  }
  return boost::none;
}

void MHAStar::addNodeToOpenList(const Node &node) {
  if (m_closedList.find(node.m_index) == m_closedList.end()) {
    m_openList.emplace(addFCost(node, m_heuristicFunction), node.m_index);
  }

  if (m_inadmissableClosedList.find(node.m_index) ==
      m_inadmissableClosedList.end()) {
    for (size_t i = 0; i < m_inadmissableHeuristics.size(); ++i) {
      const auto fCost{addFCost(node, m_inadmissableHeuristics[i])};
      m_inadmissableOpenVector[i].emplace(fCost, node.m_index);
    }
  }
}

bool MHAStar::checkClosedList(const NodeIndex &nodeIndex, const bool anchor) {
  if (anchor) {
    if (m_closedList.find(nodeIndex) == m_closedList.end()) {
      return false;
    }
  } else {
    if (m_inadmissableClosedList.find(nodeIndex) ==
        m_inadmissableClosedList.end()) {
      return false;
    }
  }
  return true;
}

void MHAStar::setInadmissableHeuristics(
    std::vector<std::string> heuristicVector) {
  // Iterate over the vector of heuristic functions
  while (not heuristicVector.empty()) {
    auto functionName{heuristicVector.front()};
    // Delete first heuristic from vector
    heuristicVector.erase(heuristicVector.begin());
    auto heuristic = new Heuristic{};
    setHeuristicLambdaFunctions(heuristic, functionName, m_debug);
    // At the function to the vector of inadmissable heuristic functions
    m_inadmissableHeuristics.push_back(*heuristic);
  }
}

bool MHAStar::mhaStar(const Node &goalNode) {
  // Keep expanding nodes while the open list is not empty
  while (not m_openList.empty()) {
    // Alternate between inadmissable heuristics
    for (size_t i = 0; i < m_inadmissableHeuristics.size(); ++i) {
      // Get the node off the top of the priority queue
      const auto minKeyInadmissable{m_inadmissableOpenVector[i].top().first};
      const auto minKeyAnchor{m_openList.top().first};
      if (minKeyInadmissable <= m_weight * minKeyAnchor) {
        // These conditions are equivalent to the goal node having been expanded
        if (goalNode.m_gCost <= minKeyInadmissable) {
          if (goalNode.m_gCost < std::numeric_limits<Cost>::max()) {
            ROS_INFO("PATH FOUND");
            // Return indicating a path wasfound
            return true;
          }
        } else {
          // Get the top node off of the current inadmissable open vector
          const auto currentNode{
              getNode(m_inadmissableOpenVector[i].top().second)};
          m_inadmissableOpenVector[i].pop();
          // Expand the node
          getSuccessors(*currentNode, m_inadmissableHeuristics[i]);
          // Add the node to the inadmissable closed list
          m_inadmissableClosedList.emplace(currentNode->m_index);
        }
      } else {
        // These conditions are equivalent to the goal node having been expanded
        if (goalNode.m_gCost <= minKeyAnchor) {
          if (goalNode.m_gCost < std::numeric_limits<Cost>::max()) {
            ROS_INFO("PATH FOUND");
            // Return indicating a path wasfound
            return true;
          }
        } else {
          // Get the top node off of the current admissable open vector
          const auto currentNode{getNode(m_openList.top().second)};
          m_openList.pop();
          // Expand the node
          getSuccessors(*currentNode, m_heuristicFunction);
          // Add the node to the admissable closed list
          m_closedList.emplace(currentNode->m_index);
        }
      }
    }
  }
  ROS_INFO("PATH NOT FOUND: Entire Configuration Space Expanded");
  // Return false, indicating no path found
  return false;
}

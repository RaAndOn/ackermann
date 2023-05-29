#include <algorithm> // max and reverese
#include <cmath>
#include <iterator>

#include <ackermann_planner/a_star.hpp>
#include <utility>

AStar::AStar(const std::vector<Primitive> &primitives,
             const double distanceResolutionMeters,
             const double angularResolutionDegrees, const double epsilon,
             const std::string &heuristicFunction,
             const std::string &edgeCostFunction)
    : m_primitives{std::move(primitives)}, m_distanceResolution{distanceResolutionMeters},
      m_angularResolution{M_PI * angularResolutionDegrees / 180},
      m_collisionThresh{1} {
  ROS_INFO("Set AStar as search algorithm");
  // Set Variables
  m_epsilon = epsilon;
  // Set functions
  setHeuristicFunction(heuristicFunction);
  setEdgeCostFunction(edgeCostFunction);
}

AStar::~AStar() = default;

boost::optional<Path> AStar::search(const State &startState,
                                    const State &goalState) {
  // Mark the start time of the search
  const double begin = ros::Time::now().toSec();
  // Clear Data Structures
  m_nodeGraph.clear();
  m_openList = OpenList();
  // Set Variables
  m_goalState = goalState;
  // Add start node to the graph and open list
  m_startIndex = hashFunction(startState);
  const auto startNodeIt =
      m_nodeGraph.emplace(m_startIndex, Node{m_nodeGraph.size(), startState,
                                             m_startIndex, boost::none, 0});
  m_openList.emplace(addFCost(startNodeIt.first->second), m_startIndex);

  auto goalIndex = hashFunction(m_goalState.get());
  const auto goalNodeIt = m_nodeGraph.emplace(
      goalIndex, Node{m_nodeGraph.size(), m_goalState.get(), goalIndex,
                      boost::none, std::numeric_limits<Cost>::max()});
  // Keep expanding nodes while the open list is not empty
  while (not m_openList.empty()) {
    const auto currentNode{getNode(m_openList.top().second)};
    const auto minKey{m_openList.top().first};
    m_openList.pop();
    if (m_epsilon <= 1) {
      currentNode->m_closed = true;
    }
    if (goalNodeIt.first->second.m_gCost <= minKey and
        goalNodeIt.first->second.m_gCost < std::numeric_limits<Cost>::max()) {
      ROS_INFO("PATH FOUND");
      // Record how long it took to perform search
      const double end = ros::Time::now().toSec();
      m_latestSearchTime = end - begin;
      // Return the path found
      return getPath(currentNode->m_index);
    }
    getSuccessors(*currentNode);
  }
  ROS_INFO("PATH NOT FOUND: Entire Configuration Space Expanded");
  // Record how long it took to perform search
  const double end = ros::Time::now().toSec();
  m_latestSearchTime = end - begin;
  // Return the none, indicating no path found
  return boost::none;
}

void AStar::getSuccessors(const Node &currentNode) {
  // Iterate over all the motion primitives to the node's successors
  State currentState{currentNode.m_state};
  for (const auto &primitive : m_primitives) {
    if (primitive.m_deltaX > 0.0 and currentState.m_gear == Gear::REVERSE) {
      continue;
    }
    if (primitive.m_deltaX < 0.0 and currentState.m_gear == Gear::FORWARD) {
      continue;
    }
    // Create new successor state from primitive
    const auto newTheta =
        wrapToPi(currentNode.m_state.m_theta + primitive.m_deltaTheta);
    const auto newX = currentNode.m_state.m_x +
                      primitive.m_deltaX * std::cos(newTheta) -
                      primitive.m_deltaY * std::sin(newTheta);
    const auto newY = currentNode.m_state.m_y +
                      primitive.m_deltaX * std::sin(newTheta) +
                      primitive.m_deltaY * std::cos(newTheta);
    const auto newGear = getGear(primitive.m_deltaX);
    const State newState{newX, newY, newTheta, newGear};
    const NodeIndex newIndex{hashFunction(newState)};
    /// TODO: Add occupancy grid with cell cost
    const auto cellCost = 0;
    if (cellCost >= 0 and cellCost < m_collisionThresh) // if free
    {
      const auto gCost{m_edgeCostFunction(primitive) + currentNode.m_gCost};
      // Check if node already exists in the graph
      const auto existingNode{getNode(newIndex)};
      if (existingNode) {
        // Update the node if is not closed and the cost to come is better then
        // add it back to the open list
        if (not existingNode->m_closed and gCost < existingNode->m_gCost) {
          existingNode->m_gCost = gCost;
          existingNode->m_parentIndex = currentNode.m_index;
          m_openList.emplace(addFCost(*existingNode),
                             newIndex); // Note: we are not removing or
                                        // modifying the original entry of the
                                        // node in the openlist, simply adding
                                        // an additional entry of the same
                                        // node with the new parent
        }
        continue;
      }
      // Add node to the open list if the node does not exist in the graph
      const auto newNodeIt = m_nodeGraph.emplace(
          newIndex, Node{m_nodeGraph.size(), newState, newIndex,
                         currentNode.m_index, gCost});
      m_openList.emplace(addFCost(newNodeIt.first->second), newIndex);
    }
  }
}

boost::optional<Node &> AStar::getNode(const NodeIndex index) {
  const auto nodeIt{m_nodeGraph.find(index)};
  if (nodeIt == m_nodeGraph.end()) {
    return boost::none;
  }
  return nodeIt->second;
}

boost::optional<Path> AStar::getPath(const NodeIndex &endIndex) {
  auto currentNode{getNode(endIndex)};
  if (not currentNode) {
    ROS_INFO("Requested goal node is not in the existing node graph. "
             "Returning None");
    return boost::none;
  }
  Path path{currentNode->m_state};
  auto previousNodeIndex{currentNode->m_parentIndex};
  auto previousNode{getNode(previousNodeIndex.get())};
  while (true) {
    path.emplace_back(previousNode->m_state);
    previousNodeIndex = previousNode->m_parentIndex;
    if (not previousNodeIndex) {
      break;
    }
    previousNode = getNode(previousNodeIndex.get());
  }
  std::reverse(std::begin(path), std::end(path));
  return path;
}

void AStar::setEdgeCostFunction(const std::string &edgeCostFunction) {
  if (edgeCostFunction == "Simple") {
    m_edgeCostFunction = [this](const Primitive &primitive) { return 1; };
    return;
  }
  if (edgeCostFunction == "Euclidean") {
    m_edgeCostFunction = [this](const Primitive &primitive) {
      return std::sqrt(std::pow(primitive.m_deltaX, 2.0) +
                       std::pow(primitive.m_deltaY, 2.0));
    };
    return;
  }
  ROS_ERROR("ERROR: Valid Edge Cost Function Not Provided");
  throw "";
}

void AStar::setHeuristicFunction(const std::string &heuristicFunction) {
  if (heuristicFunction == "Euclidean") {
    m_heuristicFunction = [this](const State &state) {
      return std::sqrt(std::pow(state.m_x - m_goalState->m_x, 2.0) +
                       std::pow(state.m_y - m_goalState->m_y, 2.0));
    };
    return;
  }
  if (heuristicFunction == "None") {
    m_heuristicFunction = [this](const State &state) { return 0; };
    return;
  }
  ROS_ERROR("ERROR: Valid Heuristic Function Not Provided");
  throw "";
}

Gear AStar::getGear(const double directionOfMovement) {
  if (std::abs(directionOfMovement) < __DBL_EPSILON__) {
    return Gear::STOP;
  } else if (directionOfMovement > 0) {
    return Gear::FORWARD;
  } else if (directionOfMovement < 0) {
    return Gear::REVERSE;
  } else {
    ROS_ERROR("ERROR: Could not determine vehicle gear");
    throw "";
  }
}

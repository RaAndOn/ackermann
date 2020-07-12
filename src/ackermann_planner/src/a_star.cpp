#include <algorithm> // max and reverese
#include <cmath>
#include <iterator>

#include <ackermann_planner/a_star.hpp>

AStar::AStar(const std::vector<Primitive> &primitives,
             const double distanceThresholdMeters,
             const double angularThresholdDegrees,
             const double distanceResolutionMeters,
             const double angularResolutionDegrees)
    : m_primitives{primitives}, m_distanceThreshold{distanceThresholdMeters},
      m_angularThreshold{180 * angularThresholdDegrees / M_PI},
      m_distanceResolution{distanceResolutionMeters},
      m_angularResolution{180 * angularResolutionDegrees / M_PI} {
  ROS_INFO("Instantiate AStar");
}

AStar::~AStar() = default;

boost::optional<Path> AStar::astar(const State &startState,
                                   const State &goalState, const int epsilon,
                                   const std::string &heuristicFunction,
                                   const std::string &edgeCostFunction) {
  ROS_INFO("Clear Data Structures");
  m_nodeGraph.clear();
  m_openList = OpenList();
  ROS_INFO("Set Variables");
  m_epsilon = epsilon;
  m_goalState = goalState;
  // Set functions
  ROS_INFO("Set functions");
  setHeuristicFunction(heuristicFunction);
  setEdgeCostFunction(edgeCostFunction);
  // Add start node to the graph and open list
  ROS_INFO("Initialize List");
  m_startIndex = hashFunction(startState);
  ROS_INFO("Start Index: %d", m_startIndex);
  const auto startNodeIt = m_nodeGraph.emplace(
      m_startIndex, Node{startState, m_startIndex, boost::none, 0});
  ROS_INFO("First node inserted into graph");
  m_openList.emplace(addFCost(startNodeIt.first->second), m_startIndex);
  ROS_INFO("Begin Expansion");
  while (not m_openList.empty()) {
    ROS_INFO("Get next node in open list");
    const auto currentNode{getNode(m_openList.top().second)};
    m_openList.pop();
    if (m_epsilon <= 1) {
      currentNode->m_closed = true;
    }
    if (checkIfSameState(currentNode->m_state, m_goalState.get())) {
      ROS_INFO("PATH FOUND");
      getSuccessors(*currentNode);
      return getPath(currentNode->m_index);
    }
    ROS_INFO("Getting Successors");
    getSuccessors(*currentNode);
    ROS_INFO("Successors Gotten");
  }
  ROS_INFO("Entire Configuration Space Expanded");
  return boost::none;
}

void AStar::getSuccessors(const Node &currentNode) {
  for (const auto &primitive : m_primitives) {
    const auto newX = currentNode.m_state.m_x + primitive.m_deltaX;
    const auto newY = currentNode.m_state.m_y + primitive.m_deltaY;
    const auto newTheta = ackermann::wrapToPi(currentNode.m_state.m_theta +
                                              primitive.m_deltaTheta);
    const State newState{newX, newY, newTheta, Gear::FORWARD};
    const auto newIndex{hashFunction(newState)};
    /// TODO: Add occupancy grid with cell cost
    const auto cellCost = 0;
    if (cellCost >= 0 and cellCost < m_collisionThresh) // if free
    {
      const auto gCost{m_edgeCostFunction(primitive) + currentNode.m_gCost};
      const auto existingNode{getNode(newIndex)};
      if (existingNode) {
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
      // Add node to the open list
      const auto newNodeIt = m_nodeGraph.emplace(
          newIndex, Node{newState, newIndex, currentNode.m_index, gCost});
      m_openList.emplace(addFCost(newNodeIt.first->second), newIndex);
    }
  }
}

boost::optional<Node &> AStar::getNode(const NodeIndex index) {
  if (index < 0) {
    std::cout << "\nERROR: Node index given to AStar::getNode is negative"
              << std::endl;
    throw "";
  }
  const auto nodeIt{m_nodeGraph.find(index)};
  if (nodeIt == m_nodeGraph.end()) {
    return boost::none;
  }
  return nodeIt->second;
}

boost::optional<Path> AStar::getPath(const NodeIndex &endIndex) {
  auto currentNode{getNode(endIndex)};
  if (not currentNode) {
    std::cout << "\nRequested goal node is not in the existing node graph. "
                 "Returning None"
              << std::endl;
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

bool AStar::checkIfSameState(const State &state1, const State &state2) {
  const double thetaDiff{
      std::abs(ackermann::wrapToPi(state1.m_theta - state2.m_theta))};
  if (thetaDiff > m_angularThreshold) {
    return false;
  }
  const double euclidean{std::pow(state1.m_x - state2.m_x, 2.0) +
                         std::pow(state1.m_y - state2.m_y, 2.0)};
  return euclidean < std::pow(m_distanceThreshold, 2.0);
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
  std::cout << "\nERROR: Valid Edge Cost Function Not Provided" << std::endl;
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
  std::cout << "\nERROR: Valid Heuristic Function Not Provided" << std::endl;
  throw "";
}
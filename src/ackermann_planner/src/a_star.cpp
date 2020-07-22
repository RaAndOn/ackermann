#include <algorithm> // max and reverese
#include <cmath>
#include <iterator>
#include <sstream>

#include <ackermann_planner/a_star.hpp>

AStar::AStar(const std::vector<Primitive> &primitives,
             const double distanceResolutionMeters,
             const double angularResolutionDegrees, const double epsilon,
             const std::string &heuristicFunctions,
             const std::string &edgeCostFunction)
    : m_primitives{primitives}, m_distanceResolution{distanceResolutionMeters},
      m_angularResolution{M_PI * angularResolutionDegrees / 180},
      m_collisionThresh{1}, m_epsilon{epsilon} {
  ROS_INFO("Initialize search Class");

  // Split heuristic function string into vector
  std::vector<std::string> heuristicVector;
  std::stringstream heuristicStream(heuristicFunctions);
  while (heuristicStream.good()) {
    std::string heuristic;
    // get first string delimited by comma
    getline(heuristicStream, heuristic, ',');
    heuristicVector.push_back(heuristic);
  }

  // Set heuristic and edge cost functions
  setHeuristicLambdaFunctions(heuristicVector);
  setEdgeCostLambdaFunction(edgeCostFunction);
}

AStar::~AStar() = default;

boost::optional<Path> AStar::search(const State &startState,
                                    const State &goalState) {
  // Mark the start time of the search
  const double begin = ros::Time::now().toSec();
  // Clear Data Structures
  m_nodeGraph.clear();
  m_anchorOpenList = OpenList();
  m_inadmissableOpenVector =
      std::vector<OpenList>{m_inadmissableHeuristics.size()};
  // Set Variables
  m_goalState = goalState;
  // Add start node to the graph and open list
  m_startIndex = hashFunction(startState);

  m_startIndex = hashFunction(startState);
  const auto startNodeIt =
      m_nodeGraph.emplace(m_startIndex, Node{m_nodeGraph.size(), startState,
                                             m_startIndex, boost::none, 0});

  addNodeToOpenLists(startNodeIt.first->second);

  auto goalIndex = hashFunction(m_goalState.get());
  const auto goalNodeIt = m_nodeGraph.emplace(
      goalIndex, Node{m_nodeGraph.size(), m_goalState.get(), goalIndex,
                      boost::none, std::numeric_limits<Cost>::max()});

  bool foundPath{false};
  // Perform search
  if (m_inadmissableHeuristics.empty()) {
    ROS_INFO("AStar");
    foundPath = aStar(goalNodeIt.first->second);
  } else {
    ROS_INFO("MHAStar");
    foundPath = mhaStar(goalNodeIt.first->second);
  }
  // Record how long it took to perform search
  const double end = ros::Time::now().toSec();
  m_latestSearchTime = end - begin;

  if (foundPath) {
    return getPath(goalIndex);
  } else {
    return boost::none;
  }
}

bool AStar::aStar(const Node &goalNode) {
  // Keep expanding nodes while the open list is not empty
  while (not m_anchorOpenList.empty()) {
    // Get the node off the top of the priority queue
    const auto currentNode{getNode(m_anchorOpenList.top().second)};
    const auto minKey{m_anchorOpenList.top().first};
    m_anchorOpenList.pop();
    // This is equivalent to the condition of goal node having been expanded
    if (goalNode.m_gCost <= minKey and
        goalNode.m_gCost < std::numeric_limits<Cost>::max()) {
      ROS_INFO("PATH FOUND");
      // Return true, indicating path found
      return true;
    }
    // Expand the current node
    getSuccessors(*currentNode, m_anchorHeuristic);
    // Add the current node to the closed list
    if (m_epsilon <= 1) {
      currentNode->m_anchorClosed = true;
    }
  }
  ROS_INFO("PATH NOT FOUND: Entire Configuration Space Expanded");
  // Return false, indicating no path found
  return false;
}

void AStar::addNodeToOpenLists(const Node &node) {
  if (not node.m_anchorClosed) {
    m_anchorOpenList.emplace(addFCost(node, m_anchorHeuristic), node.m_index);
  }

  if (not node.m_inadmissableClosed) {
    for (size_t i = 0; i < m_inadmissableHeuristics.size(); ++i) {
      const auto fCost{addFCost(node, m_inadmissableHeuristics[i])};
      m_inadmissableOpenVector[i].emplace(fCost, node.m_index);
    }
  }
}

bool AStar::mhaStar(const Node &goalNode) {
  // Keep expanding nodes while the open list is not empty
  while (not m_anchorOpenList.empty()) {
    // Alternate between inadmissable heuristics
    for (size_t i = 0; i < m_inadmissableHeuristics.size(); ++i) {
      // Get the node off the top of the priority queue
      const auto minKeyInadmissable{m_inadmissableOpenVector[i].top().first};
      const auto minKeyAnchor{m_anchorOpenList.top().first};
      if (minKeyInadmissable <= minKeyAnchor) {
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
          getSuccessors(*currentNode, m_inadmissableHeuristics[i], false);
          // Add the node to the inadmissable closed list
          currentNode->m_inadmissableClosed = true;
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
          const auto currentNode{getNode(m_anchorOpenList.top().second)};
          m_anchorOpenList.pop();
          // Expand the node
          getSuccessors(*currentNode, m_anchorHeuristic, true);
          // Add the node to the admissable closed list
          currentNode->m_anchorClosed = true;
        }
      }
    }
  }
  ROS_INFO("PATH NOT FOUND: Entire Configuration Space Expanded");
  // Return false, indicating no path found
  return false;
}

void AStar::getSuccessors(const Node &currentNode, const Heuristic &heuristic,
                          const bool anchor) {
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
        if ((anchor and not existingNode->m_anchorClosed) or
            (not anchor and not existingNode->m_inadmissableClosed)) {
          if (gCost < existingNode->m_gCost) {
            existingNode->m_gCost = gCost;
            existingNode->m_parentIndex = currentNode.m_index;
            addNodeToOpenLists(
                *existingNode); // Note: we are not removing or
                                // modifying the original entry of the
                                // node in the openlist, simply adding
                                // an additional entry of the same
                                // node with the new parent
          }
        }
        continue;
      }
      // Add node to the open list if the node does not exist in the graph
      const auto newNodeIt = m_nodeGraph.emplace(
          newIndex, Node{m_nodeGraph.size(), newState, newIndex,
                         currentNode.m_index, gCost});
      addNodeToOpenLists(newNodeIt.first->second);
    }
  }
}

boost::optional<Node &> AStar::getNode(const NodeIndex &index) {
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

void AStar::setEdgeCostLambdaFunction(const std::string &edgeCostFunction) {
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

void AStar::setHeuristicLambdaFunctions(
    std::vector<std::string> heuristicVector) {
  // The first function is set as the anchor heuristic and all other functions
  // are added as inadmissable heuristics.

  // Initialize variables
  auto heuristic{&m_anchorHeuristic};
  bool anchor{true};
  while (not heuristicVector.empty()) {
    // Get first heuristic in vector
    auto heuristicFunction{heuristicVector.front()};
    // Delete first heuristic from vector
    heuristicVector.erase(heuristicVector.begin());
    if (not anchor) {
      heuristic = new Heuristic{};
    }
    if (heuristicFunction == "Euclidean") {
      *heuristic = [this](const State &state) {
        return std::sqrt(std::pow(state.m_x - m_goalState->m_x, 2.0) +
                         std::pow(state.m_y - m_goalState->m_y, 2.0));
      };
    } else if (heuristicFunction == "Angular") {
      *heuristic = [this](const State &state) {
        return 180 / M_PI *
               std::abs(wrapToPi(state.m_theta - m_goalState->m_theta));
      };
    } else if (heuristicFunction == "None") {
      *heuristic = [this](const State &state) { return 0; };
    } else {
      ROS_ERROR("ERROR: Valid Heuristic Function Not Provided");
      throw "";
    }
    // Report the heuristics to the command line
    if (anchor) {
      ROS_INFO("Admissable Heuristic set as %s", heuristicFunction.c_str());
    } else {
      m_inadmissableHeuristics.push_back(*heuristic);
      ROS_INFO("Inadmissable Heuristic set as %s", heuristicFunction.c_str());
    }
    // Ensure all heuristics after the first are inadmissable
    anchor = false;
  }
}

Gear AStar::getGear(const double directionOfMovement) {
  if (std::abs(directionOfMovement) < std::numeric_limits<double>::epsilon()) {
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
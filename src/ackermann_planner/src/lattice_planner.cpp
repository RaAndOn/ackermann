#include <ackermann_planner/lattice_planner.hpp>
#include <ackermann_project/ackermann_utils.hpp>

// Point x -- east, y -- north and z -- up yaw
LatticePlanner::LatticePlanner(ros::NodeHandle &privateNH,
                               ros::NodeHandle &publicNH)
    : m_privateNH{privateNH}, m_publicNH{publicNH} {
  // Get parameters from Param server
  m_privateNH.param("wheelbase", m_wheelbase, 3.0);
  m_privateNH.param("dt", m_dt, .1);
  m_privateNH.param("discretization_degrees", m_discretizationDegrees, 22.5);
  m_privateNH.param("steering_increments", m_steeringIncrements, 1);
  m_privateNH.param("velocity", m_velocity, 5.0);
  // Set publishers and subscribers
  m_pubVisualization = m_publicNH.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker", 0);

  m_markerID = 0;

  MotionPrimitive motionPrimitive{m_wheelbase, m_velocity, m_dt,
                                  m_discretizationDegrees,
                                  m_steeringIncrements};
  m_motionPrimitivesVector = motionPrimitive.getMotionPrimitives();

  initializeMarkers();
}

LatticePlanner::~LatticePlanner() = default;

void LatticePlanner::initializeMarkers() {
  m_forwardMarker.header.frame_id = "ground_truth";
  m_forwardMarker.header.stamp = ros::Time();
  // marker.ns = "my_namespace";
  m_forwardMarker.id = 0;
  m_forwardMarker.type = visualization_msgs::Marker::ARROW;
  m_forwardMarker.action = visualization_msgs::Marker::ADD;
  m_forwardMarker.pose.position.x = 1;
  m_forwardMarker.pose.position.y = 1;
  m_forwardMarker.pose.position.z = 1;
  m_forwardMarker.pose.orientation.x = 0.0;
  m_forwardMarker.pose.orientation.y = 0.0;
  m_forwardMarker.pose.orientation.z = 0.0;
  m_forwardMarker.pose.orientation.w = 1.0;
  m_forwardMarker.scale.x = .4;
  m_forwardMarker.scale.y = 0.075;
  m_forwardMarker.scale.z = 0.075;
  m_forwardMarker.color.a = 1.0; // Don't forget to set the alpha!
  m_forwardMarker.color.r = 0.0;
  m_forwardMarker.color.g = 1.0;
  m_forwardMarker.color.b = 0.0;

  m_reverseMarker = m_forwardMarker;
  m_reverseMarker.scale.x = .3;
  m_reverseMarker.scale.y = 0.15;
  m_reverseMarker.scale.z = 0.15;
}

void LatticePlanner::visualizationLoopTEST() {
  visualization_msgs::MarkerArray markerArray;
  // State state{m_markerID, 0, 0, 0, Gear::FORWARD};
  // addMarkerToArray(markerArray, state);
  // ++m_markerID;

  // for (int i = 1; i < 150; ++i) {
  //   state.m_id = i;
  //   state.m_x +=
  //       m_motionPrimitivesVector[1].m_deltaX * std::cos(state.m_theta) -
  //       m_motionPrimitivesVector[1].m_deltaY * std::sin(state.m_theta);
  //   state.m_y +=
  //       m_motionPrimitivesVector[1].m_deltaX * std::sin(state.m_theta) +
  //       m_motionPrimitivesVector[1].m_deltaY * std::cos(state.m_theta);
  //   state.m_theta = ackermann::wrapToPi(
  //       state.m_theta + m_motionPrimitivesVector[1].m_deltaTheta);
  //   // ROS_INFO("Steer Angle %s", std::to_string(state.m_theta).c_str());
  //   addMarkerToArray(markerArray, state);
  // }

  for (const auto primitive : m_motionPrimitivesVector) {
    Gear gear = (primitive.m_deltaX > 0) ? Gear::FORWARD : Gear::REVERSE;
    State state{m_markerID, primitive.m_deltaX, primitive.m_deltaY,
                primitive.m_deltaTheta, gear};
    addMarkerToArray(markerArray, state);
    ++m_markerID;
  }
  m_pubVisualization.publish(markerArray);
}

void LatticePlanner::addMarkerToArray(
    visualization_msgs::MarkerArray &markerArray, const State &state) {
  // Initialize marker shape based on the gear
  visualization_msgs::Marker marker;
  if (state.m_gear == Gear::FORWARD) {
    marker = m_forwardMarker;
  } else if (state.m_gear == Gear::REVERSE) {
    marker = m_reverseMarker;
  } else {
    ROS_ERROR("Member variable 'm_gear' of state was not set properly");
  }
  // Set Marker specifics
  marker.id = state.m_id;
  marker.pose.position.x = state.m_x;
  marker.pose.position.y = state.m_y;
  marker.pose.orientation.z = state.m_theta;

  // Add marker to marker array
  markerArray.markers.push_back(marker);
}

void LatticePlanner::reset() {}

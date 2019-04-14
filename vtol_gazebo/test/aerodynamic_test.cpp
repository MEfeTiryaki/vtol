/*
 File name: aerodynamic_test.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 23.03.2019
 Date last modified: 23.03.2019
 */

#include <iostream>

#include "vtol_gazebo/effort_module/AerodynamicForce.hpp"
#include "ros_node_utils/ros_node_utils.hpp"

using namespace ros_node_utils;
using namespace gazebo;
using namespace effort;
int main(int argc, char **argv)
{
  std::string nodeName = "aerodynics_test_node";
  ros::init(argc, argv, nodeName);
  ros::NodeHandle* nodeHandle_ = new ros::NodeHandle("~");

  CONFIRM("Aerodynamics test code started.");
  AerodynamicForce* aerodynamicsModule = new AerodynamicForce(nodeHandle_);

  aerodynamicsModule->readParameters();
  aerodynamicsModule->initialize();
  aerodynamicsModule->initializeSubscribers();
  aerodynamicsModule->initializePublishers();
  aerodynamicsModule->initializeServices();

  // ROBOT POSITION AND ORIENTATION IS SET
  Eigen::Vector3d positionWorldToBase;
  Eigen::Quaterniond orientationWorldToBase;  // Q = [w,x,y,z] order
  Eigen::Vector3d linearVelocityOfBaseInBaseFrame;
  Eigen::Vector3d angularVelocityOfBaseInBaseFrame;
  paramRead(nodeHandle_, "/aerodynamics_test/position", positionWorldToBase);
  paramRead(nodeHandle_, "/aerodynamics_test/orientation", orientationWorldToBase);
  paramRead(nodeHandle_, "/aerodynamics_test/linear_velocity", linearVelocityOfBaseInBaseFrame);
  paramRead(nodeHandle_, "/aerodynamics_test/angular_velocity", angularVelocityOfBaseInBaseFrame);
  aerodynamicsModule->setPosition(positionWorldToBase);
  aerodynamicsModule->setOrientation(orientationWorldToBase);
  aerodynamicsModule->setLinearVelocity(linearVelocityOfBaseInBaseFrame);
  aerodynamicsModule->setAngularVelocity(angularVelocityOfBaseInBaseFrame);

  // CALCULATE AEROYNAMICS
  aerodynamicsModule->calculateAerodynamics();
  Eigen::Vector3d origin = aerodynamicsModule->getOrigin();
  Eigen::Vector3d forceInWorldFrame = aerodynamicsModule->getForce();
  Eigen::Vector3d torqueInWorldFrame = aerodynamicsModule->getTorque();

  return 0;
}
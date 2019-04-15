/*
 File name: aerodynamic_test.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 23.03.2019
 Date last modified: 23.03.2019
 */

#include <iostream>

#include "vtol_gazebo/wrench_module/AerodynamicForce.hpp"
#include "ros_node_utils/ros_node_utils.hpp"

using namespace ros_node_utils;
using namespace gazebo;
using namespace wrench;
int main(int argc, char **argv)
{
  std::string nodeName = "aerodynics_test_node";
  ros::init(argc, argv, nodeName);
  ros::NodeHandle* nodeHandle_ = new ros::NodeHandle("~");

  wrench::WrenchLink* wrenchLink_;

  CONFIRM("Aerodynamics test code started.");
  AerodynamicForce* aerodynamicsModule = new AerodynamicForce(nodeHandle_,wrenchLink_);

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
  wrenchLink_->setPositionWorldtoBase(positionWorldToBase);
  wrenchLink_->setOrientationWorldtoBase(orientationWorldToBase);
  wrenchLink_->setLinearVelocityOfBaseInBaseFrame(linearVelocityOfBaseInBaseFrame);
  wrenchLink_->setAngularVelocityOfBaseInBaseFrame(angularVelocityOfBaseInBaseFrame);

  // CALCULATE AEROYNAMICS
  aerodynamicsModule->advance();
  Eigen::Vector3d origin = aerodynamicsModule->getOrigin();
  Eigen::Vector3d forceInWorldFrame = aerodynamicsModule->getForceInWorldFrame();
  Eigen::Vector3d torqueInWorldFrame = aerodynamicsModule->getTorqueInWorldFrame();

  return 0;
}

/*
 File name: DoubleRotorGazeboPlugin.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 30.10.2018
 Date last modified: 17.03.2019
 */

#include "vtol_gazebo/rotor/DoubleRotorGazeboPlugin.hpp"

using namespace ros_node_utils;

namespace gazebo {

DoubleRotorGazeboPlugin::DoubleRotorGazeboPlugin()
    : GazeboModelPluginBase(),
      topCommand_(),
      bottomCommand_(),
      debug_(false),
      thrust_(0),
      torque_(0),
      thrustCoefficient_(0),
      dragCoefficient_(0),
      reductionCoefficient_(0),
      simulationSpeedRatio_(0.01)
{
}

void DoubleRotorGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  GazeboModelPluginBase::Load(model, sdf);


  // connect to world updates from Gazebo
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DoubleRotorGazeboPlugin::OnUpdate, this));
}


void DoubleRotorGazeboPlugin::create()
{
  GazeboModelPluginBase::create();
  CONFIRM("[DoubleRotorGazeboPlugin::" + rotorName_ + ": is created");
}

void DoubleRotorGazeboPlugin::readParameters(sdf::ElementPtr sdf)
{
  GazeboModelPluginBase::readParameters(sdf);
  debug_ = sdf->GetElement("debug")->Get<bool>();

  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  robotName_ = sdf->GetElement("robot_name")->Get<std::string>();
  if (debug_) {
    std::cout << "Robot Name : " << robotName_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  baseLinkName_ = sdf->GetElement("base_link_name")->Get<std::string>();
  if (debug_) {
    std::cout << "base link name : " << baseLinkName_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  topLinkName_ = sdf->GetElement("top_link_name")->Get<std::string>();
  if (debug_) {
    std::cout << "top link name : " << topLinkName_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  bottomLinkName_ = sdf->GetElement("bottom_link_name")->Get<std::string>();
  if (debug_) {
    std::cout << "bottom link name : " << bottomLinkName_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  rotorName_ = sdf->GetElement("rotor_name")->Get<std::string>();
  if (debug_) {
    std::cout << "rotor name : " << rotorName_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  topJointName_ = sdf->GetElement("top_joint_name")->Get<std::string>();
  if (debug_) {
    std::cout << "top joint name : " << topJointName_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  bottomJointName_ = sdf->GetElement("bottom_joint_name")->Get<std::string>();
  if (debug_) {
    std::cout << "bottom joint name : " << bottomJointName_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  simulationSpeedRatio_ = sdf->GetElement("simulation_speed_ratio")->Get<double>();
  if (debug_) {
    std::cout << "Simulation Speed Ratio : " << simulationSpeedRatio_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  thrustCoefficient_ = sdf->GetElement("thrust_coefficient")->Get<double>();
  if (debug_) {
    std::cout << "Thrust Coefficient : " << thrustCoefficient_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  dragCoefficient_ = sdf->GetElement("drag_coefficient")->Get<double>();
  if (debug_) {
    std::cout << "Drag Coefficient : " << dragCoefficient_ << std::endl;
  }
  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  reductionCoefficient_ = sdf->GetElement("second_blade_reduction")->Get<double>();
  if (debug_) {
    std::cout << "Second Blade Reduction : " << reductionCoefficient_ << std::endl;
  }

  CONFIRM("[DoubleRotorGazeboPlugin::" + rotorName_ + ": read parameters");
}

void DoubleRotorGazeboPlugin::initialize()
{
  GazeboModelPluginBase::initialize();
  CONFIRM("[DoubleRotorGazeboPlugin::" + rotorName_ + ": is initialized");
}

void DoubleRotorGazeboPlugin::initializeJointStructure()
{
  topJoint_ = model_->GetJoint(topJointName_);
  if (topJoint_ == NULL) {
    std::cout << " Couldn't find the joint : " << topJointName_ << std::endl;
  } else {
    topJoint_->SetVelocity(0, 0);
  }

  bottomJoint_ = model_->GetJoint(bottomJointName_);
  if (bottomJoint_ == NULL) {
    std::cout << " Couldn't find the joint : " << bottomJointName_ << std::endl;
  } else {
    topJoint_->SetVelocity(0, 0);
  }

  CONFIRM("[DoubleRotorGazeboPlugin::" + rotorName_ + ": initialized Joints");
}

void DoubleRotorGazeboPlugin::initializeLinkStructure()
{
  topLink_ = model_->GetLink(topLinkName_);
  if (topLink_ == NULL) {
    std::cout << " Couldn't find the link : " << topLinkName_ << std::endl;
  }
  bottomLink_ = model_->GetLink(bottomLinkName_);
  if (bottomLink_ == NULL) {
    std::cout << " Couldn't find the link : " << bottomLinkName_ << std::endl;
  }

  baseLink_ = model_->GetLink(baseLinkName_);
  if (baseLink_ == NULL) {
    std::cout << " Couldn't find the link : " << baseLinkName_ << std::endl;
  }

  parentLink_ = topLink_->GetParentJointsLinks().at(0);
  std::string parentLinkName_ = parentLink_->GetName();

  poseTopLinkToParent_ = parentLink_->GetInitialRelativePose().GetInverse()
      * topLink_->GetInitialRelativePose();
  poseBottomLinkToParent_ = parentLink_->GetInitialRelativePose().GetInverse()
      * bottomLink_->GetInitialRelativePose();
  CONFIRM("[DoubleRotorGazeboPlugin::" + rotorName_ + ": initialized Links");
}

void DoubleRotorGazeboPlugin::initializeSubscribers()
{
  topCommandSubscriber_ = this->nodeHandle_->subscribe(
      "/" + robotName_ + "/" + rotorName_ + "/top/command", 10,
      &DoubleRotorGazeboPlugin::TopCommandsCallback, this);
  bottomCommandSubscriber_ = this->nodeHandle_->subscribe(
      "/" + robotName_ + "/" + rotorName_ + "/bottom/command", 10,
      &DoubleRotorGazeboPlugin::BottomCommandsCallback, this);
  topCommand_.data = 0;
  bottomCommand_.data = 0;

  CONFIRM("[DoubleRotorGazeboPlugin::" + rotorName_ + ": initialized Subscribers");
}

void DoubleRotorGazeboPlugin::initializePublishers()
{
  forceVisualizationPublisher_ = this->nodeHandle_->advertise<visualization_msgs::Marker>(
      "/" + robotName_ + "/" + rotorName_ + "/thrust_marker", 0);
  torqueVisualizationPublisher_ = this->nodeHandle_->advertise<visualization_msgs::Marker>(
      "/" + robotName_ + "/" + rotorName_ + "/drag_marker", 0);

  CONFIRM("[DoubleRotorGazeboPlugin::" + rotorName_ + ": initialized Publishers");
}

void DoubleRotorGazeboPlugin::writeSimulation()
{

  double topVelocity = simulationSpeedRatio_ * topCommand_.data;
  double bottomVelocity = simulationSpeedRatio_ * bottomCommand_.data;
  topJoint_->SetVelocity(0, topVelocity);
  bottomJoint_->SetVelocity(0, bottomVelocity);

  // APPLY TRUST ON ROTOR
  thrust_ = thrustCoefficient_ * reductionCoefficient_ * topCommand_.data * topCommand_.data;
  thrust_ += thrustCoefficient_ * reductionCoefficient_ * bottomCommand_.data * bottomCommand_.data;
  topLink_->AddRelativeForce(math::Vector3(0, 0, thrust_));

  // APPLY DRAG
  torque_ = thrustCoefficient_ * reductionCoefficient_ * dragCoefficient_ * topCommand_.data
      * topCommand_.data;
  torque_ += thrustCoefficient_ * reductionCoefficient_ * dragCoefficient_ * bottomCommand_.data
      * bottomCommand_.data;

  math::Vector3 dragTorqueInParentFrame = poseTopLinkToParent_.rot.RotateVector(
      math::Vector3(0, 0, torque_));
}

void DoubleRotorGazeboPlugin::readSimulation()
{
  double topYaw = topJoint_->GetAngle(0).Radian();
  double bottomYaw = bottomJoint_->GetAngle(0).Radian();

  math::Pose orientationRotorToTopBladeMath = math::Pose(0, 0, 0, 0, 0, topYaw)
      * poseTopLinkToParent_;
  math::Pose orientationRotorToBottomBladeMath = math::Pose(0, 0, 0, 0, 0, bottomYaw)
      * poseBottomLinkToParent_;
  T_rotor_top_blade_.setOrigin(
      tf::Vector3(orientationRotorToTopBladeMath.pos.x, orientationRotorToTopBladeMath.pos.y,
                  orientationRotorToTopBladeMath.pos.z));
  T_rotor_top_blade_.setRotation(
      tf::Quaternion(orientationRotorToTopBladeMath.rot.x, orientationRotorToTopBladeMath.rot.y,
                     orientationRotorToTopBladeMath.rot.z, orientationRotorToTopBladeMath.rot.w));

  T_rotor_bottom_blade_.setOrigin(
      tf::Vector3(orientationRotorToBottomBladeMath.pos.x, orientationRotorToBottomBladeMath.pos.y,
                  orientationRotorToBottomBladeMath.pos.z));
  T_rotor_bottom_blade_.setRotation(
      tf::Quaternion(orientationRotorToBottomBladeMath.rot.x,
                     orientationRotorToBottomBladeMath.rot.y,
                     orientationRotorToBottomBladeMath.rot.z,
                     orientationRotorToBottomBladeMath.rot.w));
}

void DoubleRotorGazeboPlugin::publishTf()
{
  static tf::TransformBroadcaster br;
  br.sendTransform(
      tf::StampedTransform(T_rotor_top_blade_, ros::Time::now(),
                           topLink_->GetParentJointsLinks().at(0)->GetName(), topLinkName_));
  br.sendTransform(
      tf::StampedTransform(T_rotor_bottom_blade_, ros::Time::now(),
                           topLink_->GetParentJointsLinks().at(0)->GetName(), bottomLinkName_));

}

void DoubleRotorGazeboPlugin::publish()
{
  // TODO Mehmet Efe Tiryaki 14.11.2018 : Correct the virtualization using correct orienation of moment and force

  visualization_msgs::Marker marker;

 if (thrust_ != 0) {
    marker.header.stamp = ros::Time();
    marker.header.frame_id = topLinkName_;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = -sin(0.5);
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = sin(0.5);
    marker.scale.x = 1.0 * thrust_;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    forceVisualizationPublisher_.publish(marker);
  }
  if (torque_ != 0) {
    // TODO : Rotate this according to body orientation
    marker.header.stamp = ros::Time();
    marker.header.frame_id = baseLinkName_;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = -sin(0.5);
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = sin(0.5);
    marker.scale.x = 1.0 * torque_;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    torqueVisualizationPublisher_.publish(marker);
  }
}

void DoubleRotorGazeboPlugin::TopCommandsCallback(const std_msgs::Float64& msg)
{
  topCommand_ = msg;
}

void DoubleRotorGazeboPlugin::BottomCommandsCallback(const std_msgs::Float64& msg)
{
  bottomCommand_ = msg;
}

GZ_REGISTER_MODEL_PLUGIN(DoubleRotorGazeboPlugin)
}

/*
 File name: VtolBaseGazeboPlugin.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 28.10.2018
 Date last modified: 17.03.2019
 */

#include "vtol_gazebo/VtolBaseGazeboPlugin.hpp"

using namespace ros_node_utils;

namespace gazebo {

VtolBaseGazeboPlugin::VtolBaseGazeboPlugin()
    : GazeboModelPluginBase(),
      debug_(false),
      forceOnBodyInWorldFrame_(Eigen::Vector3d::Zero()),
      torqueOnBodyInWorldFrame_(Eigen::Vector3d::Zero()),
      aerodynamics_()
{
}

void VtolBaseGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  GazeboModelPluginBase::Load(model, sdf);
  // connect to world updates from Gazebo
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VtolBaseGazeboPlugin::OnUpdate, this));
}

void VtolBaseGazeboPlugin::create()
{
  GazeboModelPluginBase::create();
  aerodynamics_ = new effort::AerodynamicForce(this->nodeHandle_);

  CONFIRM("[VtolBaseGazeboPlugin] : is created");
}

void VtolBaseGazeboPlugin::readParameters(sdf::ElementPtr sdf)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  GazeboModelPluginBase::readParameters(sdf);

  debug_ = sdf->GetElement("debug")->Get<bool>();

  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  robotName_ = sdf->GetElement("robot_name")->Get<std::string>();
  if (debug_) {
    std::cout << "Robot Name : " << robotName_ << std::endl;
  }

  // TODO EFE TIRYAKI 17.03.2019: make this rosparam
  linkName_ = sdf->GetElement("link_name")->Get<std::string>();
  if (debug_) {
    std::cout << "link name : " << linkName_ << std::endl;
  }

  // READ PARAMETERS

  // TODO EFE TIRYAKI 17.03.2019: get robotname through rosparam
  // in the aerodynamics
  aerodynamics_->setName(robotName_);
  aerodynamics_->readParameters();

  CONFIRM("[VtolBaseGazeboPlugin] : read parameters");
}

// INITILIZATION

void VtolBaseGazeboPlugin::initialize()
{
  GazeboModelPluginBase::initialize();
  aerodynamics_->initialize();
  CONFIRM("[VtolBaseGazeboPlugin] : is initialized");
}

void VtolBaseGazeboPlugin::initializeLinkStructure()
{
  GazeboModelPluginBase::initializeLinkStructure();
  std::lock_guard<std::mutex> lock(this->mutex_);
  link_ = model_->GetLink(linkName_);
  if (link_ == NULL) {
    std::cout << " Couldn't find the link : " << linkName_ << std::endl;
  }
  CONFIRM("[VtolBaseGazeboPlugin] : initialized Links");
}

void VtolBaseGazeboPlugin::initializeSubscribers()
{
  GazeboModelPluginBase::initializeSubscribers();
  aerodynamics_->initializeSubscribers();
  CONFIRM("[VtolBaseGazeboPlugin] : initialized Subscribers");
}

void VtolBaseGazeboPlugin::initializePublishers()
{
  GazeboModelPluginBase::initializePublishers();
  aerodynamics_->initializePublishers();
  CONFIRM("[VtolBaseGazeboPlugin] : initialized Publishers");
}

// SIMULATION METHODS
void VtolBaseGazeboPlugin::writeSimulation()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  // Calculate Force and Moments
  aerodynamics_->setPosition(positionWorldToBase_);
  aerodynamics_->setOrientation(orientationWorldToBase_);
  aerodynamics_->setLinearVelocity(linearVelocityOfBaseInBaseFrame_);
  aerodynamics_->setAngularVelocity(angularVelocityOfBaseInBaseFrame_);
  Eigen::Vector3d origin = aerodynamics_->getOrigin();
  Eigen::Vector3d forceInWorldFrame = aerodynamics_->getForce();
  Eigen::Vector3d torqueInWorldFrame = aerodynamics_->getTorque();

  link_->AddForceAtRelativePosition(
      math::Vector3(forceInWorldFrame[0], forceInWorldFrame[1], forceInWorldFrame[2]),
      math::Vector3(origin[0], origin[1], origin[2]));
  link_->AddRelativeTorque(
      math::Vector3(torqueInWorldFrame[0], torqueInWorldFrame[1], torqueInWorldFrame[2]));

}

void VtolBaseGazeboPlugin::readSimulation()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  positionWorldToBase_ = Eigen::Vector3d(link_->GetWorldPose().pos.x, link_->GetWorldPose().pos.y,
                                         link_->GetWorldPose().pos.z);
  orientationWorldToBase_ = Eigen::Quaterniond(link_->GetWorldPose().rot.w,
                                               link_->GetWorldPose().rot.x,
                                               link_->GetWorldPose().rot.y,
                                               link_->GetWorldPose().rot.z);

  angularVelocityOfBaseInBaseFrame_ << link_->GetWorldAngularVel().x, link_->GetWorldAngularVel().y, link_
      ->GetWorldAngularVel().z;
  linearVelocityOfBaseInBaseFrame_ << link_->GetWorldLinearVel().x, link_->GetWorldLinearVel().y, link_
      ->GetWorldLinearVel().z;

}

// PUBLISHING METHODS
void VtolBaseGazeboPlugin::publishTf()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  static tf::TransformBroadcaster br;
  T_WB_.setOrigin(
      tf::Vector3(positionWorldToBase_[0], positionWorldToBase_[1], positionWorldToBase_[2]));
  tf::Quaternion q = tf::Quaternion(orientationWorldToBase_.x(), orientationWorldToBase_.y(),
                                    orientationWorldToBase_.z(), orientationWorldToBase_.w());
  T_WB_.setRotation(q);
  br.sendTransform(tf::StampedTransform(T_WB_, ros::Time::now(), "odom", linkName_));

}

void VtolBaseGazeboPlugin::publish()
{
  // TODO Mehmet Efe Tiryaki 14.11.2018 : Publish force and moment markers for
  // RViz

}

// AERODYNAMICS

//

GZ_REGISTER_MODEL_PLUGIN(VtolBaseGazeboPlugin)
}

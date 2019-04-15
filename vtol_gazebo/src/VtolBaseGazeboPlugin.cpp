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

void VtolBaseGazeboPlugin::OnUpdate()
{
  VtolBaseGazeboPlugin::OnUpdate();
  publishMarker();
}

void VtolBaseGazeboPlugin::create()
{
  GazeboModelPluginBase::create();
  wrenchLink_ = new wrench::WrenchLink();

  aerodynamics_ = new wrench::AerodynamicForce(this->nodeHandle_,wrenchLink_);

  visualizer_ = new wrench::WrenchVisualizer(this->nodeHandle_);
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

  visualizer_->addWrench(aerodynamics_, baseLink_->GetName(), Eigen::Vector4d(1, 0, 0, 1),
                         Eigen::Vector4d(1, 0, 0, 1));

  CONFIRM("[VtolBaseGazeboPlugin] : is initialized");
}

void VtolBaseGazeboPlugin::initializeLinkStructure()
{
  GazeboModelPluginBase::initializeLinkStructure();
  std::lock_guard<std::mutex> lock(this->mutex_);
  baseLink_ = model_->GetLink(linkName_);
  if (baseLink_ == NULL) {
    std::cout << " Couldn't find the link : " << linkName_ << std::endl;
  }
  CONFIRM("[VtolBaseGazeboPlugin] : initialized Links");
}

void VtolBaseGazeboPlugin::initializeSubscribers()
{
  GazeboModelPluginBase::initializeSubscribers();
  aerodynamics_->initializeSubscribers();

  markerPublisher_ = nodeHandle_->advertise<visualization_msgs::MarkerArray>("visualization_marker",
                                                                              1);
  
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
  aerodynamics_->advance();
  Eigen::Vector3d origin = aerodynamics_->getOrigin();
  Eigen::Vector3d forceInWorldFrame = aerodynamics_->getForceInWorldFrame();
  Eigen::Vector3d torqueInWorldFrame = aerodynamics_->getTorqueInWorldFrame();

  baseLink_->AddForceAtRelativePosition(
      math::Vector3(forceInWorldFrame[0], forceInWorldFrame[1], forceInWorldFrame[2]),
      math::Vector3(origin[0], origin[1], origin[2]));
  baseLink_->AddRelativeTorque(
      math::Vector3(torqueInWorldFrame[0], torqueInWorldFrame[1], torqueInWorldFrame[2]));

}

void VtolBaseGazeboPlugin::readSimulation()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  auto basePose = baseLink_->GetWorldPose();
  math::Vector3 linearVel = baseLink_->GetRelativeLinearVel();
  math::Vector3 angularVel = baseLink_->GetRelativeAngularVel();
  auto gravity = baseLink_->GetWorld()->GetPhysicsEngine()->GetGravity();
  positionWorldToBase_ = Eigen::Vector3d(basePose.pos.x, basePose.pos.y, basePose.pos.z);
  orientationWorldToBase_ = Eigen::Quaterniond(basePose.rot.w, basePose.rot.x, basePose.rot.y,
                                               basePose.rot.z);
  linearVelocityOfBaseInBaseFrame_ = Eigen::Vector3d(linearVel.x, linearVel.y, linearVel.z);
  angularVelocityOfBaseInBaseFrame_ = Eigen::Vector3d(angularVel.x, angularVel.y, angularVel.z);


      // Update Wrench
   wrenchLink_->setPositionWorldtoBase(positionWorldToBase_);
   wrenchLink_->setOrientationWorldtoBase(orientationWorldToBase_);
   wrenchLink_->setLinearVelocityOfBaseInBaseFrame(linearVelocityOfBaseInBaseFrame_);
   wrenchLink_->setAngularVelocityOfBaseInBaseFrame(angularVelocityOfBaseInBaseFrame_);
   wrenchLink_->setMass(baseLink_->GetInertial()->GetMass());
   wrenchLink_->setGravity(Eigen::Vector3d(gravity[0], gravity[1], gravity[2]));

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

void VtolBaseGazeboPlugin::publishMarker()
{
  visualization_msgs::MarkerArray markerArray;

    visualizer_->calculateMarkers(markerArray);

    markerPublisher_.publish(markerArray);

}


// AERODYNAMICS

//

GZ_REGISTER_MODEL_PLUGIN(VtolBaseGazeboPlugin)
}

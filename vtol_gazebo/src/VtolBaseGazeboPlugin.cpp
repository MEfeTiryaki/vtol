/*
 Author : M. Efe Tiryaki
 */


#include "vtol_gazebo/VtolBaseGazeboPlugin.hpp"
namespace gazebo {

// Todo : check if we can add gopigo name here
VtolBaseGazeboPlugin::VtolBaseGazeboPlugin()
    : nodeHandle_()
{
}

VtolBaseGazeboPlugin::~VtolBaseGazeboPlugin()
{
}

void VtolBaseGazeboPlugin::Init()
{
}
void VtolBaseGazeboPlugin::Reset()
{
}

void VtolBaseGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  ns_ = ros::this_node::getNamespace();

  nodeHandle_ = new ros::NodeHandle("~");

  // Note : check if this is placed correctly
  this->readParameters(sdf);

  // Model
  this->model_ = model;

  // initialize joint structure
  initJointStructures();
  initLinkStructure();
  // initialize ROS pub/sub/services
  initSubscribers();
  initPublishers();

  // reset simulation variables
  Reset();

  // connect to world updates from Gazebo
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VtolBaseGazeboPlugin::OnUpdate, this));
}

void VtolBaseGazeboPlugin::readParameters(sdf::ElementPtr sdf)
{
  debug_ = sdf->GetElement("debug")->Get<bool>();

  robotName_  = sdf->GetElement("robot_name")->Get<std::string>();
  if(debug_){std::cout<<"Robot Name : " << robotName_ << std::endl;}

  linkName_  = sdf->GetElement("link_name")->Get<std::string>();
  if(debug_){std::cout<<"link name : " << linkName_ << std::endl;}

}



void VtolBaseGazeboPlugin::initJointStructures()
{
}

void VtolBaseGazeboPlugin::initLinkStructure()
{
  link_ = model_->GetLink(linkName_);
  if (link_ == NULL){
    std::cout<<" Couldn't find the link : " << linkName_ << std::endl;
  }
}

void VtolBaseGazeboPlugin::initSubscribers()
{
}

void VtolBaseGazeboPlugin::initPublishers()
{

}

void VtolBaseGazeboPlugin::OnUpdate()
{
  readSimulation();
  writeSimulation();
}


void VtolBaseGazeboPlugin::writeSimulation()
{
}
void VtolBaseGazeboPlugin::readSimulation()
{
  positionWorldToBase_ << link_->GetWorldPose().pos.x
                        , link_->GetWorldPose().pos.y
                        , link_->GetWorldPose().pos.z ;
  orientationWorldToBase_ << link_->GetWorldPose().rot.x
                           , link_->GetWorldPose().rot.y
                           , link_->GetWorldPose().rot.z
                           , link_->GetWorldPose().rot.w;

  linearVelocityWorldToBase_ << link_->GetWorldAngularVel().x
                             , link_->GetWorldAngularVel().y
                             , link_->GetWorldAngularVel().z;
  angularVelocityWorldToBase_ << link_->GetWorldLinearVel().x
                             , link_->GetWorldLinearVel().y
                             , link_->GetWorldLinearVel().z;

  //std::cout << "World Position   :"<< positionWorldToBase_<<std::endl;
  //std::cout << "World Orientaion :"<< orientationWorldToBase_<<std::endl;
  //std::cout << "World Angular Vel:"<< angularVelocityWorldToBase_<<std::endl;
  //std::cout << "World Linear Vel :"<< linearVelocityWorldToBase_<<std::endl;


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( positionWorldToBase_[0]
                                  , positionWorldToBase_[1]
                                  , positionWorldToBase_[2]));
  tf::Quaternion q = tf::Quaternion( orientationWorldToBase_[0]
                                   , orientationWorldToBase_[1]
                                   , orientationWorldToBase_[2]
                                   , orientationWorldToBase_[3]);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", linkName_));

}


GZ_REGISTER_MODEL_PLUGIN(VtolBaseGazeboPlugin)
}

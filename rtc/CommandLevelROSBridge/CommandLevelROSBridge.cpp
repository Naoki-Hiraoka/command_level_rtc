#include "CommandLevelROSBridge.h"
#include <tf2/utils.h>

#include <cnoid/BodyLoader>
#include <cnoid/EigenUtil>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

CommandLevelROSBridge::CommandLevelROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_qIn_("qIn", m_q_),
  m_basePoseIn_("basePoseIn", m_basePose_)
{
}

RTC::ReturnCode_t CommandLevelROSBridge::onInitialize(){
  addInPort("qIn", this->m_qIn_);
  addInPort("basePoseIn", this->m_basePoseIn_);

  cnoid::BodyLoader bodyLoader;

  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  this->robot_vrml_ = bodyLoader.load(fileName);
  if(!this->robot_vrml_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->robot_urdf_ = std::make_shared<urdf::Model>();
  this->robot_urdf_->initParam("robot_description");

  ros::NodeHandle pnh("~");
  jointStatePub_ = pnh.advertise<sensor_msgs::JointState>("output", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t CommandLevelROSBridge::onExecute(RTC::UniqueId ec_id){
  //ros::spinOnce();

  if(this->m_basePoseIn_.isNew()){
    robot_vrml_->rootLink()->p()[0] = m_basePose_.data.position.x;
    robot_vrml_->rootLink()->p()[1] = m_basePose_.data.position.y;
    robot_vrml_->rootLink()->p()[2] = m_basePose_.data.position.z;
    robot_vrml_->rootLink()->R() = cnoid::rotFromRpy(m_basePose_.data.orientation.r,
                                                     m_basePose_.data.orientation.p,
                                                     m_basePose_.data.orientation.y);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "/hrpsys/odom";
    transformStamped.child_frame_id = "/hrpsys/" + robot_urdf_->getRoot()->name;
    transformStamped.transform.translation.x = m_basePose_.data.position.x;
    transformStamped.transform.translation.y = m_basePose_.data.position.y;
    transformStamped.transform.translation.z = m_basePose_.data.position.z;
    tf2::Quaternion q;
    q.setRPY(m_basePose_.data.orientation.r,
             m_basePose_.data.orientation.p,
             m_basePose_.data.orientation.y);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
  }

  if(this->m_qIn_.isNew()){
    this->m_qIn_.read();
    if(this->m_q_.data.length() == robot_vrml_->numJoints()){
      sensor_msgs::JointState msg;
      msg.header.stamp = ros::Time::now();
      for(int i=0;i<robot_vrml_->numJoints();i++){
        robot_vrml_->joint(i)->q() = m_q_.data[i];

        msg.name.push_back(robot_vrml_->joint(i)->name());
        msg.position.push_back(m_q_.data[i]);
      }
      this->jointStatePub_.publish(msg);
    }

    robot_vrml_->calcForwardKinematics();
    robot_vrml_->calcCenterOfMass();

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "/hrpsys/odom";
    transformStamped.child_frame_id =  "/hrpsys/com";
    transformStamped.transform.translation.x = robot_vrml_->centerOfMass()[0];
    transformStamped.transform.translation.y = robot_vrml_->centerOfMass()[1];
    transformStamped.transform.translation.z = robot_vrml_->centerOfMass()[2];
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    br.sendTransform(transformStamped);
  }

  return RTC::RTC_OK;
}

static const char* CommandLevelROSBridge_spec[] = {
  "implementation_id", "CommandLevelROSBridge",
  "type_name",         "CommandLevelROSBridge",
  "description",       "CommandLevelROSBridge component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void CommandLevelROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(CommandLevelROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<CommandLevelROSBridge>, RTC::Delete<CommandLevelROSBridge>);
    }
};

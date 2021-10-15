#ifndef CommandLevelROSBridge_H
#define CommandLevelROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <urdf/model.h>
#include <cnoid/Body>

class CommandLevelROSBridge : public RTC::DataFlowComponentBase{
protected:
  std::shared_ptr<urdf::Model> robot_urdf_;
  cnoid::BodyPtr robot_vrml_;

  RTC::TimedDoubleSeq m_q_;
  RTC::InPort<RTC::TimedDoubleSeq> m_qIn_;
  RTC::TimedPose3D m_basePose_;
  RTC::InPort<RTC::TimedPose3D> m_basePoseIn_;
  ros::Publisher jointStatePub_;
  tf2_ros::TransformBroadcaster br;

public:
  CommandLevelROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

};


extern "C"
{
  void CommandLevelROSBridgeInit(RTC::Manager* manager);
};

#endif // CommandLevelROSBridge_H

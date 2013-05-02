/**
 * @file simpleWalk.h
 * @brief
 * @author A. Huaman 
 */

#ifndef __WALK_ANA__
#define __WALK_ANA__

#include <ros/ros.h>
#include "zmp/zmpUtilities.h"

/**
 * @class walk
 */
class walk {

 public:
  walk();
  ~walk();

  void setCommunications();
  void setInitPosition();
  void setJointStates( const sensor_msgs::JointState::ConstPtr &_js );
  void setZMPConfig();
  void setJointConfig();
  
  ///////////////////
  ros::NodeHandle* mRosNode;
  ros::Subscriber mSubJointStates;
  ros::Publisher mPubJointCommands;
  ros::Publisher mPubMode;
  ros::Publisher mPub_trajKeyframes;
  osrf_msgs::JointCommands mJointCommands;
  zmpUtilities mZmp;
  int mStatus;

  std::vector<int> mDofIndices;
  std::vector<double> mInitPose;
  double mTimeToSetInitPose;
  
  
};

#endif

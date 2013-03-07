/**
 * @file foreleg_jointControl.cpp
 */
#include "foreleg_jointControl.h"

namespace gazebo {

/**
 * @function
 * @brief
 */
foreleg_jointControl::foreleg_jointControl() {
}

/**
 * @function ~foreleg_jointControl
 * @brief Destructor
 */  
foreleg_jointControl::~foreleg_jointControl() {
  
  event::Events::DisconnectWorldUpdateBegin( this->mUpdateConnection ); 
}

/**
 * @function Load
 * @brief
 */
void foreleg_jointControl::Load( physics::ModelPtr _parent, 
				 sdf::ElementPtr _sdf ) {
  
  printf("Load from foreleg_jointControl \n");
  this->mModel = _parent;
  this->mWorld = this->mModel->GetWorld();
  
  
  SetInitialPose();

  // Init controlBundle - LEFT LEG
  int numActuatedJoints = 10;
  mCb.setSize(numActuatedJoints);
  std::vector<physics::JointPtr> joints(numActuatedJoints);
  joints[0] = mModel->GetJoint("LHP");
  joints[1] = mModel->GetJoint("LKP");
  joints[2] = mModel->GetJoint("LAP");
  joints[3] = mModel->GetJoint("LAR");

  joints[4] = mModel->GetJoint("RHP");
  joints[5] = mModel->GetJoint("RKP");
  joints[6] = mModel->GetJoint("RAP");
  joints[7] = mModel->GetJoint("RAR");

  joints[8] = mModel->GetJoint("LHY");
  joints[9] = mModel->GetJoint("RHY");

  mCb.setJoints(joints);
  
  std::vector<double> targets(numActuatedJoints);
  targets[0] = -90.0*3.1416/180.0;
  targets[1] = 150.0*3.1416/180.0;
  targets[2] = -60.0*3.1416/180.0;
  targets[3] = 0.0*3.1416/180.0;

  targets[4] = -90.0*3.1416/180.0;
  targets[5] = 150.0*3.1416/180.0;
  targets[6] = -60.0*3.1416/180.0;
  targets[7] = 0.0*3.1416/180.0;

  targets[8] = 0.0*3.1416/180.0;
  targets[9] = 0.0*3.1416/180.0;

  mCb.setTargets(targets);

  mCb.initPID( 0, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 1, 50, 0, 5, 0, 0, 50, -50 ); // LKP
  mCb.initPID( 2, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 3, 50, 0, 5, 0, 0, 50, -50 );

  mCb.initPID( 4, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 5, 50, 0, 5, 0, 0, 50, -50 ); // RKP
  mCb.initPID( 6, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 7, 50, 0, 5, 0, 0, 50, -50 );

  mCb.initPID( 8, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 9, 50, 0, 5, 0, 0, 50, -50 );

  mLastUpdateTime = mModel->GetWorld()->GetSimTime();
  
  // Set to update every world cycle. Listen to update event
  this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin( boost::bind(&foreleg_jointControl::UpdateStates, this));
  

  // Get World Pose
  math::Pose worldPose = mModel->GetWorldPose();
  std::cout << "Pose xyz: "<< worldPose.pos.x << "," << worldPose.pos.y << "," << worldPose.pos.z << std::endl;
  std::cout << "Rot q: " << worldPose.rot.w << ", " << worldPose.rot.x <<", "<< worldPose.rot.y << ", " << worldPose.rot.z << std::endl;


  // Get foot pose
  math::Pose LARPose = mModel->GetLink("Body_LAR")->GetWorldPose();  
  std::cout << "Pose LAR xyz: "<< LARPose.pos.x << "," << LARPose.pos.y << "," << LARPose.pos.z << std::endl;
  std::cout << "Rot q: " << LARPose.rot.w << ", " << LARPose.rot.x <<", "<< LARPose.rot.y << ", " << LARPose.rot.z << std::endl;

  // Set inverse
  math::Quaternion invFoot = (LARPose.rot).GetInverse();
  math::Quaternion newPoseWorld = invFoot*worldPose.rot;
  math::Pose flatPose;
  flatPose.Set( worldPose.pos, invFoot );
  mModel->SetWorldPose( flatPose );
}

/**
 * @function SetInitialPose
 * @brief
 */
void foreleg_jointControl::SetInitialPose() {   

  printf("Set initial pose \n");
  std::map<std::string, double> joint_position_map;
  
  
  // First simplest rule for flat surfaces: 
  // |LHP| + |LAP| = |LKP|
  joint_position_map["huboplus_foreleg::LHP"] = -90*3.1416/180.0;
  joint_position_map["huboplus_foreleg::LKP"] = 150*3.1416/180.0;
  joint_position_map["huboplus_foreleg::LAP"] = -60*3.1416/180.0;
  joint_position_map["huboplus_foreleg::LAR"] = 0*3.1416/180.0;
  
  joint_position_map["huboplus_foreleg::RHP"] = -90*3.1416/180.0;
  joint_position_map["huboplus_foreleg::RKP"] = 150*3.1416/180.0;
  joint_position_map["huboplus_foreleg::RAP"] = -60*3.1416/180.0;
  joint_position_map["huboplus_foreleg::RAR"] = 0*3.1416/180.0;

  this->mModel->SetJointPositions( joint_position_map );
  
  
}

/**
 * @function UpdateStates
 * @brief
 */
void foreleg_jointControl::UpdateStates() {
  
  // Get current time and dt
  common::Time current_time = mModel->GetWorld()->GetSimTime();
  double dt = current_time.Double() - mLastUpdateTime.Double();
  
  // Update controllers
  mCb.updateControls(dt);
    
  mLastUpdateTime = current_time;
  
}

/**
 * @function FixLink
 * @brief
 */
void foreleg_jointControl::FixLink( physics::LinkPtr _link ) {
  
  this->mJoint = this->mWorld->GetPhysicsEngine()->CreateJoint( "revolute", this->mModel );
  this->mJoint->SetModel( this->mModel );
  math::Pose pose = _link->GetWorldPose();
  this->mJoint->Load( physics::LinkPtr(), _link, pose );
  this->mJoint->SetAxis( 0, math::Vector3(0,0,0) );
  this->mJoint->SetHighStop(0,0);
  this->mJoint->SetLowStop(0,0);
  this->mJoint->SetAnchor(0, pose.pos );
  this->mJoint->Init();
}

/**
 * @function UnfixLink
 * @brief
 */
void foreleg_jointControl::UnfixLink() {
  this->mJoint.reset();
}


  GZ_REGISTER_MODEL_PLUGIN( foreleg_jointControl )
  } // end namespace gazebo

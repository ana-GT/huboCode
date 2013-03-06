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
  
  event::Events::DisconnectWorldUpdateStart( this->mUpdateConnection ); 
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

  // Init controlBundle
  int numActuatedJoints = 3;
  mCb.setSize(numActuatedJoints);
  std::vector<physics::JointPtr> joints(numActuatedJoints);

  joints[0] = mModel->GetJoint("LKP");
  joints[1] = mModel->GetJoint("LAP");
  joints[2] = mModel->GetJoint("LAR");

  mCb.setJoints(joints);
  
  std::vector<double> targets(numActuatedJoints);
  targets[0] = -10.0*3.1416/180.0;
  targets[1] = -10.0*3.1416/180.0;
  targets[2] = 0.0*3.1416/180.0; 
  mCb.setTargets(targets);

  mCb.initPID( 0, 100, 0, 10, 0, 0, 100, -100 );
  mCb.initPID( 1, 100, 0, 10, 0, 0, 100, -100 );
  mCb.initPID( 2, 100, 0, 10, 0, 0, 100, -100 );

  mLastUpdateTime = mModel->GetWorld()->GetSimTime();
  
  // Set to update every world cycle. Listen to update event
  this->mUpdateConnection = event::Events::ConnectWorldUpdateStart( boost::bind(&foreleg_jointControl::UpdateStates, this));
  

  // Get World Pose
  math::Pose worldPose = mModel->GetWorldPose();
  std::cout << "Pose xyz: "<< worldPose.pos.x << "," << worldPose.pos.y << "," << worldPose.pos.z << std::endl;
  std::cout << "Rot q: " << worldPose.rot.w << ", " << worldPose.rot.x <<", "<< worldPose.rot.y << ", " << worldPose.rot.z << std::endl;
  // Get LHP pose
  math::Pose LHPPose = mModel->GetLink("Body_LHP")->GetWorldPose();  
  std::cout << "Pose LHP xyz: "<< LHPPose.pos.x << "," << LHPPose.pos.y << "," << LHPPose.pos.z << std::endl;
  std::cout << "Rot q: " << LHPPose.rot.w << ", " << LHPPose.rot.x <<", "<< LHPPose.rot.y << ", " << LHPPose.rot.z << std::endl;

  // Get LKP pose
  math::Pose LKPPose = mModel->GetLink("Body_LKP")->GetWorldPose();  
  std::cout << "Pose LKP xyz: "<< LKPPose.pos.x << "," << LKPPose.pos.y << "," << LKPPose.pos.z << std::endl;
  std::cout << "Rot q: " << LKPPose.rot.w << ", " << LKPPose.rot.x <<", "<< LKPPose.rot.y << ", " << LKPPose.rot.z << std::endl;

  // Get LAP pose
  math::Pose LAPPose = mModel->GetLink("Body_LAP")->GetWorldPose();  
  std::cout << "Pose LAP xyz: "<< LAPPose.pos.x << "," << LAPPose.pos.y << "," << LAPPose.pos.z << std::endl;
  std::cout << "Rot q: " << LAPPose.rot.w << ", " << LAPPose.rot.x <<", "<< LAPPose.rot.y << ", " << LAPPose.rot.z << std::endl;

  // Get foot pose
  math::Pose LARPose = mModel->GetLink("Body_LAR")->GetWorldPose();  
  std::cout << "Pose LAR xyz: "<< LARPose.pos.x << "," << LARPose.pos.y << "," << LARPose.pos.z << std::endl;
  std::cout << "Rot q: " << LARPose.rot.w << ", " << LARPose.rot.x <<", "<< LARPose.rot.y << ", " << LARPose.rot.z << std::endl;
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
  joint_position_map["huboplus_foreleg::LKP"] = 0*3.1416/180.0;
  joint_position_map["huboplus_foreleg::LAP"] = -10*3.1416/180.0;
  joint_position_map["huboplus_foreleg::LAR"] = 0*3.1416/180.0;
  
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

/**
 * @file foreleg_jointControl.cpp
 */
#include "foreleg_jointControl.h"

namespace gazebo {

  /**
   * @function foreleg_jointControl
   * @brief Constructor
   */
  foreleg_jointControl::foreleg_jointControl() { 

    mNumActuatedJoints = 13;

    mActuatedJointNames.resize(mNumActuatedJoints);
    mActuatedJointNames[0] = "LHP"; mActuatedJointNames[1] = "LKP"; 
    mActuatedJointNames[2] = "LAP"; mActuatedJointNames[3] = "LAR";
 
    mActuatedJointNames[4] = "RHP"; mActuatedJointNames[5] = "RKP"; 
    mActuatedJointNames[6] = "RAP"; mActuatedJointNames[7] = "RAR"; 

    mActuatedJointNames[8] = "LHY"; mActuatedJointNames[9] = "RHY";
    mActuatedJointNames[10] = "LHR"; mActuatedJointNames[11] = "RHR";
    mActuatedJointNames[12] = "HPY"; 
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
    
    // Fill the joints
    mActuatedJoints.resize( mNumActuatedJoints );
    for( int i = 0; i < mNumActuatedJoints; ++i ) {
      mActuatedJoints[i] = mModel->GetJoint( mActuatedJointNames[i].c_str() );
    }

    // Set initial pose
    SetInitialPose();
    // Set robot to start with feet parallel to floor
    SetFootParallelToFloor();

    // Give the gaiter access to the model to control
    mG.init( mModel, mNumActuatedJoints, mActuatedJoints );

    // Save last update time
    mLastUpdateTime = mModel->GetWorld()->GetSimTime();
    
    // Set to update every world cycle. Listen to update event
    this->mUpdateConnection = event::Events::ConnectWorldUpdateBegin( boost::bind(&foreleg_jointControl::UpdateStates, this));
    
  }
  
  /**
   * @function SetInitialPose
   * @brief
   */
  void foreleg_jointControl::SetInitialPose() {   
    
    printf("Set initial pose \n");
    std::map<std::string, double> joint_position_map;
    // Should use numActuatedJoints...
    double initialJointVal[13] = {-90, 150, -60, 0, -90, 150, -60, 0, 0, 0, 0, 0, 0 };

    // First simplest rule for flat surfaces: |LHP| + |LAP| = |LKP|
    for( int i = 0; i < mNumActuatedJoints; ++i ) {
      char fullName[40];
      sprintf( fullName, "huboplus_foreleg::%s", mActuatedJointNames[i].c_str() );
      joint_position_map[fullName] = initialJointVal[i]*3.1416/180.0;
    }
    
    this->mModel->SetJointPositions( joint_position_map );    
  }

  /**
   * @function setFootParallelToFloor
   */
  void foreleg_jointControl::SetFootParallelToFloor() {
    
    // Get World Pose
    math::Pose worldPose = mModel->GetWorldPose();
       
    // Get foot pose (let's assume left and right feet both have the same pose)
    math::Pose LARPose = mModel->GetLink("Body_LAR")->GetWorldPose();  
    
    // Set inverse
    math::Quaternion invLeftFoot = (LARPose.rot).GetInverse();
    math::Quaternion newPoseWorld = invLeftFoot*worldPose.rot;
    math::Pose flatPose;
    flatPose.Set( worldPose.pos, invLeftFoot );
    mModel->SetWorldPose( flatPose );

  }

  
 
  /**
   * @function CalculateBoundaryValues
   */
  void foreleg_jointControl::CalculateBoundaryValues() {

    double d1 = 0.28; // Hip to Knee (0.280007)
    double d2 = 0.28; // Knee to Ankle (0.279942)
    double p = 0.07; // Length of foot (from ankle forward) (0.07-0.12)
    double alpha = -30*3.1416/180.0;
    double max_beta = asin( d2*cos(alpha)/d1 ); // right on top of ankle
    double min_beta = asin( ( d2*cos(alpha) - p ) / d1 ); // about to leave foot forward
    
    double LAP;
    double minLHP, maxLHP;
    double minLKP, maxLKP;

    LAP = ( 3.1416/2 + alpha )*180.0 / 3.1416;
    minLHP = min_beta*180.0 / 3.1416; 
    maxLHP = max_beta*180.0 / 3.1416;
    minLKP = ( minLHP + LAP ); 
    maxLKP = ( maxLHP + LAP );
    
    std::cout << "Right on top: LAP:"<< LAP <<" LHP: "<<maxLHP<<" LKP: "<<maxLKP<<std::endl;
    std::cout << "Right on front: LAP:"<< LAP <<" LHP: "<<minLHP<<" LKP: "<<minLKP<<std::endl;
  }

  
  /**
   * @function UpdateStates
   * @brief
 */
void foreleg_jointControl::UpdateStates() {


  // Get current time and dt
  common::Time current_time = mModel->GetWorld()->GetSimTime();
  double dt = current_time.Double() - mLastUpdateTime.Double();

  // Call gaiter
  mG.update( current_time.Double(), dt );

  // Update last time
  mLastUpdateTime = current_time;  
}

  GZ_REGISTER_MODEL_PLUGIN( foreleg_jointControl )
  } // end namespace gazebo

/**
 * @file lowerBody_jointControl.cpp
 */
#include "lowerBody_jointControl.h"

namespace gazebo {

  /**
   * @function
   * @brief
   */
  lowerBody_jointControl::lowerBody_jointControl() {
  }

  /**
   * @function ~lowerBody_jointControl
   * @brief Destructor
   */  
  lowerBody_jointControl::~lowerBody_jointControl() {

    event::Events::DisconnectWorldUpdateStart( this->mUpdateConnection ); 
  }
  
  /**
   * @function Load
   * @brief
   */
  void lowerBody_jointControl::Load( physics::ModelPtr _parent, 
				sdf::ElementPtr _sdf ) {

    printf("Load from lowerBody_jointControl \n");
    this->mModel = _parent;
    this->mWorld = this->mModel->GetWorld();


    SetInitialPose();

    // Set to update every world cycle. Listen to update event
    this->mUpdateConnection = event::Events::ConnectWorldUpdateStart( boost::bind(&lowerBody_jointControl::UpdateStates, this));
    
  }
  
  /**
   * @function SetInitialPose
   * @brief
   */
  void lowerBody_jointControl::SetInitialPose() {   

    printf("Set initial pose \n");
    std::map<std::string, double> joint_position_map;
    

    // First simplest rule for flat surfaces: 
    // |LHP| + |LAP| = |LKP|
    joint_position_map["huboplus_lowerBody::LHP"] = -45*3.1416/180.0;
    joint_position_map["huboplus_lowerBody::RHP"] = -45*3.1416/180.0;

    joint_position_map["huboplus_lowerBody::LKP"] =  90*3.1416/180.0;
    joint_position_map["huboplus_lowerBody::RKP"] =  90*3.1416/180.0;

    joint_position_map["huboplus_lowerBody::LAP"] = -45*3.1416/180.0;
    joint_position_map["huboplus_lowerBody::RAP"] = -45*3.1416/180.0;


    // Keep the rest 0 by now
    joint_position_map["huboplus_lowerBody::LHY"] = 0.0;
    joint_position_map["huboplus_lowerBody::RHY"] = 0.0;

    joint_position_map["huboplus_lowerBody::LHR"] = 0.0;
    joint_position_map["huboplus_lowerBody::RHR"] = 0.0;

    joint_position_map["huboplus_lowerBody::LAR"] = 0.0;
    joint_position_map["huboplus_lowerBody::RAR"] = 0.0;


    this->mModel->SetJointPositions( joint_position_map );

  }

  /**
   * @function UpdateStates
   * @brief
   */
  void lowerBody_jointControl::UpdateStates() {
    
    common::Time cur_time = this->mWorld->GetSimTime();
    
    bool is_paused = this->mWorld->IsPaused();
    if( !is_paused ) this->mWorld->SetPaused(true);

    std::map<std::string, double> joint_position_map;

    joint_position_map["huboplus_lowerBody::LHP"] = -45*3.1416/180.0;
    joint_position_map["huboplus_lowerBody::RHP"] = -45*3.1416/180.0;

    joint_position_map["huboplus_lowerBody::LKP"] = 90*3.1416/180.0;
    joint_position_map["huboplus_lowerBody::RKP"] = 90*3.1416/180.0;

    joint_position_map["huboplus_lowerBody::LAP"] = -45*3.1416/180.0;
    joint_position_map["huboplus_lowerBody::RAP"] = -45*3.1416/180.0;


    // Keep the rest 0 by now
    joint_position_map["huboplus_lowerBody::LHY"] = 0.0;
    joint_position_map["huboplus_lowerBody::RHY"] = 0.0;

    joint_position_map["huboplus_lowerBody::LHR"] = 0.0;
    joint_position_map["huboplus_lowerBody::RHR"] = 0.0;

    joint_position_map["huboplus_lowerBody::LAR"] = 0.0;
    joint_position_map["huboplus_lowerBody::RAR"] = 0.0;

    this->mModel->SetJointPositions( joint_position_map );

    // Resume original pause-state
    this->mWorld->SetPaused( is_paused );
    
  }
  
  /**
   * @function FixLink
   * @brief
   */
  void lowerBody_jointControl::FixLink( physics::LinkPtr _link ) {
    
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
  void lowerBody_jointControl::UnfixLink() {
    this->mJoint.reset();
  }
    

  GZ_REGISTER_MODEL_PLUGIN( lowerBody_jointControl )
} // end namespace gazebo

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

    std::map<std::string, double> joint_position_map;
    
    joint_position_map["huboplus::LHP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::RHP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::LHY"] = 0.0;
    joint_position_map["huboplus::RHY"] = 0.0;
    joint_position_map["huboplus::LHR"] = 0.3;
    joint_position_map["huboplus::RHR"] = -0.3;

    joint_position_map["huboplus::LKP"] =  20*3.1416/180.0;
    joint_position_map["huboplus::RKP"] =  20*3.1416/180.0;

    joint_position_map["huboplus::LAP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::RAP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::LAR"] = 0.3;
    joint_position_map["huboplus::RAR"] = 0.3;

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

    std::map<std::string, double> joint_velocity_map;

    joint_velocity_map["huboplus::LHP"] = -10*3.1416/180.0;
    joint_velocity_map["huboplus::RHP"] = -10*3.1416/180.0;
    joint_velocity_map["huboplus::LHY"] = 0.0;
    joint_velocity_map["huboplus::RHY"] = 0.0;
    joint_velocity_map["huboplus::LHR"] = 0.3;
    joint_velocity_map["huboplus::RHR"] = -0.3;

    joint_velocity_map["huboplus::LKP"] =  20*3.1416/180.0;
    joint_velocity_map["huboplus::RKP"] =  20*3.1416/180.0;

    joint_velocity_map["huboplus::LAP"] = -10*3.1416/180.0;
    joint_velocity_map["huboplus::RAP"] = -10*3.1416/180.0;
    joint_velocity_map["huboplus::LAR"] = 0.3;
    joint_velocity_map["huboplus::RAR"] = 0.3;



    this->mModel->SetJointVelocities( joint_position_map );

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

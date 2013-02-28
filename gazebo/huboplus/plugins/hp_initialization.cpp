/**
 * @file hp_initialization.cpp
 */
#include "hp_initialization.h"

namespace gazebo {

  /**
   * @function
   * @brief
   */
  hp_initialization::hp_initialization() {
  }

  /**
   * @function ~hp_initialization
   * @brief Destructor
   */  
  hp_initialization::~hp_initialization() {

    event::Events::DisconnectWorldUpdateStart( this->mUpdateConnection ); 
  }
  
  /**
   * @function Load
   * @brief
   */
  void hp_initialization::Load( physics::ModelPtr _parent, 
				sdf::ElementPtr _sdf ) {
    printf("Load from hp_initialization \n");
    this->mModel = _parent;
    this->mWorld = this->mModel->GetWorld();


    SetHardCodedInitialPose();

    // Set to update every world cycle. Listen to update event
    this->mUpdateConnection = event::Events::ConnectWorldUpdateStart( boost::bind(&hp_initialization::UpdateStates, this));
    
  }
  
  /**
   * @function SetHardCodedInitialPose
   * @brief
   */
  void hp_initialization::SetHardCodedInitialPose() {
    

    std::map<std::string, double> joint_position_map;

    joint_position_map["huboplus::LSR"] = 0.3;
    joint_position_map["huboplus::RSR"] = -0.3;
    joint_position_map["huboplus::LSP"] = 0.4;
    joint_position_map["huboplus::RSP"] = 0.4;

    joint_position_map["huboplus::LHP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::RHP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::LKP"] =  20*3.1416/180.0;
    joint_position_map["huboplus::RKP"] =  20*3.1416/180.0;
    joint_position_map["huboplus::LAP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::RAP"] = -10*3.1416/180.0;


    joint_position_map["huboplus::LHY"] = 0.0;
    joint_position_map["huboplus::RHY"] = 0.0;
    joint_position_map["huboplus::LHR"] = 0.3;
    joint_position_map["huboplus::RHR"] = -0.3;
    joint_position_map["huboplus::LAR"] = 0.3;
    joint_position_map["huboplus::RAR"] = 0.3;

    this->mModel->SetJointPositions( joint_position_map );

    /*
    int count = 0;
    for( physics::Joint_V::const_iterator j = this->mModel->GetJoints().begin();
	 j != this->mModel->GetJoints().end(); ++j ) {
      (*j)->SetAngle(0, 0.5);
      count++;
    }
    printf("Set joints %d times \n", count);
    */


  }

  /**
   * @function UpdateStates
   * @brief
   */
  void hp_initialization::UpdateStates() {
    
    common::Time cur_time = this->mWorld->GetSimTime();
    /*
    bool is_paused = this->mWorld->IsPaused();
    if( !is_paused ) this->mWorld->SetPaused(true);

    std::map<std::string, double> joint_position_map;

    joint_position_map["huboplus::LSR"] = 0.3;
    joint_position_map["huboplus::RSR"] = -0.3;
    joint_position_map["huboplus::LSP"] = 0.4;
    joint_position_map["huboplus::RSP"] = 0.4;

    joint_position_map["huboplus::LHP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::RHP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::LKP"] =  20*3.1416/180.0;
    joint_position_map["huboplus::RKP"] =  20*3.1416/180.0;
    joint_position_map["huboplus::LAP"] = -10*3.1416/180.0;
    joint_position_map["huboplus::RAP"] = -10*3.1416/180.0;

    joint_position_map["huboplus::LHY"] = 0.0;
    joint_position_map["huboplus::RHY"] = 0.0;
    joint_position_map["huboplus::LHR"] = 0.0;
    joint_position_map["huboplus::RHR"] = 0.0;
    joint_position_map["huboplus::LAR"] = 0.0;
    joint_position_map["huboplus::RAR"] = 0.0;

    this->mModel->SetJointPositions( joint_position_map );

    // Resume original pause-state
    this->mWorld->SetPaused( is_paused );
    */
  }
  
  /**
   * @function FixLink
   * @brief
   */
  void hp_initialization::FixLink( physics::LinkPtr _link ) {
    
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
  void hp_initialization::UnfixLink() {
    this->mJoint.reset();
  }
    

  GZ_REGISTER_MODEL_PLUGIN( hp_initialization )
} // end namespace gazebo

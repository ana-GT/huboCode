/**
  * @file hiWorld.cpp
  */
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


namespace gazebo
{
  /**
   * @class hiWorld
   * @brief Set initial pose for left arm to Hi gesture
   */
  class hiWorld : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      printf("Set initial pose \n");
      std::map<std::string, double> joint_position_map;
      
      joint_position_map["huboplus::LSP"] = 0*3.1416/180.0;
      joint_position_map["huboplus::LSR"] = 15*3.1416/180.0;
      joint_position_map["huboplus::LSY"] = 85*3.1416/180.0;
      joint_position_map["huboplus::LEP"] = -125*3.1416/180.0;
      joint_position_map["huboplus::LWY"] = 20*3.1416/180.0;
      joint_position_map["huboplus::LWP"] = -35*3.1416/180.0;
			joint_position_map["huboplus::leftIndexKnuckle1"] = -60*3.1416/180.0;
			joint_position_map["huboplus::leftMiddleKnuckle1"] = -60*3.1416/180.0;
			joint_position_map["huboplus::leftPinkyKnuckle1"] = -60*3.1416/180.0;
			joint_position_map["huboplus::leftRingKnuckle1"] = -60*3.1416/180.0;
			joint_position_map["huboplus::leftThumbKnuckle1"] = -120*3.1416/180.0;
      
      _parent->SetJointPositions( joint_position_map );   
    }
    
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(hiWorld)
}

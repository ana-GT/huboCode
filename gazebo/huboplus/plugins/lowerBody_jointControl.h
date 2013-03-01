/**
 * @file lowerBody_jointControl.h
 */

#ifndef _LOWERBODY_JOINTCONTROL_H_
#define _LOWERBODY_JOINTCONTROL_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


namespace gazebo {

  /**
   * @class lowerBody_jointControl
   */
  class lowerBody_jointControl : public ModelPlugin
  {
  public:
    lowerBody_jointControl();
    ~lowerBody_jointControl();
    
    void Load( physics::ModelPtr _parent, 
	       sdf::ElementPtr _sdf );

    void SetInitialPose();
  private:
    void UpdateStates();
    void FixLink( physics::LinkPtr _link );
    void UnfixLink();
    
    physics::WorldPtr mWorld;
    physics::ModelPtr mModel;
    physics::JointPtr mJoint;

    boost::mutex mUpdate_Mutex;

    // Pointer to the update event connection
    event::ConnectionPtr mUpdateConnection;
  };

}

#endif /** _LOWERBODY_JOINTCONTROL_H_ */

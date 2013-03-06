/**
 * @file foreleg_jointControl.h
 */

#ifndef _FORELEG_JOINTCONTROL_H_
#define _FORELEG_JOINTCONTROL_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include "../controlBundle.h"
#include "../pid_controller.h"

namespace gazebo {

  /**
   * @class foreleg_jointControl
   */
  class foreleg_jointControl : public ModelPlugin
  {
  public:
    foreleg_jointControl();
    ~foreleg_jointControl();
    
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

    controlBundle mCb;

    common::Time mLastUpdateTime;

    boost::mutex mUpdate_Mutex;

    // Pointer to the update event connection
    event::ConnectionPtr mUpdateConnection;
  };

}

#endif /** _FORELEG_JOINTCONTROL_H_ */

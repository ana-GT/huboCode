/**
 * @file cog_visual.h
 */

#ifndef _COG_VISUAL_H_
#define _COG_VISUAL_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo {

  /**
   * @class cog_visual
   */
  class cog_visual : public ModelPlugin
  {
  public:
    cog_visual();
    ~cog_visual();
    
    void Load( physics::ModelPtr _parent, 
	       sdf::ElementPtr _sdf );

  private:
    void UpdateView();

    
    physics::WorldPtr mWorld;
    physics::ModelPtr mModel;

    common::Time mLastUpdateTime;

    boost::mutex mUpdate_Mutex;

    // Pointer to the update event connection
    event::ConnectionPtr mUpdateConnection;
  };

}

#endif /** _COG_VISUAL_H_ */

/**
 * @file cog_visual.cpp
 */
#include "cog_visual.h"
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/physics/PhysicsTypes.hh> // For Link_V

namespace gazebo {

/**
 * @function
 * @brief
 */
cog_visual::cog_visual() {
}

/**
 * @function ~cog_visual
 * @brief Destructor
 */  
cog_visual::~cog_visual() {
  
  event::Events::DisconnectWorldUpdateStart( this->mUpdateConnection ); 
}

/**
 * @function Load
 * @brief
 */
void cog_visual::Load( physics::ModelPtr _parent, 
		       sdf::ElementPtr _sdf ) {
  
  printf("Load from cog_visual \n");
  this->mModel = _parent;
  this->mWorld = this->mModel->GetWorld();
  
  mLastUpdateTime = mModel->GetWorld()->GetSimTime();
  
  // Set to update every world cycle. Listen to update event
  this->mUpdateConnection = event::Events::ConnectWorldUpdateStart( boost::bind(&cog_visual::UpdateView, this));
  
  physics::Link_V links = mModel->GetLinks();

  std::cout << "Number of links: "<< links.size() << std::endl;
  for( int i = 0; i < mModel->GetLinks().size(); ++i ) {
      math::Pose p = links[i]->GetWorldCoGPose();
      std::cout << "Pose for link "<< links[i]->GetName()<<std::endl;
      std::cout<<": "<<p.pos.x<<", "<<p.pos.y<<", "<<p.pos.z <<std::endl;    
  }

}


/**
 * @function UpdateView
 * @brief
 */
void cog_visual::UpdateView() {
  
  // Get current time and dt
  common::Time current_time = mModel->GetWorld()->GetSimTime();
  double dt = current_time.Double() - mLastUpdateTime.Double();
    
  mLastUpdateTime = current_time;
  
}

  GZ_REGISTER_MODEL_PLUGIN( cog_visual )
} // end namespace gazebo

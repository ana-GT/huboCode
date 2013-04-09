/**
 * @file gaiter.h
 * @brief Gait generator for Hubo lower body
 * @author A. Huaman Quispe
 */

#ifndef _HUBOPLUS_GAITER_
#define _HUBOPLUS_GAITER_

#include <gazebo/physics/physics.hh>
#include <controls/controlBundle.h>

typedef enum {
  RAISE_UP,
  SWING,
  RAISE_LEFT,
  RAISE_RIGHT,
  NO_STARTED,
  FAILURE,
  END
} STATE;


/**
 * @class gaiter
 */
class gaiter {

 public:
  gaiter();
  ~gaiter();
  
  bool init( physics::ModelPtr _model,
	     int _numActuatedJoints,
	     std::vector<physics::JointPtr> _actuatedJoints );
  void update( double _currentTime, double _dt );

  // Behavior functions

  // Raise Up
  void run_RaiseUp( double _currentTime, double _dt  );
  void init_RaiseUp( double _startTime );

  // Swing
  void run_Swing( double _currentTime, double _dt  );
  void init_Swing( double _startTime );

  // Raise Left
  void run_RaiseLeft( double _currentTime, double _dt  );
  void init_RaiseLeft( double _startTime );

 private:
  int mState;
  double mStartBehaviorTime;
  physics::ModelPtr mModel;
  controlBundle mCb;
  int mNumActuatedJoints;
  std::vector<physics::JointPtr> mActuatedJoints;
};


#endif /** _HUBOPLUS_GAITER_ */

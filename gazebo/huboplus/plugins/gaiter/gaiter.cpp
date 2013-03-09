/**
 * @file gaiter.cpp
 * @brief Function implementations 
 */

#include "gaiter.h"

/**
 * @function gaiter
 * @brief Constructor
 */
gaiter::gaiter() {

  mState = NO_STARTED;
  mState = RISE_UP;
}

/**
 * @function ~gaiter
 * @brief
 */
gaiter::~gaiter() {

}

/**
 * @function init
 * @brief Set model to control
 */
bool gaiter::init( physics::ModelPtr _model,
		   int _numActuatedJoints,
		   std::vector<physics::JointPtr> _actuatedJoints ) {
  
  // Give access to the model to all behaviors
  mModel = _model;

  // Set actuated joints
  mNumActuatedJoints = _numActuatedJoints;
  mActuatedJoints = _actuatedJoints;
  // Set info to controller
  mCb.setSize( mNumActuatedJoints );    
  mCb.setJoints(mActuatedJoints);

  // Call initialization procedures for particular behaviors
  init_RaiseUp();
  
}


/**
 * @function update
 * @brief
 */
void gaiter::update( double _currentTime, double _dt ) {


  switch( mState ) {
    
    /** Rise up */
  case RISE_UP: {
    run_RaiseUp( _currentTime, _dt );
    
  }
    break;

    /** Rise left leg */
  case RISE_LEFT: {
    run_RaiseLeft( _currentTime, _dt );
  }
    break;

  } // end switch

}

/*************** RAISE UP *****************************/
/**
 * @function run_RaiseUp
 */
void gaiter::run_RaiseUp( double _currentTime, double _dt  ) {

  // Change of behavior if time is up
  if( _currentTime > 8.0 ) {
    mState = RISE_LEFT;
    init_RaiseLeft( _currentTime );
    return;
  }
  
  // Else
  else if( _currentTime > 3.0 ) {

    // Move ankles (LAP) until time is up
    if( _currentTime < 5.0 ) {
    mCb.mTargets[2] = mCb.mTargets[2] + _dt*(5.0*3.1416/180.0);
    mCb.mTargets[6] = mCb.mTargets[6] + _dt*(5.0*3.1416/180.0);
    }

    // LKP and RKP
    mCb.mTargets[1] = mCb.mTargets[1] - _dt*(5.0*3.1416/180.0);
    mCb.mTargets[5] = mCb.mTargets[5] - _dt*(5.0*3.1416/180.0);

  }
  
  // Update controllers
  mCb.updateControls(_dt);

}

/**
 * @function initRaiseUp
 */
void gaiter::init_RaiseUp() {
  
  // Set targets as their initial values
  std::vector<double> targets( mNumActuatedJoints );
  for( int i = 0; i < mNumActuatedJoints; ++i ) {
    targets[i] = mActuatedJoints[i]->GetAngle(0).Radian(); 
  }  
  mCb.setTargets(targets);
  
  // Set PID initial values
  mCb.initPID( 0, 75, 0, 7.5, 0, 0, 75, -75 );
  mCb.initPID( 1, 1000, 0, 40, 0, 0, 1000, -1000 ); // LKP
  mCb.initPID( 2, 100, 0, 5, 0, 0, 100, -100 );
  mCb.initPID( 3, 50, 0, 5, 0, 0, 50, -50 );
  
  mCb.initPID( 4, 75, 0, 7.5, 0, 0, 75, -75 );
  mCb.initPID( 5, 1000, 0, 40, 0, 0, 1000, -1000 ); // RKP
  mCb.initPID( 6, 100, 0, 5, 0, 0, 100, -100 );
  mCb.initPID( 7, 50, 0, 5, 0, 0, 50, -50 );
  
  mCb.initPID( 8, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 9, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 10, 50, 0, 5, 0, 0, 50, -50 ); // LHR
  mCb.initPID( 11, 50, 0, 5, 0, 0, 50, -50 ); // RHR
  
  mCb.initPID( 12, 50, 0, 5, 0, 0, 50, -50 );    
  
}

/*************** RAISE LEFT *****************************/
/**
 * @function run_RaiseLeft
 * @brief Raise straight right while tucking left
 */
void gaiter::run_RaiseLeft( double _currentTime, double _dt  ) {


  // Change of behavior if time is up
  if( _currentTime - mStartBehaviorTime  > 3.0 ) {
    //mState = FORWARD_LEFT;
    //init_ForwardLeft( _currentTime );
    return;
  }
  
  // Else
  else {

    // Move ankles (RAP) until time is up
      mCb.mTargets[6] = mCb.mTargets[6] + _dt*(5.0*3.1416/180.0);
    // RKP
      mCb.mTargets[5] = mCb.mTargets[5] - _dt*(5.0*3.1416/180.0);    
    // LKP and RKP
    //mCb.mTargets[1] = mCb.mTargets[1] - _dt*(5.0*3.1416/180.0);
    //mCb.mTargets[5] = mCb.mTargets[5] - _dt*(5.0*3.1416/180.0);
    
  }
  
  // Update controllers
  mCb.updateControls(_dt);

}

/**
 * @function initRaiseLeft
 */
void gaiter::init_RaiseLeft( double _startTime ) {
  
  // Keep targets as their current values
  std::vector<double> targets( mNumActuatedJoints );
  for( int i = 0; i < mNumActuatedJoints; ++i ) {
    targets[i] = mActuatedJoints[i]->GetAngle(0).Radian(); 
  }  
  mCb.setTargets(targets);

  mStartBehaviorTime = _startTime;
  
  
}



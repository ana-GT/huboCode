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
  // Init with current time
  common::Time currentTime = mModel->GetWorld()->GetSimTime();
  init_RaiseUp( currentTime.Double() );
  
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

  double startLegTime = 3.0;
  double startHipTime = 6.0;
  double endTime = 7.0;

  // Change of behavior if time is up
  if( (_currentTime - mStartBehaviorTime) > endTime ) {
    mState = RISE_LEFT;
    init_RaiseLeft( _currentTime );
    return;
  }
  
  // Else
  else if( (_currentTime - mStartBehaviorTime)  > startLegTime ) {
    
    if( (_currentTime - mStartBehaviorTime) < startHipTime ) {
      // Ankles: LAP & RAP
      mCb.mTargets[8] = mCb.mTargets[8] + _dt*(5.0*3.1416/180.0);
      mCb.mTargets[9] = mCb.mTargets[9] + _dt*(5.0*3.1416/180.0);

      // Hips: LHP && RHP (0 change)
      mCb.mTargets[0] = mCb.mTargets[0] + _dt*(0.0*3.1416/180.0);
      mCb.mTargets[1] = mCb.mTargets[1] + _dt*(0.0*3.1416/180.0);

      // Knees: LKP and RKP
      mCb.mTargets[6] = -1*( mCb.mTargets[8] + mCb.mTargets[0] );
      mCb.mTargets[7] = -1*( mCb.mTargets[9] + mCb.mTargets[1] ); // - dt*...
    }
      
    // Start turning to the left
    else if( (_currentTime - mStartBehaviorTime) > startHipTime ) {

      // Roll ankles: LAR  & RAR
      mCb.mTargets[10] = mCb.mTargets[10] + _dt*(5.0*3.1416/180.0);
      mCb.mTargets[11] = mCb.mTargets[11] + _dt*(5.0*3.1416/180.0);
      // And hips: LHR & RHR
      mCb.mTargets[4] = mCb.mTargets[4] - _dt*(5.0*3.1416/180.0);
      mCb.mTargets[5] = mCb.mTargets[5] - _dt*(5.0*3.1416/180.0);

    }
    
  }
  
  // Update controllers
  mCb.updateControls(_dt);

}

/**
 * @function initRaiseUp
 */
void gaiter::init_RaiseUp( double _startTime ) {
  
  // Set starting time
  mStartBehaviorTime = _startTime;

  // Set targets as their initial values
  std::vector<double> targets( mNumActuatedJoints );
  for( int i = 0; i < mNumActuatedJoints; ++i ) {
    targets[i] = mActuatedJoints[i]->GetAngle(0).Radian(); 
  }  
  mCb.setTargets(targets);
  
  // Set PID initial values
  mCb.initPID( 0, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 1, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 2, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 3, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 4, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 5, 50, 0, 5, 0, 0, 50, -50 );

  mCb.initPID( 6, 1000, 0, 5, 0, 0, 1000, -1000 ); // LKP
  mCb.initPID( 7, 1000, 0, 5, 0, 0, 1000, -1000 ); // RKP

  mCb.initPID( 8, 200, 0, 10, 0, 0, 200, -200 ); // LAP
  mCb.initPID( 9, 200, 0, 10, 0, 0, 200, -200 ); // RAP
  mCb.initPID( 10, 50, 0, 5, 0, 0, 50, -50 );
  mCb.initPID( 11, 50, 0, 5, 0, 0, 50, -50 );

  mCb.initPID( 12, 50, 0, 5, 0, 0, 50, -50 ); // Torso    
  
}

/*************** RAISE LEFT *****************************/
/**
 * @function run_RaiseLeft
 * @brief Raise straight right while tucking left
 */
void gaiter::run_RaiseLeft( double _currentTime, double _dt  ) {


 double F_LHY = mActuatedJoints[2]->GetForce(0);
 double F_RHY = mActuatedJoints[3]->GetForce(0);

 double LHY = mActuatedJoints[2]->GetAngle(0).Degree();
 double RHY = mActuatedJoints[3]->GetAngle(0).Degree();

 std::cout << "Force in LHY ( "<< LHY << " ): " << F_LHY << " and RHY ( "<< RHY <<" ): "<< F_RHY << std::endl;
  /*
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
    
    } */
  
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



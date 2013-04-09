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
  mState = RAISE_UP;
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
  case RAISE_UP: {
    run_RaiseUp( _currentTime, _dt );
    
  }
    break;

    /** Swing */
  case SWING: {
    run_Swing( _currentTime, _dt );
    
  }
    break;

    /** Rise left leg */
  case RAISE_LEFT: {
    run_RaiseLeft( _currentTime, _dt );
  }
    break;

  } // end switch

}

/*************** RAISE LEFT *****************************/
/**
 * @function run_RaiseLeft
 * @brief Raise straight right while tucking left
 */
void gaiter::run_RaiseLeft( double _currentTime, double _dt  ) {
  /*
  mCb.mTargets[4] = mCb.mTargets[4] - _dt*(5.0*3.1416/180.0);
  mCb.mTargets[10] = mCb.mTargets[10] - _dt*(5.0*3.1416/180.0);
  */
  // Making yaw in hip more sturdy
  mCb.mPIDs[2].setPGain( 500 );
  mCb.mPIDs[3].setPGain( 500 );

  // Making ankle more sturdy
  mCb.mPIDs[10].setPGain( 500 );

  mCb.updateControls(_dt);     
}

/**
 * @function init_RaiseLeft
 * @brief Raise straight right while tucking left
 */
void gaiter::init_RaiseLeft( double _startTime ) {
  
  // Set starting time
  mStartBehaviorTime = _startTime;
  
  // Set targets as their initial values 
/*
  std::vector<double> targets( mNumActuatedJoints );
  for( int i = 0; i < mNumActuatedJoints; ++i ) {
    targets[i] = mActuatedJoints[i]->GetAngle(0).Radian(); 
  }  
  mCb.setTargets(targets);
*/
}


/*************** SWING *****************************/
/**
 * @function run_Swing
 * @brief Raise straight right while tucking left
 */
void gaiter::run_Swing( double _currentTime, double _dt  ) {

  /*
  double F_LHY = mActuatedJoints[2]->GetForce(0);
  double F_RHY = mActuatedJoints[3]->GetForce(0);
  
  double LHY = mActuatedJoints[2]->GetAngle(0).Degree();
  double RHY = mActuatedJoints[3]->GetAngle(0).Degree();
  
  std::cout << "Force in LHY ( "<< LHY << " ): " << F_LHY << " and RHY ( "<< RHY <<" ): "<< F_RHY << std::endl;
  */
  if( ( _currentTime - mStartBehaviorTime ) < 4.0 ) {

    // Roll ankles: LAR  & RAR
    mCb.mTargets[10] = mCb.mTargets[10] + _dt*(5.0*3.1416/180.0);
    //mCb.mTargets[11] = mCb.mTargets[11] + _dt*(5.0*3.1416/180.0);
    // And hips: LHR & RHR
    mCb.mTargets[4] = mCb.mTargets[4] - _dt*(5.0*3.1416/180.0);
    //mCb.mTargets[5] = mCb.mTargets[5] - _dt*(5.0*3.1416/180.0);

    // RKP (tuck right leg)
    mCb.mTargets[7] = mCb.mTargets[7] + _dt*(5.0*3.1416/180.0);

    mCb.mPIDs[4].setPGain( 200 );
    
    // Update controllers
    mCb.updateControls(_dt);    
  }

  else {
    std::cout<< "End Swinging LAR: "<< mActuatedJoints[10]->GetAngle(0).Degree() << std::endl;
    std::cout<< "End Swinging RAR: "<< mActuatedJoints[11]->GetAngle(0).Degree() << std::endl;
    std::cout<< "End Swinging LHR: "<< mActuatedJoints[4]->GetAngle(0).Degree() << std::endl;
    std::cout<< "End Swinging RHR: "<< mActuatedJoints[5]->GetAngle(0).Degree() << std::endl;
    
    mState = RAISE_LEFT;
    init_RaiseLeft( _currentTime );
    return;
  }

}

/**
 * @function initSwing
 */
void gaiter::init_Swing( double _startTime ) {
  
  // Keep targets as their current values
  std::vector<double> targets( mNumActuatedJoints );
  for( int i = 0; i < mNumActuatedJoints; ++i ) {
    targets[i] = mActuatedJoints[i]->GetAngle(0).Radian(); 
  }  
  mCb.setTargets(targets);

  mStartBehaviorTime = _startTime;
  
  std::cout<< "Start Swinging LAR: "<< mActuatedJoints[10]->GetAngle(0).Degree() << std::endl;
  std::cout<< "Start Swinging RAR: "<< mActuatedJoints[11]->GetAngle(0).Degree() << std::endl;
  std::cout<< "Start Swinging LHR: "<< mActuatedJoints[4]->GetAngle(0).Degree() << std::endl;
  std::cout<< "Start Swinging RHR: "<< mActuatedJoints[5]->GetAngle(0).Degree() << std::endl;
}


/*************** RAISE UP *****************************/
/**
 * @function run_RaiseUp
 */
void gaiter::run_RaiseUp( double _currentTime, double _dt  ) {

  double startLegTime = 3.0;
  double endTime = 6.0;

  // Change of behavior if time is up
  if( (_currentTime - mStartBehaviorTime) > endTime ) {
    mState = SWING;
    init_Swing( _currentTime );
    return;
  }
  
  // Else
  else if( (_currentTime - mStartBehaviorTime)  > startLegTime ) {
    
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

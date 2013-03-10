  double startLegTime = 3.0;
  double startHipTime = 5.0;
  double endTime = 8.0;

  // Change of behavior if time is up
  if( (_currentTime - mStartBehaviorTime) > endTime ) {
    mState = RISE_LEFT;
    init_RaiseLeft( _currentTime );
    return;
  }
  
  // Else
  else if( (_currentTime - mStartBehaviorTime)  > startLegTime ) {

    // Knees: LKP and RKP
    mCb.mTargets[1] = mCb.mTargets[1] - _dt*(5.0*3.1416/180.0);
    mCb.mTargets[5] = mCb.mTargets[5] - _dt*(5.0*3.1416/180.0);
    


    // Move ankles (LAP) until time is up
    if( (_currentTime - mStartBehaviorTime) < startHipTime ) {
      mCb.mTargets[2] = mCb.mTargets[2] + _dt*(5.0*3.1416/180.0);
      mCb.mTargets[6] = mCb.mTargets[6] + _dt*(5.0*3.1416/180.0);
    }
      
    // Start hips: LHP and RHP
    if( (_currentTime - mStartBehaviorTime) > startHipTime ) {
      mCb.mTargets[0] = mCb.mTargets[0] + _dt*(5.0*3.1416/180.0);
      mCb.mTargets[4] = mCb.mTargets[4] + _dt*(5.0*3.1416/180.0);
    }
    
  }
  
  // Update controllers
mCb.updateControls(_dt);

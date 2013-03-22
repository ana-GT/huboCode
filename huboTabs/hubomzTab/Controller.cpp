/**
 * @file Controller.cpp
 * @brief Controller for waypoints input and frequency
 * @author
 */
#include "Controller.h"
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/Dof.h>
#include <iostream>
#include <dynamics/BodyNodeDynamics.h>
#include <utils/UtilsMath.h>


/**
 * @function Controller
 * @brief Constructor
 */
Controller::Controller( dynamics::SkeletonDynamics* _skel,
			const std::vector<int> &_actuatedDofs,
			const Eigen::VectorXd &_Kp,
			const Eigen::VectorXd &_Kd,
			const std::vector<int> &_ankleDofs,
			const Eigen::VectorXd &_Kp_Ankle,
			const Eigen::VectorXd &_Kd_Ankle ) :
  mSkel( _skel ),
  mKp( _Kp.asDiagonal() ),
  mKd( _Kd.asDiagonal() ),
  mAnkleDofs( _ankleDofs ),
  mKp_Ankle( _Kp_Ankle ),
  mKd_Ankle( _Kd_Ankle )
{
  const int nDof = mSkel->getNumDofs();
  mSelectionMatrix = Eigen::MatrixXd::Zero( nDof, nDof );
  for( int i = 0; i < _actuatedDofs.size(); ++i ) {
    mSelectionMatrix( _actuatedDofs[i], _actuatedDofs[i] ) = 1.0;
  }

  // Set desired dof to the current pose (As the robot is when you create the controller pointer)
  mDesiredDofs.resize( nDof );
  for( int i = 0; i < nDof; ++i ) {
    mDesiredDofs[i] = mSkel->getDof(i)->getValue();
  }

  // Not sure what this does but Tobi put it there
  Eigen::Vector3d com = mSkel->getWorldCOM();
  double cop = 0.0;
  mPreOffset = com[0] - cop;
}

/**
 * @function ~Controller
 * @brief destructor
 */
Controller::~Controller() {

}

/**
 * @function setWaypoints
 * @brief Set the waypoints the joints will go trhough at each time step
 */	   
void Controller::setWaypoints( std::vector<Eigen::VectorXd> _waypoints,
			       const std::vector<int> &_dofs,
			       const std::vector<double> &_nominalWaypointVelocities,
			       double _startTime,
			       double _dt ) {

  mWaypointDofs = _dofs;
  mStartTime = _startTime;
  mWaypoints = _waypoints;
  mNumWaypoints = mWaypoints.size();
  mdt = _dt;
  mNominalWaypointVelocities = _nominalWaypointVelocities;

}
  
/**
 * @function getTorques
 * @brief
 */
Eigen::VectorXd Controller::getTorques( const Eigen::VectorXd &_dof,
					const Eigen::VectorXd &_dofVel,
					double _time ) {
  
  Eigen::VectorXd desiredDofVels = VectorXd::Zero( mSkel->getNumDofs() );
  
  double currentTime = _time - mStartTime;
  int currentWaypoint = (int) ( currentTime / mdt );
  if( mWaypoints.size() > 0 && 
      currentTime >= 0.0 & 
      currentWaypoint < mNumWaypoints ) {
    
    for(unsigned int i = 0; i < mWaypointDofs.size(); i++) {
      mDesiredDofs( mWaypointDofs[i] ) = mWaypoints[ currentWaypoint ](i);
      desiredDofVels( mWaypointDofs[i] ) = mNominalWaypointVelocities[i];
    }
  }
  
  Eigen::VectorXd torques;
  const double mTimestep = mdt;
  
  // SPD controller
  // J. Tan, K. Liu, G. Turk. Stable Proportional-Derivative Controllers. IEEE Computer Graphics and Applications, Vol. 31, No. 4, pp 34-44, 2011.
  MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  VectorXd p = -mKp * (_dof - mDesiredDofs + _dofVel * mTimestep);
  VectorXd d = -mKd * (_dofVel - desiredDofVels);
  VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d);
  torques = p + d - mKd * qddot * mTimestep;
  
  // ankle strategy for sagital plane
  Vector3d com = mSkel->getWorldCOM();
  double cop = 0.0;
  double offset = com[0] - cop;
  
  for(unsigned int i = 0; i < mAnkleDofs.size(); i++) {
    torques[mAnkleDofs[i]] = - mKp_Ankle[i] * offset - mKd_Ankle[i] * (offset - mPreOffset) / mTimestep;
  }
  
  mPreOffset = offset;

  return mSelectionMatrix * torques;
  
}

/**
 * @function
 * @brief
 */
Eigen::Vector3d Controller::evalAngMomentum( const Eigen::VectorXd &_dofVel ) {

}

/**
 * @function
 * @brief
 */
Eigen::VectorXd Controller::adjustAngMomentum( Eigen::VectorXd _deltaMomentum,
					       Eigen::VectorXd _controlledAxis ) {

}



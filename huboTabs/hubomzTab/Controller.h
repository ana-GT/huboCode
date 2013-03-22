/**
 * @file Controller.h
 * @author T. Kunz
 */

#pragma once

#include <vector>
#include <Eigen/Core>

namespace dynamics { class SkeletonDynamics; }

class Controller {

 public:
  Controller( dynamics::SkeletonDynamics* _skel,
	      const std::vector<int> &_actuatedDofs,
	      const Eigen::VectorXd &_Kp,
	      const Eigen::VectorXd &_Kd,
	      const std::vector<int> &_ankleDof,
	      const Eigen::VectorXd &_Kp_Ankle,
	      const Eigen::VectorXd &_Kd_Ankle );

  ~Controller();
	   
  void setWaypoints( std::vector<Eigen::VectorXd> _waypoints,
		     const std::vector<int> &_dofs,
		     const std::vector<double> &_nominalWaypointVelocities,
		     double _startTime,
		     double _dt );
  
  // Returns zero torque for non-actuated DOFs
  Eigen::VectorXd getTorques( const Eigen::VectorXd &_dof,
			      const Eigen::VectorXd &_dofVel,
			      double _time );

 protected:
  Eigen::Vector3d evalAngMomentum( const Eigen::VectorXd &_dofVel );
  Eigen::VectorXd adjustAngMomentum( Eigen::VectorXd _deltaMomentum,
				     Eigen::VectorXd _controlledAxis );

  dynamics::SkeletonDynamics *mSkel;
  std::vector<int> mWaypointDofs;
  std::vector<Eigen::VectorXd> mWaypoints;
  int mNumWaypoints;
  std::vector<double> mNominalWaypointVelocities;

  Eigen::VectorXd mDesiredDofs;
  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  Eigen::MatrixXd mSelectionMatrix;

  double mStartTime;
  double mdt;

  double mPreOffset;
  std::vector<int> mAnkleDofs;
  Eigen::VectorXd mKp_Ankle;
  Eigen::VectorXd mKd_Ankle;
};

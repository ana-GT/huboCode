/**
 * @file motion_rt.h
 * @brief
 * @author
 */
#ifndef __MOTION_RT_H__
#define __MOTION_RT_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

/** Define for the arm's DOF as well as for legs */
typedef Eigen::Matrix<double,6,1> Vector6d;

/** Enum of sides*/
typedef enum {
  LEFT= 0,
  RIGHT
} side;

/**
 * @class motion_rt
 */
class motion_rt {
  
 public:
  motion_rt();
  ~motion_rt();
  void setSkel( dynamics::SkeletonDynamics* _skel );
  void initialize();
  void print_DofInfo();
  void testLeftArmFK();
  void testRightArmFK();
  void testLeftLegFK();
  void testRightLegFK();

  void testInverseTF();

  // ~~~*** Kinematics ***~~~ //
  inline double min(double x, double y) { return ( x > y ) ? y : x; }
  inline double max(double x, double y) { return ( x < y ) ? y : x; }
  
  
  void DH2HG(Eigen::Isometry3d &B, 
	     double t, double f, 
	     double r, double d );
  
  void huboArmFK( Eigen::Isometry3d &B, 
		  Eigen::VectorXd &q, 
		  int side);


  void huboArmFK( Eigen::Isometry3d &B, 
		  Eigen::VectorXd &q, 
		  int side,  
		 const Eigen::Isometry3d &endEffector);
  void huboLegFK(Eigen::Isometry3d &B,  Eigen::VectorXd &q, int side);

  // Test stuff
  dynamics::SkeletonDynamics* mSkel;

  // Transformations
  Eigen::MatrixXd mT_Torso2Neck;
  Eigen::MatrixXd mT_HandToEE;
  Eigen::MatrixXd mT_AnkleToFoot;

  // Arm
  int mNumArmDofs;
  std::vector<std::string> mLeftArmLinkNames;
  std::vector<int> mLeftArmDofIds;
  std::vector<std::string> mRightArmLinkNames;
  std::vector<int> mRightArmDofIds;

  Eigen::VectorXd mLeftArmConfig;
  Eigen::VectorXd mRightArmConfig;

  // Leg
  int mNumLegDofs;
  std::vector<std::string> mLeftLegLinkNames;
  std::vector<int> mLeftLegDofIds;
  std::vector<std::string> mRightLegLinkNames;
  std::vector<int> mRightLegDofIds;

  Eigen::VectorXd mLeftLegConfig;
  Eigen::VectorXd mRightLegConfig;

};

#endif /** __MOTION_RT_H__ */

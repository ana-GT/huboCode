/**
 * @file motion_rt.h
 * @brief
 * @author
 */
#ifndef __MOTION_RT_H__
#define __MOTION_RT_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>
#include <complex>


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
  void testLegIK();

  void tesArmIKwithURDF();
  void tesLegIKwithURDF();

  void testInverseTF();

  // ~~~*** Kinematics ***~~~ //
  // Useful functions
  // Useful functions
  inline double mod(double x, double y) { 
    if (0 == y) { return x; }   
    return x - y * floor(x/y); 
  }
  
  inline double wrapToPi(double fAng) { return mod(fAng + M_PI, 2*M_PI) - M_PI; }
  inline double min(double x, double y) { return ( x > y ) ? y : x; }
  inline double max(double x, double y) { return ( x < y ) ? y : x; }
  
  
  void DH2HG(Eigen::Isometry3d &B, 
	     double t, double f, 
	     double r, double d );
  
  void huboArmFK( Eigen::Isometry3d &B, 
		  Eigen::VectorXd &q, 
		  int side);

  void huboArmIK(Eigen::VectorXd &q, 
		 Eigen::Isometry3d B, 
		 Eigen::VectorXd qPrev, int side);
  void huboArmIK(Eigen::VectorXd &q, const Eigen::Isometry3d B, 
		 Eigen::VectorXd qPrev, int side, 
		 const Eigen::Isometry3d &endEffector);

  void huboArmFK( Eigen::Isometry3d &B, 
		  Eigen::VectorXd &q, 
		  int side,  
		 const Eigen::Isometry3d &endEffector);
  void huboLegFK(Eigen::Isometry3d &B,  Eigen::VectorXd &q, int side);

  void huboLegIK(Eigen::VectorXd &q, const Eigen::Isometry3d B, Eigen::VectorXd qPrev, int side);
  void huboLegIK2( Eigen::VectorXd &q, 
		   const Eigen::Isometry3d B, 
		   Eigen::VectorXd qPrev, int side);

 
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

  Eigen::VectorXd mArmLengths;
  Eigen::VectorXd mLegLengths;

};

#endif /** __MOTION_RT_H__ */

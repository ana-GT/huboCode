/**
 * @file motion_rt.cpp
 * @brief
 * @author 
 */
#include <dynamics/SkeletonDynamics.h>
#include "kinematics/BodyNode.h"
#include "kinematics/Dof.h"
#include "motion_rt.h"
#include <iostream>

/**
 * @function testInverseTF
 */
void motion_rt::testInverseTF() {
  Eigen::MatrixXd torso;
  Eigen::MatrixXd tFootToTorso;
  Eigen::MatrixXd tWorldToFoot;
  Eigen::MatrixXd tTorsoToFoot;
  Eigen::MatrixXd tPosTorso;

  mLeftLegConfig << 0, 0, 0, 0, 0, 0;
  mRightLegConfig << 0, 0, 0, 0, 0, 0;
  mSkel->setConfig( mLeftLegDofIds, mLeftLegConfig );
  mSkel->setConfig( mRightLegDofIds, mRightLegConfig );
  torso = mSkel->getNode("Body_Torso")->getWorldTransform();
  std::cout<< "Torso location after pitch in ankle of 0.0: "<< torso(0,3)<<","<<torso(1,3)<<","<<torso(2,3)<<std::endl;

  tTorsoToFoot = mSkel->getNode("Body_LAP")->getWorldTransform();
  std::cout<< "Ankle location after pitch in ankle of 0.0: "<< tTorsoToFoot(0,3)<<","<<  tTorsoToFoot(1,3)<<","<<  tTorsoToFoot(2,3)<<std::endl;
  tFootToTorso = tTorsoToFoot.inverse();
  tPosTorso = ( ( tFootToTorso.block(0,0,3,3) ).inverse() )*( tFootToTorso.block(0,3,3,1) );
  std::cout<< "Foot to torso 0.0: \n"<<tFootToTorso<<std::endl;  

  std::cout<< "Torso location after pitch in ankle of 0.0 (INVERSE): "<< tPosTorso(0,0)<<","<< tPosTorso(1,0)<<","<< tPosTorso(2,0)<<std::endl;


  mLeftLegConfig << 0, 0, 0, 0, -0.3, 0;
  mRightLegConfig << 0, 0, 0, 0, -0.3, 0;
  mSkel->setConfig( mLeftLegDofIds, mLeftLegConfig );
  mSkel->setConfig( mRightLegDofIds, mRightLegConfig );
  torso = mSkel->getNode("Body_Torso")->getWorldTransform();
  std::cout<< "Torso location after pitch in ankle of 0.3: "<< torso(0,3)<<","<<torso(1,3)<<","<<torso(2,3)<<std::endl;

  tTorsoToFoot = mSkel->getNode("Body_LAP")->getWorldTransform();
  std::cout<< "Ankle location after pitch in ankle of 0.3: "<< tTorsoToFoot(0,3)<<","<<  tTorsoToFoot(1,3)<<","<<  tTorsoToFoot(2,3)<<std::endl;
  tFootToTorso = tTorsoToFoot.inverse();
  tPosTorso = ( ( tFootToTorso.block(0,0,3,3) ).inverse() )*( tFootToTorso.block(0,3,3,1) );
  std::cout<< "Foot to torso 0.3: \n"<<tFootToTorso<<std::endl;
  std::cout<< "Torso location after pitch in ankle of 0.3 (INVERSE): "<< tPosTorso(0,0)<<","<< tPosTorso(1,0)<<","<< tPosTorso(2,0)<<std::endl;;

}

/**
 * @function testLeftArmFK
 */
void motion_rt::testLeftArmFK() {

  // ******* Left Arm *******
  mLeftArmConfig << -80, 25, 44, -33, 1, -14;
  mLeftArmConfig = mLeftArmConfig*(3.1416/180.0);

  mSkel->setConfig( mLeftArmDofIds, mLeftArmConfig );
 
  std::cout<< "Left Arm test config"<<std::endl;
  for( int i = 0; i < mLeftArmConfig.size();++i ) {
    std::cout<< mLeftArmConfig(i)<<" ";
  }
  std::cout << std::endl;

  // Current FK
  Eigen::Isometry3d BLeftArm_iso_0;
  Eigen::MatrixXd BLeftArm_0;
  huboArmFK( BLeftArm_iso_0, mLeftArmConfig, LEFT ); 
  BLeftArm_0 = BLeftArm_iso_0.matrix();
  BLeftArm_0 = mT_Torso2Neck*BLeftArm_0;

  std::cout<< "Current FK for Left Arm: \n"<<BLeftArm_0<<std::endl;
  
  // Dart Lite FK
  Eigen::MatrixXd BLeftArm_1 = mSkel->getNode( mLeftArmLinkNames[mNumArmDofs - 1].c_str() )->getWorldTransform();
  BLeftArm_1 = BLeftArm_1*mT_HandToEE;

  std::cout<< "Dart Lite FK for Left Arm: \n"<<BLeftArm_1<<std::endl;

}


/**
 * @function testRightArmFK
 */
void motion_rt::testRightArmFK() {

  // ******* Right Arm *******
  mRightArmConfig << -80, 25, 44, -33, 1, -14;
  mRightArmConfig = mRightArmConfig*(3.1416/180.0);

  mSkel->setConfig( mRightArmDofIds, mRightArmConfig );
 
  std::cout<< "Right Arm test config"<<std::endl;
  for( int i = 0; i < mRightArmConfig.size();++i ) {
    std::cout<< mRightArmConfig(i)<<" ";
  }
  std::cout << std::endl;

  // Current FK
  Eigen::Isometry3d BRightArm_iso_0;
  Eigen::MatrixXd BRightArm_0;
  huboArmFK( BRightArm_iso_0, mRightArmConfig, RIGHT ); 
  BRightArm_0 = BRightArm_iso_0.matrix();
  BRightArm_0 = mT_Torso2Neck*BRightArm_0;

  std::cout<< "Current FK for Right Arm: \n"<<BRightArm_0<<std::endl;
  
  // Dart Lite FK
  Eigen::MatrixXd BRightArm_1 = mSkel->getNode( mRightArmLinkNames[mNumArmDofs - 1].c_str() )->getWorldTransform();
  BRightArm_1 = BRightArm_1*mT_HandToEE;

  std::cout<< "Dart Lite FK for Right Arm: \n"<<BRightArm_1<<std::endl;

}


/**
 * @function testLeftLegFK
 */
void motion_rt::testLeftLegFK() {

  // ******* Left Leg *******
  mLeftLegConfig << 33,8,-24,40,-30,-6;
  mLeftLegConfig = mLeftLegConfig*(3.1416/180.0);

  mSkel->setConfig( mLeftLegDofIds, mLeftLegConfig );
 
  std::cout<< "Left Leg test config"<<std::endl;
  for( int i = 0; i < mLeftLegConfig.size();++i ) {
    std::cout<< mLeftLegConfig(i)<<" ";
  }
  std::cout << std::endl;

  // Current FK
  Eigen::Isometry3d BLeftLeg_iso_0;
  Eigen::MatrixXd BLeftLeg_0;
  huboLegFK( BLeftLeg_iso_0, mLeftLegConfig, LEFT ); 
  BLeftLeg_0 = BLeftLeg_iso_0.matrix();
  BLeftLeg_0 = mT_Torso2Neck*BLeftLeg_0;

  std::cout<< "Current FK for Left Leg: \n"<<BLeftLeg_0<<std::endl;
  
  // Dart Lite FK
  Eigen::MatrixXd BLeftLeg_1 = mSkel->getNode( mLeftLegLinkNames[mNumLegDofs - 1].c_str() )->getWorldTransform();
  BLeftLeg_1 = BLeftLeg_1*mT_AnkleToFoot;

  std::cout<< "Dart Lite FK for Left Leg: \n"<<BLeftLeg_1<<std::endl;

}


/**
 * @function testRightLegFK
 */
void motion_rt::testRightLegFK() {

  // ******* Right Leg *******
  mRightLegConfig << 33,8,-24,40,-30,-6;;
  mRightLegConfig = mRightLegConfig*(3.1416/180.0);

  mSkel->setConfig( mRightLegDofIds, mRightLegConfig );
 
  std::cout<< "Right Leg test config"<<std::endl;
  for( int i = 0; i < mRightLegConfig.size();++i ) {
    std::cout<< mRightLegConfig(i)<<" ";
  }
  std::cout << std::endl;

  // Current FK
  Eigen::Isometry3d BRightLeg_iso_0;
  Eigen::MatrixXd BRightLeg_0;
  huboLegFK( BRightLeg_iso_0, mRightLegConfig, RIGHT ); 
  BRightLeg_0 = BRightLeg_iso_0.matrix();
  BRightLeg_0 = mT_Torso2Neck*BRightLeg_0;

  std::cout<< "Current FK for Right Leg: \n"<<BRightLeg_0<<std::endl;
  
  // Dart Lite FK
  Eigen::MatrixXd BRightLeg_1 = mSkel->getNode( mRightLegLinkNames[mNumLegDofs - 1].c_str() )->getWorldTransform();
  BRightLeg_1 = BRightLeg_1*mT_AnkleToFoot;

  std::cout<< "Dart Lite FK for Right Leg: \n"<<BRightLeg_1<<std::endl;

}

/**
 * @function motion_rt
 * @brief constructor
 */
motion_rt::motion_rt() {
  
}

/**
 * @function ~motion_rt
 * @brief destructor
 */
motion_rt::~motion_rt() {

}

/**
 * @function setSkel
 * @brief 
 */
void motion_rt::setSkel( dynamics::SkeletonDynamics* _skel ) {
  mSkel = _skel;
}

/**
 * @function initialize
 * @brief 
 */
void motion_rt::initialize() {

  // Torso to Neck transformation
  mT_Torso2Neck = Eigen::MatrixXd::Identity(4,4);
  mT_Torso2Neck(0,3) = 0.012258;
  mT_Torso2Neck(2,3) = 0.0486356; 

  // Hand transformation
  mT_HandToEE;

  Eigen::MatrixXd hte_rot; hte_rot = Eigen::MatrixXd::Zero(4,4);
  // Rotate 90 around y
  hte_rot(2,0) = -1 ;
  hte_rot(1,1) = 1 ;
  hte_rot(0,2) = 1 ;
  hte_rot(3,3) = 1 ;
  Eigen::MatrixXd hte_trans = Eigen::MatrixXd::Identity(4,4);
  // Displacement
  hte_trans(0,3) = 4.75*25.4/1000.0;
  
  mT_HandToEE = hte_rot*hte_trans;

  // Foot transformation
  mT_AnkleToFoot;

  Eigen::MatrixXd atf_rot; atf_rot = Eigen::MatrixXd::Zero(4,4);
  // Rotate 90 around y
  atf_rot(2,0) = -1 ;
  atf_rot(1,1) = 1 ;
  atf_rot(0,2) = 1 ;
  atf_rot(3,3) = 1 ;
  Eigen::MatrixXd atf_trans = Eigen::MatrixXd::Identity(4,4);
  // Displacement
  atf_trans(2,3) = -0.0711787;
  atf_trans(0,3) = 94.97/1000.0;
  
  mT_AnkleToFoot = atf_rot*atf_trans;



  // Load arm DOFs
  mNumArmDofs = 6;

  // Left Arm
  mLeftArmLinkNames.resize( mNumArmDofs );
  mLeftArmDofIds.resize( mNumArmDofs );
  mLeftArmConfig.resize( mNumArmDofs );

  mLeftArmLinkNames[0] = "Body_LSP"; mLeftArmLinkNames[1] = "Body_LSR";
  mLeftArmLinkNames[2] = "Body_LSY"; mLeftArmLinkNames[3] = "Body_LEP";
  mLeftArmLinkNames[4] = "Body_LWY"; mLeftArmLinkNames[5] = "Body_LWP";
  

  for( int i = 0; i < mNumArmDofs; ++i ) {
    mLeftArmDofIds[i] =  mSkel->getNode( mLeftArmLinkNames[i].c_str() )->getDof(0)->getSkelIndex();
  }
  

  // Right Arm
  mRightArmLinkNames.resize( mNumArmDofs );
  mRightArmDofIds.resize( mNumArmDofs );
  mRightArmConfig.resize( mNumArmDofs );

  mRightArmLinkNames[0] = "Body_RSP"; mRightArmLinkNames[1] = "Body_RSR";
  mRightArmLinkNames[2] = "Body_RSY"; mRightArmLinkNames[3] = "Body_REP";
  mRightArmLinkNames[4] = "Body_RWY"; mRightArmLinkNames[5] = "Body_RWP";
  

  for( int i = 0; i < mNumArmDofs; ++i ) {
    mRightArmDofIds[i] =  mSkel->getNode( mRightArmLinkNames[i].c_str() )->getDof(0)->getSkelIndex();
  }
  

  // Load leg DOFs
  mNumLegDofs = 6;

  // Left Leg
  mLeftLegLinkNames.resize( mNumLegDofs );
  mLeftLegDofIds.resize( mNumLegDofs );
  mLeftLegConfig.resize( mNumLegDofs );

  mLeftLegLinkNames[0] = "Body_LHY"; mLeftLegLinkNames[1] = "Body_LHR";
  mLeftLegLinkNames[2] = "Body_LHP"; mLeftLegLinkNames[3] = "Body_LKP";
  mLeftLegLinkNames[4] = "Body_LAP"; mLeftLegLinkNames[5] = "Body_LAR";
  

  for( int i = 0; i < mNumLegDofs; ++i ) {
    mLeftLegDofIds[i] =  mSkel->getNode( mLeftLegLinkNames[i].c_str() )->getDof(0)->getSkelIndex();
  }
  
  // Right Leg
  mRightLegLinkNames.resize( mNumLegDofs );
  mRightLegDofIds.resize( mNumLegDofs );
  mRightLegConfig.resize( mNumLegDofs );

  mRightLegLinkNames[0] = "Body_RHY"; mRightLegLinkNames[1] = "Body_RHR";
  mRightLegLinkNames[2] = "Body_RHP"; mRightLegLinkNames[3] = "Body_RKP";
  mRightLegLinkNames[4] = "Body_RAP"; mRightLegLinkNames[5] = "Body_RAR";
  

  for( int i = 0; i < mNumLegDofs; ++i ) {
    mRightLegDofIds[i] =  mSkel->getNode( mRightLegLinkNames[i].c_str() )->getDof(0)->getSkelIndex();
  }

  // Lengths for arm/leg
  mLegLengths.resize(6);
  mArmLengths.resize(6);

  Eigen::VectorXd HPY =  mSkel->getNode("Body_HPY")->getWorldTransform().block(0,3,3,1);
  Eigen::VectorXd LHY =  mSkel->getNode("Body_LHY")->getWorldTransform().block(0,3,3,1);
  Eigen::VectorXd LHP =  mSkel->getNode("Body_LHP")->getWorldTransform().block(0,3,3,1);
  Eigen::VectorXd LKP =  mSkel->getNode("Body_LKP")->getWorldTransform().block(0,3,3,1);
  Eigen::VectorXd LAP =  mSkel->getNode("Body_LAP")->getWorldTransform().block(0,3,3,1);

  mLegLengths(0) = (79.5 + 107)/1000.0;
  mLegLengths(1) = HPY(1)- LHY(1); // L2
  mLegLengths(2) = ( 289.47-107 )/ 1000.0; // L3
  mLegLengths(3) = LHP(2) - LKP(2); // L4
  mLegLengths(4) = LKP(2) - LAP(2); // L5
  mLegLengths(5) = 0.1;
}

/**
 * @function print_DofInfo
 */
void motion_rt::print_DofInfo() {

  // Left Arm
  std::cout<<"Dofs for Left Arm: "<<std::endl;
  for( int i = 0; i < mNumArmDofs; ++i ) {
    std::cout<< mLeftArmDofIds[i] << " ";
  }
  std::cout<<std::endl;

  // Right Arm
  std::cout<<"Dofs for Right Arm: "<<std::endl;
  for( int i = 0; i < mNumArmDofs; ++i ) {
    std::cout<< mRightArmDofIds[i] << " ";
  }
  std::cout<<std::endl;

  // Left Leg
  std::cout<<"Dofs for Left Leg: "<<std::endl;
  for( int i = 0; i < mNumLegDofs; ++i ) {
    std::cout<< mLeftLegDofIds[i] << " ";
  }
  std::cout<<std::endl;

  // Right Leg
  std::cout<<"Dofs for Right Leg: "<<std::endl;
  for( int i = 0; i < mNumLegDofs; ++i ) {
    std::cout<< mRightLegDofIds[i] << " ";
  }
  std::cout<<std::endl;
}

/**
 * @function DH2HG
 * @brief Convert DH parameters (standard convention) to Homogenuous transformation matrix.
 */
void motion_rt::DH2HG(Eigen::Isometry3d &B, double t, double f, 
		      double r, double d) {
  
  B = Eigen::Matrix4d::Identity();
  
  B.translate(Eigen::Vector3d(0.,0.,d));
  B.rotate(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ()));
  B.translate(Eigen::Vector3d(r,0,0));
  B.rotate(Eigen::AngleAxisd(f, Eigen::Vector3d::UnitX()));  
}

/**
 * @function huboArmFK
 * @brief
 */
void motion_rt::huboArmFK( Eigen::Isometry3d &B, 
			   Eigen::VectorXd &q, 
			   int side) {
    Eigen::Isometry3d hand;
    
    hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
    hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
    hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
    hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
    
    huboArmFK(B, q, side, hand);
}

/**
 * @function huboArmFK
 * @brief
 */
void motion_rt::huboArmFK(Eigen::Isometry3d &B, 
			  Eigen::VectorXd &q, 
			  int side,  
			  const Eigen::Isometry3d &endEffector) {
  // Declarations
  Eigen::Isometry3d neck, hand, T;
  Eigen::MatrixXd limits(6,2);
  Eigen::VectorXd offset; offset.resize(6); offset.setZero();
  
  // Parameters
  double l1 = 214.5/1000.0;
  double l2 = 179.14/1000.0;
  double l3 = 181.59/1000.0;
  double l4 = 4.75*25.4/1000.0;

  // Denavit-Hartenberg parameters    
  Eigen::Matrix<double, 6, 1> t, f, r, d;
  t <<  M_PI/2, -M_PI/2,  M_PI/2,       0,       0,  M_PI/2;
  f <<  M_PI/2,  M_PI/2, -M_PI/2,  M_PI/2, -M_PI/2,       0;
  r <<       0,       0,       0,       0,       0,      l4;
  d <<       0,       0,     -l2,       0,     -l3,       0;
  
  if (side == RIGHT) {
    // Transformation from Neck frame to right shoulder pitch frame
    neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
    neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) = -l1;
    neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
    neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
    
    limits <<
      -2,   2,
      -2,  .3,
      -2,   2,
      -2,   0.01,
      -2,   2,
      -1.4, 1.2;
    
    // Set offsets
    offset(1) = limits(1,1); // Note: I think this might be backwards
    //        offset(1) = -limits(1,1);
    
  } else {
    // Transformation from Neck frame to left shoulder pitch frame
    neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
    neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
    neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
    neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
    limits <<
      -2,   2,
      -.3,   2,
      -2,   2,
      -2,   0.01,
      -2,   2,
      -1.4, 1.2;
    
    // Set offsets
    offset(1) = limits(1,0); // Note: I think this might be backwards
    //        offset(1) = -limits(1,0);
  }
  
  // Calculate forward kinematics
  B = neck;
  for (int i = 0; i < 6; i++) {
    DH2HG(T, t(i)+q(i), f(i), r(i), d(i));
    //DH2HG(T, t(i)+q(i)-offset(i), f(i), r(i), d(i));
    B = B*T;
  }
  B = B*endEffector;
  
}


void motion_rt::huboArmIK(Eigen::VectorXd &q, 
			  Eigen::Isometry3d B, 
			  Eigen::VectorXd qPrev, int side) {
    // Hand	
    Eigen::Isometry3d hand;
    hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
    hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
    hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
    hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
    
    huboArmIK(q, B, qPrev, side, hand);
}
  
void motion_rt::huboArmIK(Eigen::VectorXd &q, 
			  const Eigen::Isometry3d B, 
			  Eigen::VectorXd qPrev, 
			  int side, const Eigen::Isometry3d &endEffector)
{
    Eigen::ArrayXXd qAll(6,8);
    
    // Declarations
    Eigen::Isometry3d neck, neckInv, endEffectorInv, BInv;
    Eigen::MatrixXd limits(6,2);
    Eigen::VectorXd offset; offset.resize(6); offset.setZero();
    double nx, sx, ax, px;
    double ny, sy, ay, py;
    double nz, sz, az, pz;
    double q1, q2, q3, q4, q5, q6;
    double qP1, qP3;
    double qT;
    Eigen::Matrix<int, 8, 3> m;
    
    double S2, S4, S5, S6;
    double C2, C4, C5, C6;
    
    // Parameters
    double l1 = 214.5/1000.0;
    double l2 = 179.14/1000.0;
    double l3 = 181.59/1000.0;
    double l4 = 4.75*25.4/1000.0;
    
    if (side == RIGHT) {
        // Transformation from Neck frame to right shoulder pitch frame
        neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
        neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) = -l1;
        neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
        neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
        
        limits <<
        -2,   2,
        -2,  .3,
        -2,   2,
        -2,   0.01,
        -2,   2,
        -1.4, 1.2;
        
        // Set offsets
        offset(1) = limits(1,1); 
        
    } else {
        // Transformation from Neck frame to left shoulder pitch frame
        neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
        neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
        neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
        neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
        
        limits <<
        -2,   2,
        -.3,   2,
        -2,   2,
        -2,   0.01,
        -2,   2,
        -1.4, 1.2;
        
        // Set offsets
        offset(1) = limits(1,0); 
    }
    neckInv = neck.inverse();
    
    endEffectorInv = endEffector.inverse();        

    double zeroSize = .000001;
    
    // Variables
    BInv = (neckInv*B*endEffectorInv).inverse();
    
    nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
    ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
    nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);

    
    qP1 = qPrev(0); qP3 = qPrev(2);
    
    m <<
    1,  1,  1,
    1,  1, -1,
    1, -1,  1,
    1, -1, -1,
    -1,  1,  1,
    -1,  1, -1,
    -1, -1,  1,
    -1, -1, -1;
    
    // Calculate inverse kinematics
    for (int i = 0; i < 8; i++) {
        
        // Solve for q4
        C4 = max(min((2*l4*px - l2*l2 - l3*l3 + l4*l4 + px*px + py*py + pz*pz)/(2*l2*l3),1),-1);
        if (fabs(C4 - 1) < zeroSize) { // Case 1: q4 == 0
            
            // Set q4
            q4 = 0;
            
            // Set q3
            q3 = qP3;
            
            // Solve for q6
            S6 = max(min( py/(l2 + l3), 1),-1);
            C6 = max(min( -(l4 + px)/(l2 + l3), 1), -1);
            q6 = atan2(S6,C6);
            

            // Solve for q2
            S2 = max(min( C4*C6*ax - C4*S6*ay, 1),-1);
            if (fabs(S2 - 1) < zeroSize) {
	      q2 = M_PI/2;
            } else if (fabs(S2 + 1) < zeroSize) {
	      q2 = -M_PI/2;
            } else {
	      std::complex<double> radical(1-S2*S2, 0);
	      q2 = atan2(S2,m(i,2)*std::real(std::sqrt(radical)));
            }
            
            // Solve for q5
            qT = atan2(-C6*ay - S6*ax,az);
            C2 = cos(q2);
            
            if (fabs(C2) < zeroSize) { // Case 3: q2 = pi/2 or -pi/2
                
                q1 = qP1;
                q3 = qP3;
                
                // Solve for q5
                if (S2 > 0) { // Case 3a: q2 = pi/2
                    qT = atan2(nz,-sz);
                    q5 = wrapToPi(q1 - q3 - qT);
                } else { // Case 3b: q2 = -pi/2
                    qT = atan2(-nz,sz);
                    q5 = wrapToPi(qT - q1 - q3);
                }
                
                
            } else {
                
                if (C2 < 0) {
                    qT = qT + M_PI;
                }
                q5 = wrapToPi(qT - q3);
                
                // Solve for q1
                q1 = atan2(S6*ny - C6*nx,C6*sx - S6*sy);
                if (C2 < 0) {
                    q1 = q1 + M_PI;
                }
                q1 = wrapToPi(q1);
            }
            
        } else {
            
            // Solve for q4
	    std::complex<double> radical(1-C4*C4,0.0);
            q4 = atan2(m(i,0)*std::real( std::sqrt(radical) ),C4);
            
            // Solve for q5
            S4 = sin(q4);
            S5 = pz/(S4*l2);
            if (fabs(S5 - 1) < zeroSize) {
                q5 = M_PI/2;
            } else if (fabs(S5 + 1) < zeroSize) {
                q5 = -M_PI/2;
            } else {
                radical = 1-S5*S5;
                q5 = atan2(S5,m(i,1)*std::real(std::sqrt(radical)));
            }
            
            // Solve for q6
            C5 = cos(q5);
            S6 =max(min( (C5*S4*l2 + (py*(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px)))/(l4 + px + py*py/(l4 + px)))/(l4 + px), 1),-1);
            C6 = max(min( -(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px))/(l4 + px + py*py/(l4 + px)), 1),-1);
            q6 = atan2(S6,C6);
            
            // Solve for q2
            S2 = max(min(ax*(C4*C6 - C5*S4*S6) - ay*(C4*S6 + C5*C6*S4) - S4*S5*az,1),-1);
            if (fabs(S2 - 1) < zeroSize) {
                q2 = M_PI/2;
            } else if (fabs(S2 + 1) < zeroSize) {
                q2 = -M_PI/2;
            } else {
                radical = 1-S2*S2;
                q2 = atan2(S2,m(i,2)*std::real(std::sqrt(radical)));
            }
            
            // Solve for q3
            C2 = cos(q2);
            
            if (fabs(C2) < zeroSize) { // Case 2: q2 = pi/2 or -pi/2
                
                q3 = qP3;
                // Solve for q1
                if (S2 > 0) { // Case 2a: q2 = pi/2
                    qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                    if (S4 < 0) {
                        qT = qT + M_PI;
                    }
                    q1 = wrapToPi(qT + q3);
                } else { // Case 2b: q2 = -pi/2
                    qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                    if (S4 < 0) {
                        qT = qT + M_PI;
                    }
                    q1 = wrapToPi(qT - q3);
                }
                
            } else {
                q3 = atan2(S4*S6*ay - C4*S5*az - C6*S4*ax - C4*C5*C6*ay - C4*C5*S6*ax,C5*az - C6*S5*ay - S5*S6*ax);
                if (C2 < 0) {
                    q3 = q3 - M_PI;
                }
                q3 = wrapToPi(q3);
                
                // Solve for q1
                q1 = atan2(C4*S6*ny - C4*C6*nx + S4*S5*nz + C5*C6*S4*ny + C5*S4*S6*nx,C4*C6*sx - C4*S6*sy - S4*S5*sz - C5*C6*S4*sy - C5*S4*S6*sx);
                if (C2 < 0) {
                    q1 = q1 + M_PI;
                }
                q1 = wrapToPi(q1);
            }
        }
        
        qAll(0,i) = q1;
        qAll(1,i) = q2;
        qAll(2,i) = q3;
        qAll(3,i) = q4;
        qAll(4,i) = q5;
        qAll(5,i) = q6;

    }
    // Set to offsets
    for( int j=0; j<8; j++) {
        for (int i = 0; i < 6; i++) {
            if (side==RIGHT) {
                qAll(i,j) = wrapToPi(qAll(i,j) + offset(i));
            } else {
                qAll(i,j) = wrapToPi(qAll(i,j) + offset(i));
            } 
        }
    }
    // TODO: Find best solution using better method

    Eigen::ArrayXd qDiff(6,1); qDiff.setZero();
    Eigen::ArrayXd qDiffSum(8,1);
    bool withinLim[8];
    int minInd;

    // if any joint solution is infintesimal, set it to zero
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if(qAll(j,i) < zeroSize && qAll(j,i) > -zeroSize)
                qAll(j,i) = 0.0;

    // Initialize withinLim to all trues for all eight solutions
    for(int i=0; i<8; i++)
        withinLim[i] = true;

    // Check each set of solutions to see if any are outside the limits
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if( limits(j,0) > qAll(j,i) || qAll(j,i) > limits(j,1) )
                withinLim[i] = false;
    
    // Initialze anyWithin boolean array to all trues
    bool anyWithin=false;
    for(int i=0; i<8; i++)
        if( withinLim[i] )
            anyWithin = true;

    // If any solution has all joints within the limits...
    if(anyWithin)
    {
        // for each solution...
        for (int i = 0; i < 8; i++) {
            // if all the joints of solution i are within the limits...
            if( withinLim[i] )
            {
                // calculate the differences between solution angles, j, and previous angles
                for (int j=0; j < 6; j++)
                    qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
                // sum the absolute values of the differences to get total difference
                qDiffSum(i) = qDiff.abs().sum();
            }
            // if the solution doesn't have all the joints within the limits...
            else
                // set the difference for that solution to infinity
                qDiffSum(i) = std::numeric_limits<double>::infinity();
        }
        // and take the solution closest to previous solution
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // if no solution has all the joints within the limits...
    else
    {
        // then for each solution...
        for( int i=0; i<8; i++)
        {
            // create a 6d vector of angles of solution i
            Eigen::VectorXd qtemp = qAll.col(i).matrix();
            // take the min of the angles and the joint upper limits
            qtemp = qtemp.cwiseMin(limits.col(1));
            // then take the max of those angles and the joint lower limits
            qtemp = qtemp.cwiseMax(limits.col(0));
            // create an Isometry3d 4x4 matrix for the temp pose
            Eigen::Isometry3d Btemp;
            // find the pose associated with the temp angles
            huboArmFK( Btemp, qtemp, side );
            // calculate the distance from previous pose to temp pose locations
            qDiffSum(i) = (Btemp.translation() - B.translation()).norm();
        }
        // find the solution that's closest the previous position
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // set the final joint angles to the solution closest to the previous solution
    for( int i=0; i<6; i++ )
        q(i) = max( min( q(i), limits(i,1)), limits(i,0) );
}


/**
 * @function huboLegFK
 */
void motion_rt::huboLegFK( Eigen::Isometry3d &B, 
			   Eigen::VectorXd &q, int side) {
    // Declarations
  Eigen::Isometry3d neck, waist, T;
  Eigen::MatrixXd limits(6,2);
  Eigen::VectorXd offset; offset.resize(6); offset.setZero();
  
  // Parameters
  double l1 = (79.5+107)/1000.0;
  double l2 = 88.43/1000.0;
  double l3 = (289.47-107)/1000.0;
  double l4 = 300.03/1000.0;
  double l5 = 300.38/1000.0;
  double l6 = 94.97/1000.0;

  // Denavit-Hartenberg parameters 
  Eigen::Matrix<double,6,1> t, f, r, d;
  t <<       0, -M_PI/2,       0,       0,       0,       0;
  f <<  M_PI/2, -M_PI/2,       0,       0,  M_PI/2,       0;
  r <<       0,       0,      l4,      l5,       0,      l6;
  d <<       0,       0,       0,       0,       0,       0;
  
  // Transformation from Neck frame to Waist frame
  neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
  neck(1,0) = 0; neck(1,1) =  1; neck(1,2) = 0; neck(1,3) =   0;
  neck(2,0) = 0; neck(2,1) =  0; neck(2,2) = 1; neck(2,3) = -l1;
  neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
  
  if (side == RIGHT) {
    // Transformation from Waist frame to right hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) = -l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
    
    limits <<
      -1.80,   0.0,
      -0.58,   0.0,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.23,   0.31;
    
  } else {
    // Transformation from Waist frame to left hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) =  l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
    
    limits <<
      0.0,     1.80,
      0.0,     0.58,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.31,   0.23;
    
  }
  
  // Calculate forward kinematics
  B = waist*neck;
  for (int i = 0; i < 6; i++) {
    DH2HG(T, t(i)+q(i), f(i), r(i), d(i));
    //    DH2HG(T, t(i)+q(i)+offset(i), f(i), r(i), d(i));
    B = B*T;
  }
}

/**
 * @function huboLegIK
 */
void motion_rt::huboLegIK( Eigen::VectorXd &q, 
			   const Eigen::Isometry3d B, 
			   Eigen::VectorXd qPrev, int side) {
  Eigen::ArrayXXd qAll(6,8);
  
  // Declarations
  Eigen::Isometry3d neck, neckInv, waist, waistInv, BInv;
  Eigen::MatrixXd limits(6,2);
  Eigen::VectorXd offset; offset.resize(6);offset.setZero();
  double nx, sx, ax, px;
  double ny, sy, ay, py;
  double nz, sz, az, pz;
  double q1, q2, q3, q4, q5, q6;
  double C45, psi, q345;
  Eigen::Matrix<int, 8, 3> m;
  
  double S2, S4, S6;
  double C2, C4, C5, C6;
  
  // Parameters
  double l1 = (79.5+107)/1000.0;
  double l2 = 88.43/1000.0;
  double l3 = (289.47-107)/1000.0;
  double l4 = 300.03/1000.0;
  double l5 = 300.38/1000.0;
  double l6 = 94.97/1000.0;
  
  // Transformation from Neck frame to Waist frame
  neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
  neck(1,0) = 0; neck(1,1) =  1; neck(1,2) = 0; neck(1,3) =   0;
  neck(2,0) = 0; neck(2,1) =  0; neck(2,2) = 1; neck(2,3) = -l1;
  neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
  
  if (side == RIGHT) {
    // Transformation from Waist frame to right hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) = -l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
    
    limits <<
      -1.80,   0.0,
      -0.58,   0.0,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.23,   0.31;
    
  } else {
    // Transformation from Waist frame to left hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) =  l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
    
    limits <<
      0.0,     1.80,
      0.0,     0.58,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.31,   0.23;
    
  }
  neckInv = neck.inverse();
  waistInv = waist.inverse();
  
  // Variables
  BInv = (neckInv*waistInv*B).inverse();
  
  nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
  ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
  nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);
  
  m <<
    1,  1,  1,
    1,  1, -1,
    1, -1,  1,
    1, -1, -1,
    -1,  1,  1,
    -1,  1, -1,
    -1, -1,  1,
    -1, -1, -1;
    
  for (int i = 0; i < 8; i++)
    {
      C4 = ((l6 + px)*(l6 + px) - l4*l4 - l5*l5 + py*py + pz*pz)/(2*l4*l5);
      std::complex<double> radical(1-C4*C4, 0 );
      q4 = atan2(m(i,0)*std::real(std::sqrt(radical)),C4);
        
        S4 = sin(q4);
        psi = atan2(S4*l4, C4*l4+l5);
        radical = ((px+l6)*(px+l6)+(py*py));
        q5 = wrapToPi(atan2(-pz, m(i,1)*std::real(std::sqrt(radical)))-psi);
        
        q6 = atan2(py, -px-l6);
        C45 = cos(q4+q5);
        C5 = cos(q5);
        if (C45*l4 + C5*l5 < 0)
        {
            q6 = wrapToPi(q6 + M_PI);
        }
        
        S6 = sin(q6);
        C6 = cos(q6);
        
        S2 = C6*ay + S6*ax;
        radical = 1-S2*S2;
        q2 = atan2(S2,m(i,2)*std::real(std::sqrt(radical)));
        
        q1 = atan2(C6*sy + S6*sx,C6*ny + S6*nx);
        C2 = cos(q2);
        if (C2 < 0) {
            q1 = wrapToPi(q1 + M_PI);
        }
        
        q345 = atan2(-az/C2,-(C6*ax - S6*ay)/C2);
        q3 = wrapToPi(q345-q4-q5);
        
        qAll(0,i) = q1;
        qAll(1,i) = q2;
        qAll(2,i) = q3;
        qAll(3,i) = q4;
        qAll(4,i) = q5;
        qAll(5,i) = q6;
    }
    
    // Set to offsets
    for (int i = 0; i < 6; i++) {
        if (side==RIGHT) {
            q(i) = wrapToPi(q(i) + offset(i));
        } else {
            q(i) = wrapToPi(q(i) + offset(i));
        }
    }
    
    // Find best solution // TODO: Need to find a better way of choosing the best solution
    Eigen::ArrayXd qDiff(6,1); qDiff.setZero();
    Eigen::ArrayXd qDiffSum(8,1);
    bool withinLim[8];
    int minInd;
    double zeroSize = 0.000001;

    // if any joint solution is infintesimal, set it to zero
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if(qAll(j,i) < zeroSize && qAll(j,i) > -zeroSize)
                qAll(j,i) = 0.0;

    // Initialize withinLim to all trues for all eight solutions
    for(int i=0; i<8; i++)
        withinLim[i] = true;

    // Check each set of solutions to see if any are outside the limits
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if( limits(j,0) > qAll(j,i) || qAll(j,i) > limits(j,1) )
                withinLim[i] = false;

    // Initialze anyWithin boolean array to all trues
    bool anyWithin=false;
    for(int i=0; i<8; i++)
        if( withinLim[i] )
            anyWithin = true;

    // If any solution has all joints within the limits...
    if(anyWithin)
    {
        // for each solution...
        for (int i = 0; i < 8; i++)
        {
            // if all the joints of solution i are within the limits...
            if( withinLim[i] )
            {
                // calculate the differences between solution angles, j, and previous angles
                for (int j=0; j < 6; j++)
                {
                    qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
                }
                // sum the absolute values of the differences to get total difference
                qDiffSum(i) = qDiff.abs().sum();
            }
            // if the solution doesn't have all the joints within the limits...
            else
                // set the difference for that solution to infinity
                qDiffSum(i) = std::numeric_limits<double>::infinity();
        }
        // and take the solution closest to previous solution
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }

    // if no solution has all the joints within the limits...
    else
    {
        // then for each solution...
        for(int i=0; i<8; i++)
        {
            // create a 6d vector of angles of solution i
	  Eigen::VectorXd qtemp; qtemp.resize(6); qtemp = qAll.col(i).matrix();
            // take the min of the angles and the joint upper limits
            qtemp = qtemp.cwiseMin(limits.col(1));
            // then take the max of those angles and the joint lower limits
            qtemp = qtemp.cwiseMax(limits.col(0));
            // create an Isometry3d 4x4 matrix for the temp pose
            Eigen::Isometry3d Btemp;
            // find the pose associated with the temp angles
            huboLegFK( Btemp, qtemp, side );
            // calculate the distance from previous pose to temp pose locations
            qDiffSum(i) = (Btemp.translation() - B.translation()).norm();
        }
        // find the solution that's closest the previous position
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // set the final joint angles to the solution closest to the previous solution
    for( int i=0; i<6; i++)
        q(i) = max( min( q(i), limits(i,1)), limits(i,0) ); 
}
/*
void motion_rt::HuboDrillFK(Eigen::Isometry3d &B, Eigen::VectorXd &q) {
    Eigen::Isometry3d drill;
    
    double ld = 7*25.4/1000.0;
    double ad = M_PI/4;
    
    drill(0,0) = cos(ad);  drill(0,1) = 0; drill(0,2) = sin(ad); drill(0,3) = ld*cos(ad);
    drill(1,0) = 0;        drill(1,1) = 1; drill(1,2) = 0;       drill(1,3) = 0;
    drill(2,0) = -sin(ad); drill(2,1) = 0; drill(2,2) = cos(ad); drill(2,3) = -ld*sin(ad);
    drill(3,0) = 0;        drill(3,1) = 0; drill(3,2) = 0;       drill(3,3) = 1;
    
    huboArmFK(B, q, RIGHT, drill);
}


void motion_rt::HuboDrillIK(Eigen::VectorXd &q, double y) {
  Eigen::VectorXd qPrev; qPrev.resize(6); qPrev.setZero();
    Eigen::Isometry3d drill, B;
    
    double l1 = 214.5/1000;
    double l2 = 179.14/1000;
    double l3 = 181.59/1000;
    double l4 = 4.75*25.4/1000;
    double ld = 7*25.4/1000.0;
    double ad = M_PI/4;
    
    drill(0,0) = cos(ad);  drill(0,1) = 0; drill(0,2) = sin(ad); drill(0,3) = ld*cos(ad);
    drill(1,0) = 0;        drill(1,1) = 1; drill(1,2) = 0;       drill(1,3) = 0;
    drill(2,0) = -sin(ad); drill(2,1) = 0; drill(2,2) = cos(ad); drill(2,3) = -ld*sin(ad);
    drill(3,0) = 0;        drill(3,1) = 0; drill(3,2) = 0;       drill(3,3) = 1;
    
    double pyLimits[2] = {-0.2050, 0.1850};
    
    double px = 0.0925;
    double py = min(max(y,pyLimits[0]),pyLimits[1]);
    double pz = 0.04;
    
    double px0 = (l3+l4)*sin(M_PI/4)+ld;
    double py0 = -l1;
    double pz0 = (l3+l4)*cos(M_PI/4)-l2;
    
    B(0,0) = 1; B(0,1) = 0; B(0,2) = 0; B(0,3) = px0+px;
    B(1,0) = 0; B(1,1) = 1; B(1,2) = 0; B(1,3) = py0+py;
    B(2,0) = 0; B(2,1) = 0; B(2,2) = 1; B(2,3) = pz0+pz;
    B(3,0) = 0; B(3,1) = 0; B(3,2) = 0; B(3,3) = 1;
    
    huboArmIK(q, B, qPrev, RIGHT, drill);
    
}

*/

/**
 * @function huboLegIK2
 */
void motion_rt::huboLegIK2( Eigen::VectorXd &q, 
			    const Eigen::Isometry3d B, 
			    Eigen::VectorXd qPrev, int side) {
  Eigen::ArrayXXd qAll(6,8);
  
  // Declarations
  Eigen::Isometry3d neck, neckInv, waist, waistInv, BInv;
  Eigen::MatrixXd limits(6,2);
  Eigen::VectorXd offset; offset.resize(6);offset.setZero();
  double nx, sx, ax, px;
  double ny, sy, ay, py;
  double nz, sz, az, pz;
  double q1, q2, q3, q4, q5, q6;
  double C45, psi, q345;
  Eigen::Matrix<int, 8, 3> m;
  
  double S2, S4, S6;
  double C2, C4, C5, C6;
  
  // Parameters
  double l1 = mLegLengths(0);
  double l2 = mLegLengths(1);
  double l3 = mLegLengths(2);
  double l4 = mLegLengths(3);
  double l5 = mLegLengths(4);
  double l6 = mLegLengths(5);
  
  // Transformation from Neck frame to Waist frame
  neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
  neck(1,0) = 0; neck(1,1) =  1; neck(1,2) = 0; neck(1,3) =   0;
  neck(2,0) = 0; neck(2,1) =  0; neck(2,2) = 1; neck(2,3) = -l1;
  neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
  
  if (side == RIGHT) {
    // Transformation from Waist frame to right hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) = -l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
    
    limits <<
      -1.80,   0.0,
      -0.58,   0.0,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.23,   0.31;
    
  } else {
    // Transformation from Waist frame to left hip yaw frame
    waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
    waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) =  l2;
    waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
    waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
    
    limits <<
      0.0,     1.80,
      0.0,     0.58,
      -1.30,   1.30,
      0.0,     2.50,
      -1.26,   1.80,
      -0.31,   0.23;
    
  }
  neckInv = neck.inverse();
  waistInv = waist.inverse();
  
  // Variables
  BInv = (neckInv*waistInv*B).inverse();
  
  nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
  ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
  nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);
  
  m <<
    1,  1,  1,
    1,  1, -1,
    1, -1,  1,
    1, -1, -1,
    -1,  1,  1,
    -1,  1, -1,
    -1, -1,  1,
    -1, -1, -1;
    
  for (int i = 0; i < 8; i++)
    {
      C4 = ((l6 + px)*(l6 + px) - l4*l4 - l5*l5 + py*py + pz*pz)/(2*l4*l5);
      std::complex<double> radical(1-C4*C4, 0 );
      q4 = atan2(m(i,0)*std::real(std::sqrt(radical)),C4);
        
        S4 = sin(q4);
        psi = atan2(S4*l4, C4*l4+l5);
        radical = ((px+l6)*(px+l6)+(py*py));
        q5 = wrapToPi(atan2(-pz, m(i,1)*std::real(std::sqrt(radical)))-psi);
        
        q6 = atan2(py, -px-l6);
        C45 = cos(q4+q5);
        C5 = cos(q5);
        if (C45*l4 + C5*l5 < 0)
        {
            q6 = wrapToPi(q6 + M_PI);
        }
        
        S6 = sin(q6);
        C6 = cos(q6);
        
        S2 = C6*ay + S6*ax;
        radical = 1-S2*S2;
        q2 = atan2(S2,m(i,2)*std::real(std::sqrt(radical)));
        
        q1 = atan2(C6*sy + S6*sx,C6*ny + S6*nx);
        C2 = cos(q2);
        if (C2 < 0) {
            q1 = wrapToPi(q1 + M_PI);
        }
        
        q345 = atan2(-az/C2,-(C6*ax - S6*ay)/C2);
        q3 = wrapToPi(q345-q4-q5);
        
        qAll(0,i) = q1;
        qAll(1,i) = q2;
        qAll(2,i) = q3;
        qAll(3,i) = q4;
        qAll(4,i) = q5;
        qAll(5,i) = q6;
    }
    
    // Set to offsets
    for (int i = 0; i < 6; i++) {
        if (side==RIGHT) {
            q(i) = wrapToPi(q(i) + offset(i));
        } else {
            q(i) = wrapToPi(q(i) + offset(i));
        }
    }
    
    // Find best solution // TODO: Need to find a better way of choosing the best solution
    Eigen::ArrayXd qDiff(6,1); qDiff.setZero();
    Eigen::ArrayXd qDiffSum(8,1);
    bool withinLim[8];
    int minInd;
    double zeroSize = 0.000001;

    // if any joint solution is infintesimal, set it to zero
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if(qAll(j,i) < zeroSize && qAll(j,i) > -zeroSize)
                qAll(j,i) = 0.0;

    // Initialize withinLim to all trues for all eight solutions
    for(int i=0; i<8; i++)
        withinLim[i] = true;

    // Check each set of solutions to see if any are outside the limits
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if( limits(j,0) > qAll(j,i) || qAll(j,i) > limits(j,1) )
                withinLim[i] = false;

    // Initialze anyWithin boolean array to all trues
    bool anyWithin=false;
    for(int i=0; i<8; i++)
        if( withinLim[i] )
            anyWithin = true;

    // If any solution has all joints within the limits...
    if(anyWithin)
    {
        // for each solution...
        for (int i = 0; i < 8; i++)
        {
            // if all the joints of solution i are within the limits...
            if( withinLim[i] )
            {
                // calculate the differences between solution angles, j, and previous angles
                for (int j=0; j < 6; j++)
                {
                    qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
                }
                // sum the absolute values of the differences to get total difference
                qDiffSum(i) = qDiff.abs().sum();
            }
            // if the solution doesn't have all the joints within the limits...
            else
                // set the difference for that solution to infinity
                qDiffSum(i) = std::numeric_limits<double>::infinity();
        }
        // and take the solution closest to previous solution
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }

    // if no solution has all the joints within the limits...
    else
    {
        // then for each solution...
        for(int i=0; i<8; i++)
        {
            // create a 6d vector of angles of solution i
	  Eigen::VectorXd qtemp; qtemp.resize(6); qtemp = qAll.col(i).matrix();
            // take the min of the angles and the joint upper limits
            qtemp = qtemp.cwiseMin(limits.col(1));
            // then take the max of those angles and the joint lower limits
            qtemp = qtemp.cwiseMax(limits.col(0));
            // create an Isometry3d 4x4 matrix for the temp pose
            Eigen::Isometry3d Btemp;
            // find the pose associated with the temp angles
            huboLegFK( Btemp, qtemp, side );
            // calculate the distance from previous pose to temp pose locations
            qDiffSum(i) = (Btemp.translation() - B.translation()).norm();
        }
        // find the solution that's closest the previous position
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // set the final joint angles to the solution closest to the previous solution
    for( int i=0; i<6; i++)
        q(i) = max( min( q(i), limits(i,1)), limits(i,0) ); 
}

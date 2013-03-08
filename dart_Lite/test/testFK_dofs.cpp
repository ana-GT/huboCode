/**
 * @file testFK_dofs.cpp
 * @brief Testing FK accuracy with model
 */

#include "robotics/parser/dart_parser/DartLoader.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/Joint.h"
#include "kinematics/BodyNode.h"
#include <stdio.h>
#include "motion_rt.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  motion_rt mr;

  // Parsing step
  DartLoader dl;
  dynamics::SkeletonDynamics* hubo;
  hubo = dl.parseSkeleton( argv[1] );
  if( !hubo ) {
    printf("Bang! No good skeleton generated! \n");
    return -1;
  }

  
  printf("Generated a nice skeleton for Hubo! \n");
  std::vector<int> dof(6);
  dof[0] = 19; dof[1] = 20;
  dof[2] = 23; dof[3] = 24;
  dof[4] = 27; dof[5] = 28;

  for( int i = 0; i < dof.size(); ++i ) {
    std::string name;
    name = hubo->getDof(dof[i])->getJoint()->getName();
    std::cout<< " Dof [" << dof[i] << "]: "<< name << std::endl;
  }

  mr.setSkel( hubo );
  mr.initialize();
  mr.print_DofInfo();
  mr.testLeftArmFK();
  mr.testRightArmFK(); 
  mr.testLeftLegFK();
  mr.testRightLegFK(); 

  return 0;
}

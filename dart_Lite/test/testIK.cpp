/**
 * @file testFK.cpp
 * @brief Testing FK accuracy with model
 */

#include "robotics/parser/dart_parser/DartLoader.h"
#include "dynamics/SkeletonDynamics.h"
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

  mr.setSkel( hubo );
  mr.initialize();
  mr.print_DofInfo();
  mr.testLeftArmFK();
  mr.testRightArmFK(); 
  mr.testLeftLegFK();
  mr.testRightLegFK(); 

  return 0;
}

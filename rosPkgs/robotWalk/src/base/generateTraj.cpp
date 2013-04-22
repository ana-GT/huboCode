/**
 * @file generateTraj.cpp
 */

#include "utilities.h"
#include <stdio.h>

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  std::vector<double> zmpx, zmpy;
  generateZmpPositions( zmpx, zmpy, 5 );
  printf("Size of zmp vector: %d \n", zmpx.size() );
  print( std::string("zmpxy.txt"), zmpx, zmpy );

}

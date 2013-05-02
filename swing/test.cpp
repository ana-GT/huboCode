/**
* @file test.cpp
* @brief Testing swing behavior
* @author A. Huaman and X. Yan
*/

#include <iostream>
#include <swing.h>

/**
* @function main
*/
int main( int argc, char* argv[] ) {

  double x0, y0, theta0;
  double x1, y1, theta1;

  int count;
  bool isLeft;
  
  double pos[N][3];
  double yaw[N];

  // Set test values
  x0 = 0; y0 = 0; theta0 = 30.0*3.1416/180.0;
  x1 = 0.2; y1 = 0.3; theta1 = 120*3.1416/180.0;
  // x0 = 0; y0 = 0; theta0 = 0;
  // x1 = 0.2; y1 = 0; theta1 = 0;
  count = 300;
  isLeft = false;
  
  swingEllipse( x0, y0, theta0,
		     x1, y1, theta1,
		     count, isLeft,
		     pos, yaw );
  
  // Get velocity and acceleration plots
  // Consider dt = 0.005 (200 Hz)
  double dt = 0.005;
  
  // Vel (pos + yaw)
  double vel[N][4];
  vel[0][0] = ( pos[1][0] - pos[0][0] ) / dt;
  vel[0][1] = ( pos[1][1] - pos[0][1] ) / dt;
  vel[0][2] = ( pos[1][2] - pos[0][2] ) / dt;
  vel[0][3] = ( yaw[1] - yaw[0] ) / dt;

  for( int i = 1; i < count; ++i ) {
    vel[i][0] = ( pos[i][0] - pos[i-1][0] ) / dt;
    vel[i][1] = ( pos[i][1] - pos[i-1][1] ) / dt;
    vel[i][2] = ( pos[i][2] - pos[i-1][2] ) / dt;
    vel[i][3] = ( yaw[i] - yaw[i-1] ) / dt;
  }

  // Accel
  double acc[N][4];
  acc[0][0] = (vel[1][0] - vel[0][0] ) / dt;
  acc[0][1] = (vel[1][1] - vel[0][1] ) / dt;
  acc[0][2] = (vel[1][2] - vel[0][2] ) / dt;
  acc[0][3] = (vel[1][3] - vel[0][3] ) / dt;

  for( int i = 1; i < count - 1; ++i ) {
    acc[i][0] = ( vel[i+1][0] - vel[i-1][0] ) / dt;
    acc[i][1] = ( vel[i+1][1] - vel[i-1][1] ) / dt;
    acc[i][2] = ( vel[i+1][2] - vel[i-1][2] ) / dt;
    acc[i][3] = ( vel[i+1][3] - vel[i-1][3] ) / dt;
  }

  acc[count-1][0] = (vel[count-1][0] - vel[count-2][0] ) / dt;
  acc[count-1][1] = (vel[count-1][1] - vel[count-2][1] ) / dt;
  acc[count-1][2] = (vel[count-1][2] - vel[count-2][2] ) / dt;
  acc[count-1][3] = (vel[count-1][3] - vel[count-2][3] ) / dt;


  FILE *pFile; FILE *vFile; FILE *aFile;
  pFile = fopen( "pos.txt", "w");
  vFile = fopen( "vel.txt", "w" );
  aFile = fopen( "acc.txt", "w" );
  double t = 0;
  for( int i = 0; i < count; ++i ) {
    fprintf( pFile, "%d %f %f %f %f \n", i, pos[i][0], pos[i][1], pos[i][2], yaw[i] );
    fprintf( vFile, "%f %f %f %f %f \n", t, vel[i][0], vel[i][1], vel[i][2], vel[i][3] );
    fprintf( aFile, "%f %f %f %f %f \n", t, acc[i][0], acc[i][1], acc[i][2], acc[i][3] );
    t += dt;
  }
  fclose( pFile );
  fclose( vFile );
  fclose( aFile );

  std::cout << "End of program" << std::endl;

  return 0;
}




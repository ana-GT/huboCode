/**
 * @file walk.cpp
 */
#include <math.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


#include <stdio.h>

#include "walk.h"

/**
 * @function main
 */
int main( int argc, char** argv ) {

  ros::init( argc, argv, "walk" );

  walk gW;

  // Wait until sim is active (play)
  ros::Time last_ros_time;
  bool wait = true;
  
  while( wait ) {
    last_ros_time = ros::Time::now();
    if( last_ros_time.toSec() > 0 ) {
      wait = false;
    }       
  }

  // Set communications
  gW.setCommunications();

  
  // Set joint command configuration
  gW.setJointConfig();
  
  // Set into initial position
  gW.setInitPosition( );

  bool run = true;
  ros::Rate t(10);
  while( run = true ) {

    if( gW.mStatus == 1 ) {
      // Set ZMP configuration values
      gW.setZMPConfig();   
      printf( " Size of whole body: %d \n", gW.mZmp.mWholeBody.size() );
      gW.mStatus = 2;
      //   run = false;
    }

    if( gW.mStatus == 3 ) {

    }

    ros::spinOnce();
    //t.sleep();
  
  }

  // ros::spin();

  return 0;
}

/**
 * @function walk
 * @brief Constructor
 */
walk::walk() {

  mRosNode = new ros::NodeHandle();

  // Init pose
  double vals[28] = { -3.852963655681663e-06,
		      0.0009848090520510056,
		      0.00010776096065789886,
		      0.7806726847358707,

		      -0.0012852229728474995,
		      0.061783845913243596,
		      -0.2856717835152658,
		      0.5266262672930972,
		      -0.21864856475431704,
		      -0.062394234133471116,

		      0.0013642411907399676,
		      -0.06195623610921519,
		      -0.2865128374461472,
		      0.5268958272322948,
		      -0.21621680953724365,
		      0.06138342176132294,				   

		      0.2988754829726865,
		      -1.3138418263023999,
		      2.00219166466513,
		      0.4950932275951452,
		      -8.449255747589035e-05,
		      -0.010542899622185686,				   

		      0.298867201336498138,
		      1.313736629564075,
		      2.0021752327042464,
		      -0.49508063541354375,
		      -8.39712346438759e-05,
		      0.01056157776909128 };
  mInitPose.resize(0);
  for( int i = 0; i < 28; ++i ) {
    mInitPose.push_back( vals[i] );
  }

  // Set time to init pose
  mTimeToSetInitPose = 6.0;
}

/**
 * @function walk
 * @brief Destructor
 */
walk::~walk() {
}

/**
 * @function setCommunications
 */
void walk::setCommunications() {

  // ros topic subscribtions
  mSubJointStates = mRosNode->subscribe( "/atlas/joint_states", 1, &walk::setJointStates, this );
  // Advertise joint commands
  mPubJointCommands = mRosNode->advertise<osrf_msgs::JointCommands>( "/atlas/joint_commands", 1, true);
}

/**
 * @function setInitPosition
 */
void walk::setInitPosition( ) {
  printf("Set init position \n");
  mStatus = 0; // WE ARE IN INIT POSITION  
}



/**
 * @function setJointStates
 */ 
void walk::setJointStates( const sensor_msgs::JointState::ConstPtr &_js ) {

  static ros::Time startTime = ros::Time::now();  
  static int counter = 0;

  // Init Pose
  if( mStatus == 0 ) {
    
    // Set it slowly to the first position
    double factor;
    factor =  ( ros::Time::now().toSec() - startTime.toSec() ) / mTimeToSetInitPose;
    
    if( factor < 1.0 ) {
      mJointCommands.header.stamp = _js->header.stamp;
      for( unsigned int i = 0; i < mJointCommands.name.size(); ++i ) {
	double val = mInitPose[i];
	mJointCommands.position[i] = val*factor;
      }
      mPubJointCommands.publish( mJointCommands );    
    }
    else {
      mStatus = 1;
    }
  }

  // Get trajectory
  else if( mStatus == 2 ) {
    if( counter < mZmp.mWholeBody.size() ) {
      for( unsigned int i = 0; i < mJointCommands.name.size(); ++i ) {
	mJointCommands.position[i] = mZmp.mWholeBody[counter](i);
      }
      counter++;
      mPubJointCommands.publish( mJointCommands );    
    }
    else {
      mStatus = 3;
    }
  } 
        
}


/**
 * @function setZMPConfig
 */
void walk::setZMPConfig() {
  printf("Start ZMP Config \n");
  // Variables
  int numSteps = 5;
  double stepLength = 0.1;
  double footSeparation = 0.16;
  double stepDuration = 1.0;
  double slopeTime = 0.15;
  double levelTime = 0.85;
  double dt = 0.001; //0.00033; 
  double zg = 0.8438;
  int numPreviewSteps = 2;
  double Qe = 1;
  double R = 0.000001;

  mZmp.setParameters( dt, 9.81 );
  mZmp.generateZmpPositions( numSteps, false, 
			     stepLength, footSeparation,
			     stepDuration,
			     slopeTime,
			     levelTime );

  mZmp.getControllerGains( Qe, R, zg, numPreviewSteps );
  mZmp.generateCOMPositions();
  mZmp.getJointTrajectories();
  printf("End ZMP Config \n");
}

/**
 * @function setJointConfig
 */
void walk::setJointConfig() {
  
  // Set names (follow this order and use ALL these joints, even if you don't set them)
  mJointCommands.name.push_back("atlas::back_lbz");
  mJointCommands.name.push_back("atlas::back_mby");
  mJointCommands.name.push_back("atlas::back_ubx");

  mJointCommands.name.push_back("atlas::neck_ay");

  mJointCommands.name.push_back("atlas::l_leg_uhz");
  mJointCommands.name.push_back("atlas::l_leg_mhx");
  mJointCommands.name.push_back("atlas::l_leg_lhy");
  mJointCommands.name.push_back("atlas::l_leg_kny");
  mJointCommands.name.push_back("atlas::l_leg_uay");
  mJointCommands.name.push_back("atlas::l_leg_lax");

  mJointCommands.name.push_back("atlas::r_leg_uhz");
  mJointCommands.name.push_back("atlas::r_leg_mhx");
  mJointCommands.name.push_back("atlas::r_leg_lhy");
  mJointCommands.name.push_back("atlas::r_leg_kny");
  mJointCommands.name.push_back("atlas::r_leg_uay");
  mJointCommands.name.push_back("atlas::r_leg_lax");

  mJointCommands.name.push_back("atlas::l_arm_usy"); 
  mJointCommands.name.push_back("atlas::l_arm_shx");
  mJointCommands.name.push_back("atlas::l_arm_ely");
  mJointCommands.name.push_back("atlas::l_arm_elx");
  mJointCommands.name.push_back("atlas::l_arm_uwy");
  mJointCommands.name.push_back("atlas::l_arm_mwx");

  mJointCommands.name.push_back("atlas::r_arm_usy");
  mJointCommands.name.push_back("atlas::r_arm_shx");
  mJointCommands.name.push_back("atlas::r_arm_ely");
  mJointCommands.name.push_back("atlas::r_arm_elx");
  mJointCommands.name.push_back("atlas::r_arm_uwy");
  mJointCommands.name.push_back("atlas::r_arm_mwx"); 


  unsigned int n = mJointCommands.name.size();

  // Set other parameters
  mJointCommands.position.resize(n);
  mJointCommands.velocity.resize(n);
  mJointCommands.effort.resize(n);
  mJointCommands.kp_position.resize(n);
  mJointCommands.ki_position.resize(n);
  mJointCommands.kd_position.resize(n);
  mJointCommands.kp_velocity.resize(n);
  mJointCommands.i_effort_min.resize(n);
  mJointCommands.i_effort_max.resize(n);

  // Set PID values from node parameters
  for( unsigned int i = 0; i < n; ++i ) {
    std::vector<std::string> pieces;
    boost::split( pieces, mJointCommands.name[i], boost::is_any_of(":") );
    double val; double k = 10;
    mRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/p", val );
    mJointCommands.kp_position[i] = val*k;
    mRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i", val );
    mJointCommands.ki_position[i] = val*k;
    mRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/d", val );
    mJointCommands.kd_position[i] = val*k;
    mRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", val );
    mJointCommands.i_effort_min[i] = -val*k;
    
    mRosNode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", val );
    mJointCommands.i_effort_max[i] = val*k;
    
    mJointCommands.velocity[i] = 0;
    mJointCommands.effort[i] = 0;
    mJointCommands.kp_velocity[i] = 0;
  }

}

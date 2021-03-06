/**
 * @function publish_joint_commands_fromTxt
 * @brief Read hardcoded angles and make Atlas walk
 */

#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;
double dt = 0.1; // 10Hz

/**
 * @function SetJointStates
 */
void SetJointStates( const sensor_msgs::JointState::ConstPtr &_js ) {
	static ros::Time startTime = ros::Time::now();
	static int counter = 0;


	if( ( ros::Time::now() - startTime ).toSec() > dt ) {
		startTime = ros::Time::now();
  		std::cout << "Time: "<< ros::Time::now().toSec() << std::endl;
	}

}

/**
 * @function main
 */
int main( int argc, char** argv ) {
  
  ros::init( argc, argv, "pub_joint_command_test" );
  ros::NodeHandle* rosnode = new ros::NodeHandle();

  ros::Time last_ros_time_;

  // Wait until sim is active (play)
  bool wait = true;

  while( wait ) {
    last_ros_time_ = ros::Time::now();
    if( last_ros_time_.toSec() > 0 ) {
      wait = false;
    }
  }

  // Set name of joints to be commanded 
  // (must match these inside AtlasPlugin)
  jointcommands.name.push_back("atlas::l_arm_usy");
  jointcommands.name.push_back("atlas::l_arm_shx");
  jointcommands.name.push_back("atlas::l_arm_ely");
  jointcommands.name.push_back("atlas::l_arm_elx");
  jointcommands.name.push_back("atlas::l_arm_uwy");
  jointcommands.name.push_back("atlas::l_arm_mwx");

  jointcommands.name.push_back("atlas::r_arm_usy");
  jointcommands.name.push_back("atlas::r_arm_shx");
  jointcommands.name.push_back("atlas::r_arm_ely");
  jointcommands.name.push_back("atlas::r_arm_elx");
  jointcommands.name.push_back("atlas::r_arm_uwy");
  jointcommands.name.push_back("atlas::r_arm_mwx");

  jointcommands.name.push_back("atlas::l_leg_uhz");
  jointcommands.name.push_back("atlas::l_leg_mhx");
  jointcommands.name.push_back("atlas::l_leg_lhy");
  jointcommands.name.push_back("atlas::l_leg_kny");
  jointcommands.name.push_back("atlas::l_leg_uay");
  jointcommands.name.push_back("atlas::l_leg_lax");

  jointcommands.name.push_back("atlas::r_leg_uhz");
  jointcommands.name.push_back("atlas::r_leg_mhx");
  jointcommands.name.push_back("atlas::r_leg_lhy");
  jointcommands.name.push_back("atlas::r_leg_kny");
  jointcommands.name.push_back("atlas::r_leg_uay");
  jointcommands.name.push_back("atlas::r_leg_lax");



  unsigned int n = jointcommands.name.size();
  jointcommands.position.resize(n);
  jointcommands.velocity.resize(n);
  jointcommands.effort.resize(n);
  jointcommands.kp_position.resize(n);
  jointcommands.ki_position.resize(n);
  jointcommands.kd_position.resize(n);
  jointcommands.kp_velocity.resize(n);
  jointcommands.i_effort_min.resize(n);
  jointcommands.i_effort_max.resize(n);

  for( unsigned int i = 0; i < n; ++i ) {
    std::vector<std::string> pieces;
    boost::split( pieces, jointcommands.name[i], boost::is_any_of(":") );
    // pieces = { "atlas", "", "name_oj_joint" }
    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p", jointcommands.kp_position[i] );
    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i", jointcommands.ki_position[i] );
    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d", jointcommands.kd_position[i] );

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", jointcommands.i_effort_min[i] );
    jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", jointcommands.i_effort_max[i] );


    jointcommands.velocity[i] = 0;
    jointcommands.effort[i] = 0;
    jointcommands.kp_velocity[i] = 0;
  }

  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, SetJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);



  ros::spin();

  return 0;
}

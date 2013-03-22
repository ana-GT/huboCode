/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "planningTab.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <iostream>

#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <robotics/Robot.h>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>
#include <planning/Trajectory.h>
#include <planning/Path.h>
#include "Controller.h"

using namespace std;

/////////////////////////////
// zmpdemo
#include "zmp/hubo-zmp.h"
#include "src/HuboPlus.h"
#include <math.h>
#include "mzcommon/MzGlutApp.h"
#include "mzcommon/TimeUtil.h"
#include <getopt.h>

#include "zmp/zmpwalkgenerator.h"
#include "zmp/footprint.h"

// Print stuff
#include <stdio.h>

using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;

size_t seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}
/*TODO remove this
*/
const int stance_foot_table[4] = { 0, 1, 0, 1 };
const int swing_foot_table[4] = { -1, -1, 1, 0 };

const stance_t next_stance_table[4] = {
  SINGLE_LEFT,
  SINGLE_RIGHT,
  DOUBLE_RIGHT,
  DOUBLE_LEFT
};

/**
* @function: validateCOMTraj(Eigen::MatrixXd& comX, Eigen::MatrixXd& comY) 
* @brief: validation of COM output trajectory data
* @return: void
*/
void validateCOMTraj(Eigen::MatrixXd& comX, Eigen::MatrixXd& comY) {
    const double dt = 1.0/TRAJ_FREQ_HZ;
    double comVel, comAcc;
    Eigen::Matrix3d comStateDiffs;
    double comStateMaxes[] = {0.0, 0.0, 0.0};
    const double comStateTol[] = {2.0, 5.0}; // m/s, m/s^2, m/s^3
    for (int n=0; n<(comX.rows()-1); n++) {
      // Calculate COM vel and acc norms from x and y components
      comVel = sqrt((comX(n+1,0)-comX(n,0))*(comX(n+1,0)-comX(n,0)) + (comY(n+1,0)-comY(n,0))*(comY(n+1,0)-comY(n,0)))/dt;
      comAcc = sqrt((comX(n+1,1)-comX(n,1))*(comX(n+1,1)-comX(n,1)) + (comY(n+1,1)-comY(n,1))*(comY(n+1,1)-comY(n,1)))/dt;
      // Update max state values
      if (comVel > comStateMaxes[0]) comStateMaxes[0] = comVel;
      if (comAcc > comStateMaxes[1]) comStateMaxes[1] = comAcc;
      // Check if any are over limit
      if (comVel > comStateTol[0]) {
          std::cerr << "COM velocity sample " << n+1 << "is larger than " << comStateTol[0] << "(" << comVel << ")\n";
      }
      if (comAcc > comStateTol[1]) {
          std::cerr << "COM acceleration of sample " << n+1 << "is larger than " << comStateTol[1] << "(" << comAcc << ")\n";
      }
    }
    std::cerr << "comMaxVel: " << comStateMaxes[0]
              << "\ncomMaxAcc: " << comStateMaxes[1] << std::endl;


}
  
/**
* @function: validateOutputData(TrajVector& traj)
* @brief: validation of joint angle output trajectory data
* @return: void
*/
void validateOutputData(TrajVector& traj) {
    const double dt = 1.0/TRAJ_FREQ_HZ;
    double maxJointVel=0;
    double jointVel;
    const double jointVelTol = 6.0; // radians/s
    for (int n=0; n<(int)(traj.size()-1); n++) {
      for (int j=0; j<HUBO_JOINT_COUNT; j++) {  
        jointVel = (traj[n+1].angles[j] - traj[n].angles[j])/dt;
        if (jointVel > jointVelTol) {
          std::cerr << "change in joint " << j << "is larger than " << jointVelTol << "(" << jointVel << ")\n";
        }
        if (jointVel > maxJointVel) maxJointVel = jointVel;
      }
    }
    std::cerr << "maxJntVel: " << maxJointVel << std::endl;
}



double getdouble(const char* str) {
  char* endptr;
  double d = strtod(str, &endptr);
  if (!endptr || *endptr) {
    std::cerr << "Error parsing number on command line!\n\n";
    exit(1);
  }
  return d;
}

long getlong(const char* str) {
  char* endptr;
  long d = strtol(str, &endptr, 10);
  if (!endptr || *endptr) {
    std::cerr << "Error parsing number on command line!\n\n";
    exit(1);
  }
  return d;
}

enum walktype {
  walk_canned,
  walk_line,
  walk_circle
};

walktype getwalktype(const std::string& s) {
  if (s == "canned") {
    return walk_canned;
  } else if (s == "line") {
    return walk_line;
  } else if (s == "circle") {
    return walk_circle;
  } else {
    std::cerr << "bad walk type " << s << "\n";
    exit(1);
  }
}

ZMPWalkGenerator::ik_error_sensitivity getiksense(const std::string& s) {
  if (s == "strict") {
    return ZMPWalkGenerator::ik_strict;
  } else if (s == "sloppy") {
    return ZMPWalkGenerator::ik_sloppy;
  } else if (s == "permissive") {
    return ZMPWalkGenerator::ik_swing_permissive;
  } else {
    std::cerr << "bad ik error sensitivity " << s << "\n";
    exit(1);
  }
}

/////////////////////////////


const int planningTab::mNumBodyDofs = 24;
std::string planningTab::mBodyDofNames[] = {"Body_LSP", "Body_LSR", "Body_LSY", "Body_LEP", "Body_LWY", "Body_LWP",
					    "Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP",
					    "Body_LHY", "Body_LHR", "Body_LHP", "Body_LKP", "Body_LAP", "Body_LAR",
					    "Body_RHY", "Body_RHR", "Body_RHP", "Body_RKP", "Body_RAP", "Body_RAR"};

std::string planningTab::mBodyJointNames[] = {"LSP", "LSR", "LSY", "LEP", "LWY", "LWP",
					      "RSP", "RSR", "RSY", "REP", "RWY", "RWP",
					      "LHY", "LHR", "LHP", "LKP", "LAP", "LAR",
					      "RHY", "RHR", "RHP", "RKP", "RAP", "RAR"};

// Define IDs for buttons
enum DynamicSimulationTabEvents {
  id_button_1 = 8345,
  id_button_2,
  id_button_3,
  id_button_4,
  id_button_Plan,
  id_button_SetController
};

// Handler for events
BEGIN_EVENT_TABLE(planningTab, wxPanel)
EVT_COMMAND (id_button_1, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButton1)
EVT_COMMAND (id_button_2, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButton2)
EVT_COMMAND (id_button_3, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButton3)
EVT_COMMAND (id_button_4, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButton4)
EVT_COMMAND (id_button_Plan, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonPlan)
EVT_COMMAND (id_button_SetController, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonSetController)
END_EVENT_TABLE()

IMPLEMENT_DYNAMIC_CLASS(planningTab, GRIPTab)

planningTab::planningTab(wxWindow *parent, const wxWindowID id, 
			 const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style)
{
  // Create user interface
  wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
  
  wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Setup planning problem"));
  wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Check"));
  wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Execute"));
  
  wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
  wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
  wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);

  ss1BoxS->Add(new wxButton(this, id_button_1, wxT("Set start")), 0, wxALL, 1); 
  ss1BoxS->Add(new wxButton(this, id_button_2, wxT("Button 2")), 0, wxALL, 1); 

  ss2BoxS->Add(new wxButton(this, id_button_3, wxT("Prev Step")), 0, wxALL, 1); 
  ss2BoxS->Add(new wxButton(this, id_button_4, wxT("Next Step")), 0, wxALL, 1); 

  ss3BoxS->Add(new wxButton(this, id_button_Plan, wxT("Run")), 0, wxALL, 1); 
  ss3BoxS->Add(new wxButton(this, id_button_SetController, wxT("Set Controller")), 0, wxALL, 1); 

  sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);

  SetSizer(sizerFull);

  // Initialize variables
  mRobotIndex = 0; // We only simulate one robot in this demo so we know its index is 0

  // Set predefined start and goal configuration
  mPredefStartConf.resize( mNumBodyDofs );
  mPredefGoalConf.resize( mNumBodyDofs );

  mPredefStartConf << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.174533,  0.349066, -0.174533, 0, 0, 0, -0.174533, 0.349066, -0.174533, 0;
  mPredefGoalConf <<  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.174533,  0.349066, -0.174533, 0, 0, 0, -0.174533, 0.349066, -0.174533, 0;
  
  mStartConf = mPredefStartConf;
  mGoalConf = mPredefGoalConf;

  mPathDelta = 10;
}


/// Gets triggered after a world is loaded
void planningTab::GRIPEventSceneLoaded() {
  
  // Store the indices for the Body Dofs
  mBodyDofs.resize( mNumBodyDofs );
  std::cout << "BodyDofs: " << std::endl;
  for(int i = 0; i < mBodyDofs.size(); i++) {
    mBodyDofs[i] = mWorld->getRobot(mRobotIndex)->getNode( mBodyDofNames[i].c_str())->getDof(0)->getSkelIndex();
		std::cout << " " << mBodyDofs[i];
  }
  std::cout << std::endl;
  
}


/// Before each simulation step we set the torques the controller applies to the joints
void planningTab::GRIPEventSimulationBeforeTimestep() {
  Eigen::VectorXd torques = mController->getTorques(mWorld->getRobot(mRobotIndex)->getPose(), mWorld->getRobot(mRobotIndex)->getQDotVector(), mWorld->mTime);
  mWorld->getRobot(mRobotIndex)->setInternalForces(torques);
}


/// Button 1
void planningTab::onButton1(wxCommandEvent & _evt) {

  if(!mWorld || mWorld->getNumRobots() < 1) {
    cout << "No world loaded or world does not contain a robot." << endl;
    return;
  }
  
  // Set predef Start
  mWorld->getRobot(mRobotIndex)->setConfig(mBodyDofs, mPredefStartConf);
  viewer->DrawGLScene();
}

/// Button 2
void planningTab::onButton2(wxCommandEvent & _evt) {

}

/**
 * @function Button 3
 * @brief Prev Step
 */
void planningTab::onButton3(wxCommandEvent & _evt) {
 
  if (path_index < 0)
    path_index = 0;
  else if (path_index >= mMzPath.size() - 1)
    path_index = mMzPath.size()-1;
  
  // Set configuration
  mWorld->getRobot(mRobotIndex)->setConfig( mBodyDofs, mMzPath[path_index] );
	mWorld->getRobot( mRobotIndex )->update();
  std::cout << "Configuration at path index "<< path_index<<std::endl;
  std::cout << mMzPath[path_index].transpose() <<std::endl;
  viewer->DrawGLScene();

  // Update path index
  path_index = path_index-mPathDelta;  
  
}

/**
 * @function Button 4
 * @brief Next Step
 */
void planningTab::onButton4(wxCommandEvent & _evt) {
  
  if (path_index < 0)
    path_index = 0;
  else if (path_index >= mMzPath.size() - 1)
    path_index = mMzPath.size()-1;
  
  // Set configuration
  mWorld->getRobot(mRobotIndex)->setConfig(mBodyDofs, mMzPath[path_index]);
  std::cout << "Configuration at path index "<< path_index<<std::endl;
  std::cout << mMzPath[path_index].transpose()<<std::endl;  
  viewer->DrawGLScene();
  
  // Update path index
  path_index = path_index+ mPathDelta;
  
}

/// Set initial dynamic parameters and call planner and controller
void planningTab::onButtonPlan(wxCommandEvent & _evt) {

  bool show_gui = false;
  bool use_ach = false;

  walktype walk_type = walk_canned;
  double walk_circle_radius = 5.0;
  double walk_dist = 20;

  double footsep_y = 0.085; // half of horizontal separation distance between feet
  double foot_liftoff_z = 0.05; // foot liftoff height

  double step_length = 0.05;
  bool walk_sideways = false;

  double com_height = 0.48; // height of COM above ANKLE
  double com_ik_ascl = 0;

  double zmpoff_y = 0; // lateral displacement between zmp and ankle
  double zmpoff_x = 0;

  double lookahead_time = 2.5;

  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.05;
  double single_support_time = 0.70;

  size_t max_step_count = 4;

  double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller

  ZMPWalkGenerator::ik_error_sensitivity ik_sense = ZMPWalkGenerator::ik_strict;


  const char* hubofile = "../myhubo.kinbody.xml";

  HuboPlus hplus(hubofile);


  //////////////////////////////////////////////////////////////////////
  // build initial state

  // the actual state
  ZMPWalkGenerator walker(hplus,
			  ik_sense,
                          com_height,
                          zmp_jerk_penalty,
			  zmpoff_x,
			  zmpoff_y,
                          com_ik_ascl,
                          single_support_time,
                          double_support_time,
                          startup_time,
                          shutdown_time,
                          foot_liftoff_z,
			  lookahead_time
    );
  ZMPReferenceContext initContext;

  // helper variables and classes
  const KinBody& kbody = hplus.kbody;
  const JointLookup& jl = hplus.jl;
  double deg = M_PI/180; // for converting from degrees to radians

  // fill in the kstate
  initContext.state.body_pos = vec3(0, 0, 0.85);
  initContext.state.body_rot = quat();
  initContext.state.jvalues.resize(kbody.joints.size(), 0.0);
  initContext.state.jvalues[jl("LSR")] =  15*deg;
  initContext.state.jvalues[jl("RSR")] = -15*deg;
  initContext.state.jvalues[jl("LSP")] =  20*deg;
  initContext.state.jvalues[jl("RSP")] =  20*deg;
  initContext.state.jvalues[jl("LEP")] = -40*deg;
  initContext.state.jvalues[jl("REP")] = -40*deg;
  
  // build and fill in the initial foot positions

  Transform3 starting_location(quat::fromAxisAngle(vec3(0,0,1), 0));
  initContext.feet[0] = Transform3(starting_location.rotation(), starting_location * vec3(0, footsep_y, 0));
  initContext.feet[1] = Transform3(starting_location.rotation(), starting_location * vec3(0, -footsep_y, 0));

  // fill in the rest
  initContext.stance = DOUBLE_LEFT;
  initContext.comX = Eigen::Vector3d(zmpoff_x, 0.0, 0.0);
  initContext.comY = Eigen::Vector3d(0.0, 0.0, 0.0);
  initContext.eX = 0.0;
  initContext.eY = 0.0;
  initContext.pX = 0.0;
  initContext.pY = 0.0;

  // apply COM IK for init context
  walker.applyComIK(initContext);

  /*
  walker.traj.resize(1);
  walker.refToTraj(initContext, walker.traj.back());
  */


  walker.initialize(initContext);

  
  //////////////////////////////////////////////////////////////////////
  // build ourselves some footprints
  
  Footprint initLeftFoot = Footprint(initContext.feet[0], true);
  Footprint initRightFoot = Footprint(initContext.feet[1], false);

  std::vector<Footprint> footprints;

  switch (walk_type) {
  case walk_circle: {

    double circle_max_step_angle = M_PI / 12.0; // maximum angle between steps TODO: FIXME: add to cmd line??
  
    footprints = walkCircle(walk_circle_radius,
			    walk_dist,
			    footsep_y,
			    step_length,
			    circle_max_step_angle,
			    &initLeftFoot,
			    &initRightFoot,
			    false);

    break;

  }

  case walk_line: {

    footprints = walkLine(walk_dist, footsep_y,
			  step_length,
			  &initLeftFoot,
			  &initRightFoot,
			  false);

    break;

  }

  default: {

    double cur_x[2] = { 0, 0 };
    double cur_y[2] = { 0, 0 };

    cur_y[0] =  footsep_y;
    cur_y[1] = -footsep_y;
    
    for (size_t i=0; i<max_step_count; ++i) {
      bool is_left = i%2;
      if (walk_sideways && step_length < 0) { is_left = !is_left; }
      int swing = is_left ? 0 : 1;
      int stance = 1-swing;
      if (walk_sideways) {
	cur_y[swing] -= step_length;
      } else {
	if (i + 1 == max_step_count) {
	  cur_x[swing] = cur_x[stance];
	} else {
	  cur_x[swing] = cur_x[stance] + 0.5*step_length;
	}
      }
      footprints.push_back(Footprint(cur_x[swing], cur_y[swing], 0, is_left));
    }

    break;

  }
  }

  if (footprints.size() > max_step_count) {
    footprints.resize(max_step_count);
  }

  //////////////////////////////////////////////////////////////////////
  // and then build up the walker


  walker.stayDogStay(startup_time * TRAJ_FREQ_HZ);


  for(std::vector<Footprint>::iterator it = footprints.begin(); it != footprints.end(); it++) {
    walker.addFootstep(*it);
  }



  walker.stayDogStay(shutdown_time * TRAJ_FREQ_HZ);
  

  //////////////////////////////////////////////////////////////////////
  // have the walker run preview control and pass on the output
  
  walker.bakeIt();
  // validateOutputData(traj);
  
/*
  if (show_gui) {

  ZmpDemo demo(argc, argv, hplus, walker.traj);
  
  demo.run();
  
  }*/
  
  // Save joints
  std::vector<int> jointOrderFake( mNumBodyDofs );
  std::cout << "The trajectory size was: "<< walker.traj.size() << std::endl;
  // Got this from HuboPlus.cpp line 318 (hnames)
  // Still not sure why this is the way it is...but that is it so don't ask me
  jointOrderFake[0] = 4; jointOrderFake[1] = 5;
  jointOrderFake[2] = 6; jointOrderFake[3] = 7;
  jointOrderFake[4] = 8; jointOrderFake[5] = 10;
  
  jointOrderFake[6] = 11; jointOrderFake[7] = 12;
  jointOrderFake[8] = 13; jointOrderFake[9] = 14;
  jointOrderFake[10] = 15; jointOrderFake[11] = 17;
  
  jointOrderFake[12] = 19; jointOrderFake[13] = 20;
  jointOrderFake[14] = 21; jointOrderFake[15] = 22;
  jointOrderFake[16] = 23; jointOrderFake[17] = 24;
  
  jointOrderFake[18] = 26; jointOrderFake[19] = 27;
  jointOrderFake[20] = 28; jointOrderFake[21] = 29;
  jointOrderFake[22] = 30; jointOrderFake[23] = 31;
  
  // Store path
  FILE* pFile;
  pFile = fopen( "traj.txt", "w");
  
  mMzPath.resize( walker.traj.size() );
  for( int i = 0; i < walker.traj.size(); ++i ) {
    Eigen::VectorXd waypoint = Eigen::VectorXd::Zero( mNumBodyDofs );
    
    fprintf( pFile, "%d ", i );
    for( int j = 0; j < mNumBodyDofs; ++j ) {
      waypoint(j) = walker.traj[i].angles[ jointOrderFake[j] ];
      fprintf( pFile, "%.4f  ", waypoint(j) );
    }		
    fprintf(  pFile, "\n" );
    
    mMzPath[i] = waypoint;
  }
  
  fclose( pFile );
  
  
  std::cout << "Done and ready to step back and forth!" << std::endl;
  
}

/**
 * @function onButtonSetController
 */
void planningTab::onButtonSetController(wxCommandEvent & _evt) {

  // If mMzPath has been filled up
  if( mMzPath.size() <= 0 ) {
    std::cout << "No Mz Path generated, exiting of controller!" << std::endl;
    return;
  }

  // Store the actuated joints (all except the first 6 which are only a convenience to locate the robot in the world)
  std::vector<int> actuatedDofs(mWorld->getRobot(mRobotIndex)->getNumDofs() - 6);
  for(unsigned int i = 0; i < actuatedDofs.size(); i++) {
    actuatedDofs[i] = i + 6;
  }
  
  // Define PD controller gains
  Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(mRobotIndex)->getNumDofs());
  Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones(mWorld->getRobot(mRobotIndex)->getNumDofs());
  Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones(mWorld->getRobot(mRobotIndex)->getNumDofs());

  // Define gains for the ankle PD
  std::vector<int> ankleDofs(2);
  ankleDofs[0] = 27;
  ankleDofs[1] = 28;
  const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
  const Eigen::VectorXd ankleDGains = -200.0 * Eigen::VectorXd::Ones(2);

  // Put the robot near the floor (hacky, I know)
  Eigen::VectorXd rootPose = mWorld->getRobot( mRobotIndex )->getRootTransform();
  rootPose(2) = 0.94; // z
  mWorld->getRobot( mRobotIndex )->setRootTransform( rootPose );
  
  // Set the robot to start configuration of waypoints
  mWorld->getRobot( mRobotIndex )->setConfig( mBodyDofs, mMzPath[0] );
  mWorld->getRobot( mRobotIndex )->update();

  // Create controller
  mController = new Controller( mWorld->getRobot(mRobotIndex), 
				actuatedDofs, 
				kP, kD, ankleDofs, 
				anklePGains, ankleDGains);


  // Convert path into time-parameterized trajectory satisfying acceleration and velocity constraints
 std:vector<double> nominalVels( mBodyDofs.size() );
  for( int i = 0; i < mBodyDofs.size(); ++i ) {
    nominalVels[i] = 0.3;
  }

  double dt = 0.005; // 200 Hz
  mController->setWaypoints( mMzPath, mBodyDofs,
			     nominalVels,
			     0.0, dt );
  
}

// Local Variables:
// c-basic-offset: 2
// End:

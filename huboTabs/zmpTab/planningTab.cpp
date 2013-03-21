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
  id_button_Plan
};

// Handler for events
BEGIN_EVENT_TABLE(planningTab, wxPanel)
EVT_COMMAND (id_button_1, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButton1)
EVT_COMMAND (id_button_2, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButton2)
EVT_COMMAND (id_button_3, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButton3)
EVT_COMMAND (id_button_4, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButton4)
EVT_COMMAND (id_button_Plan, wxEVT_COMMAND_BUTTON_CLICKED, planningTab::onButtonPlan)
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
  for(int i = 0; i < mBodyDofs.size(); i++) {
    mBodyDofs[i] = mWorld->getRobot(mRobotIndex)->getNode( mBodyDofNames[i].c_str())->getDof(0)->getSkelIndex();
  }
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
  else if (path_index >= computed_path.size())
    path_index = computed_path.size()-1;
  
  // Set configuration
  mWorld->getRobot(mRobotIndex)->setConfig( mBodyDofs, computed_path[path_index] );
  std::cout << "Configuration at path index "<< path_index<<std::endl;
  std::cout << computed_path[path_index].transpose() <<std::endl;
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
  else if (path_index >= computed_path.size())
    path_index = computed_path.size()-1;
  
  // Set configuration
  mWorld->getRobot(mRobotIndex)->setConfig(mBodyDofs, computed_path[path_index]);
  std::cout << "Configuration at path index "<< path_index<<std::endl;
  std::cout << computed_path[path_index].transpose()<<std::endl;  
  viewer->DrawGLScene();
  
  // Update path index
  path_index = path_index+ mPathDelta;
  
}

/// Set initial dynamic parameters and call planner and controller
void planningTab::onButtonPlan(wxCommandEvent & _evt) {

  dynamics::SkeletonDynamics* ground = mWorld->getSkeleton("ground");

  // Convert path into time-parameterized trajectory satisfying acceleration and velocity constraints
  const Eigen::VectorXd maxVelocity = 10*0.6 * Eigen::VectorXd::Ones( mNumBodyDofs );
  const Eigen::VectorXd maxAcceleration = 10*0.6 * Eigen::VectorXd::Ones( mNumBodyDofs );

  TrajVector traj;
  std::cout << "Planning with Matt's code" << std::endl;

  fakerave::KinBody my_kbody;
  computeTrajectory_zmp( traj, "myhubo.kinbody.xml", my_kbody );

  printf("Finished computing trajectory \n");

  std::list<Eigen::VectorXd> my_path;
  TrajVector::iterator itTraj;
  Eigen::VectorXd angles = Eigen::VectorXd::Zero( mNumBodyDofs );

  // Get joint indices in Matt's code
  std::vector<int> jointIndices( mNumBodyDofs );
  int ind;
  for (int i = 0; i < mNumBodyDofs; ++i) {
    ind = my_kbody.lookupJoint( mBodyJointNames[i] );
    
    if ( ind != -1 ) {
      jointIndices[i] = ind;
    } else {
      std::cout <<"Not found joint indice in Matt's code: JOINT "<<mBodyJointNames[i] <<std::endl;
    }

  }
  
  for (itTraj = traj.begin(); itTraj != traj.end(); ++itTraj) {

    for (int i = 0; i < mNumBodyDofs; ++i) { 
      angles[i] = (itTraj->angles)[ jointIndices[i] ];
    }
    
    my_path.push_back(angles);
    computed_path.push_back(angles);
  }
  
  
  int num_elements = traj.size();
  
  std::cout << "num_steps: " << num_elements << std::endl;
  
  
  planning::Trajectory* trajectory = new planning::Trajectory(my_path, maxVelocity, maxAcceleration);
  std::cout << "-- Trajectory duration: " << trajectory->getDuration() << endl;
 
}

// Local Variables:
// c-basic-offset: 2
// End:

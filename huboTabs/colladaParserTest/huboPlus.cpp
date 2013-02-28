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

#include "huboPlus.h"

// **********************
// STL
#include <wx/wx.h>
#include <iostream>

using namespace std;

// **********************
// GRIP UI stuff
#include <Tabs/AllTabs.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <GRIPApp.h>

// **********************
// Dynamics Stuff
#include <collision/CollisionShapes.h>
#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>
#include <kinematics/Transformation.h>
// **********************
// Drawing Stuff
#include <wx/glcanvas.h>
#include <GUI/Viewer.h>

#include <robotics/parser/dart_parser/DartLoader.h>
#include <boost/make_shared.hpp>

/** UI Control IDs */
enum DynamicSimulationTabEvents {
  id_button_Parser = 8345,
  id_button_LocalCoM,
  id_button_GeomOffset
};

/** Handlers for events **/
BEGIN_EVENT_TABLE(VisualizationTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, VisualizationTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, VisualizationTab::OnSlider)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(VisualizationTab, GRIPTab)

/**
 * @function VisualizationTab
 * @brief Constructor
 */
VisualizationTab::VisualizationTab(wxWindow *parent,
                         const wxWindowID id,
                         const wxPoint& pos,
                         const wxSize& size,
                         long style) :
GRIPTab(parent, id, pos, size, style) {
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
  // Create Static boxes (outline of your Tab)
  wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Parse"));
  
  // Create sizers for these static boxes
  wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
  
  // Add Floor button
  ss1BoxS->Add(new wxButton(this, id_button_Parser, wxT("Parse")), 0, wxALL, 1); 
  // Add CoM buttons
  ss1BoxS->Add(new wxButton(this, id_button_LocalCoM, wxT("Show local CoM")), 0, wxALL, 1); 
  // Add offset button
  ss1BoxS->Add(new wxButton(this, id_button_GeomOffset, wxT("Show geometry offset")), 0, wxALL, 1); 

  // Add the boxes to their respective sizers
  sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);

  SetSizer(sizerFull);
}

/**
 * @function loadColladaFile
 */
void VisualizationTab::loadColladaFile( std::string _filename ) {

  // Default string("/home/ana/Research/openHubo/robots/huboplus/huboplus.kinbody_COLLADA.xml");
  urdf::Model robot;
  if ( !robot.initFile(_filename) ) {
    printf( "[ERROR] Model Parsing the xml failed \n" );
  }
  
  printf( "* Robot name is: %s \n", robot.getName().c_str() );
  
  // get info from parser
  printf(" ---------- Successfully Parsed XML --------------- \n" );

  // get root link
  boost::shared_ptr<const urdf::Link> root_link=robot.getRoot();
  if (!root_link) {
    printf(" [ERROR] Did not get root \n");
  }

  printf( "* Root Link: %s has %d  child(ren) \n",root_link->name.c_str(), root_link->child_links.size() );
  
  // print entire tree
  printTree(root_link);
  
  // Parse robot from Model
  boost::shared_ptr<urdf::ModelInterface> robotInterface;
  urdf::ModelInterface robotI  = (urdf::ModelInterface) robot;
  robotInterface = boost::make_shared<urdf::ModelInterface>(robotI);
  robotics::Robot* myRobot;
  DartLoader dl;
  myRobot = dl.modelInterfaceToRobot( robotInterface  );
  
  mWorld->addRobot( myRobot );
  mWorld->rebuildCollision();
  
  treeView->CreateFromWorld();

}

/**
 * @function OnButton
 * @brief Handles button events
 */
void VisualizationTab::OnButton(wxCommandEvent & _evt) {
    int slnum = _evt.GetId();
  
    switch( slnum ) {
      
    case id_button_Parser: {
      
      wxString filename = wxFileSelector(wxT("Choose a file to open"),
					 wxT("."),wxT(""),wxT(""), // -- default extension
					 wxT("*.xml"), 0);
      if ( !filename.empty() ) {
	loadColladaFile( (std::string) filename.mb_str() );
      }
      
    }
      break;

      /** Local CoM */
    case id_button_LocalCoM:
  printf("** Display Robot info **\n");


  /** Root node info **/
  printf(" Skeleton has %d DOFs, %d nodes and %d joints \n", mWorld->getRobot(0)->getNumDofs(),
	 mWorld->getRobot(0)->getNumNodes(), mWorld->getRobot(0)->getNumJoints() );

  printf("Root node: %s ", mWorld->getRobot(0)->getRoot()->getName() );
  if( mWorld->getRobot(0)->getRoot()->getParentNode() == NULL ) {
    printf(" has as parent NULL"); 
  }
  
  printf(" and its joint with NULL (of type %d) has %d DOF and %d Transforms \n", 
	 mWorld->getRobot(0)->getRoot()->getParentJoint()->getJointType(),
	 mWorld->getRobot(0)->getRoot()->getParentJoint()->getNumDofs(),
	 mWorld->getRobot(0)->getRoot()->getParentJoint()->getNumTransforms() );

  for( int i = 0; 
       i < mWorld->getRobot(0)->getRoot()->getParentJoint()->getNumDofs();
       ++i ) {
    printf("Root DOF(%d): %s \n \n", i, mWorld->getRobot(0)->getRoot()->getParentJoint()->getDof(i)->getName() );
  }

  for( int i = 0; 
       i < mWorld->getRobot(0)->getRoot()->getParentJoint()->getNumTransforms();
       ++i ) {
    printf("Root Transform(%d): %s \n \n", i, mWorld->getRobot(0)->getRoot()->getParentJoint()->getTransform(i)->getName() );
  }


  /** Body Nodes info **/
  for( int i = 0; i < mWorld->getRobot(0)->getNumNodes(); ++i ) {
    printf("BodyNode [%d]: %s with parent joint of type %d ", i, mWorld->getRobot(0)->getNode(i)->getName(), mWorld->getRobot(0)->getNode(i)->getParentJoint()->getJointType() );
    printf(" with %d child joints and %d local DOFs \n", mWorld->getRobot(0)->getNode(i)->getNumChildJoints(), mWorld->getRobot(0)->getNode(i)->getNumLocalDofs() );
    for( int j = 0; j < mWorld->getRobot(0)->getNode(i)->getNumChildJoints(); ++j ) {
      printf("\t -- Child node: %s \n", mWorld->getRobot(0)->getNode(i)->getChildNode(j)->getName() );
    }
  }


      break;

      /** Geometry offset */
    case id_button_GeomOffset:

      break;


    default: {
        /** Default */
        printf("Default button \n");
        break;
    }
    }
}

void VisualizationTab::printTree( boost::shared_ptr<const urdf::Link> link,
				  int level)
{
  level+=2;
  int count = 0;
  for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
  {
    if (*child)
    {
      for(int j=0;j<level;j++) std::cout << "  "; //indent
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << std::endl;
      // first grandchild
      printTree(*child,level);
    }
    else
    {
      for(int j=0;j<level;j++) std::cout << " "; //indent
      std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
    }
  }

}

void VisualizationTab::GRIPEventSceneLoaded() {
    int mGroundIndex = -1;
    for(int i = 0; i < mWorld->getNumObjects(); ++i) {
        if(mWorld->getObject(i)->getName() == "ground") {
            mGroundIndex = i; break;
        }
    }
    if (mGroundIndex != -1) {
        std::cout << "-- Found ground as object %d \n" << std::endl;
    }
    else {
        robotics::Object* ground = new robotics::Object();
        ground->setName("ground");
        dynamics::BodyNodeDynamics* node = new dynamics::BodyNodeDynamics();
        node->setShape(new kinematics::ShapeBox(Eigen::Vector3d(10.0, 10.0, 0.0001), 1.0));
        kinematics::Joint* joint = new kinematics::Joint(NULL, node);
        ground->addNode(node);
        ground->initSkel();
        ground->update();
        ground->setImmobileState(true);

        mWorld->addObject(ground);
        mWorld->rebuildCollision();
        mGroundIndex = mWorld->getNumObjects() - 1;
        printf("-- Added ground as object %d\n", mGroundIndex);
    }
    treeView->CreateFromWorld();
}

/**
 * @function GRIPEventSimulationBeforeTimeStep
 * @brief 
 */
void VisualizationTab::GRIPEventSimulationBeforeTimestep() {
}

/**
 * @function GRIPEventSimulationAfterTimeStep
 * @brief
 */
void VisualizationTab::GRIPEventSimulationAfterTimestep() {
}

/**
 * @function GRIPEventSimulationStart
 * @brief
 */
void VisualizationTab::GRIPEventSimulationStart() {
}

/**
 * @function GRIPEventRender
 * @brief
 */
void VisualizationTab::GRIPEventRender() {
    // Draw some stuff
    glDisable(GL_FOG);

    // setup - we'll clean these up ourselves
    glEnable(GL_COLOR_MATERIAL);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    // set some things for ourselves - will be overwritten by later gl calls
    glLineWidth(1.5f);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);

    // do drawing
    if (mWorld && mWorld->mCollisionHandle) {
        // some preprocessing. calculate vector lengths and find max
        // length, scale down the force measurements, and figure out
        // which contact points involve to the selected body nodes
        int nContacts = mWorld->mCollisionHandle->getCollisionChecker()->getNumContact();
        Eigen::Vector3d vs[nContacts];
        Eigen::Vector3d fs[nContacts];
        float lens[nContacts];
        bool selected[nContacts];
        float maxl = 0;
        for (int k = 0; k < nContacts; k++) {
            collision_checking::ContactPoint contact = mWorld->mCollisionHandle->getCollisionChecker()->getContact(k);
            vs[k] = contact.point;
            fs[k] = contact.force.normalized() * .1 * log(contact.force.norm());
            lens[k] = (vs[k] - fs[k]).norm();
            if (lens[k] > maxl) maxl = lens[k];
            selected[k] = false;
            kinematics::BodyNode* conNode = contact.bd1;
            while (conNode != NULL) { // ascend the tree to see if any
                                      // ancestors are selected
                if (conNode == selectedNode) selected[k] = true;
                conNode = conNode->getParentNode();
            }
            conNode = contact.bd2;
            while (conNode != NULL) {
                if (conNode == selectedNode) selected[k] = true;
                conNode = conNode->getParentNode();
            }
        }
        
        // then actually do drawing.
        Eigen::Vector3d v;
        Eigen::Vector3d f;
        Eigen::Vector3d vf;
        Eigen::Vector3d arrowheadDir;
        Eigen::Vector3d arrowheadBase;
        glBegin(GL_LINES);
        for (int k = 0; k < nContacts; k++) {
            if (selected[k]) {
                glColor3d(0.0, lens[k] / (2 * maxl) + .5, 0.0);
            }
            else {
                glColor3d(lens[k] / (2 * maxl) + .5, 0.0, 0.0);
            }
            v = vs[k];
            f = fs[k];
            vf = v + f;
            arrowheadDir = v.cross(f).normalized() * .0075;
            arrowheadBase = vf - f.normalized() * .02;
            glVertex3f(v[0], v[1], v[2]);
            glVertex3f(vf[0], vf[1], vf[2]);
            glVertex3f(vf[0], vf[1], vf[2]);
            glVertex3f(arrowheadBase[0] + arrowheadDir[0], arrowheadBase[1] + arrowheadDir[1], arrowheadBase[2] + arrowheadDir[2]);
            glVertex3f(vf[0], vf[1], vf[2]);
            glVertex3f(arrowheadBase[0] - arrowheadDir[0], arrowheadBase[1] - arrowheadDir[1], arrowheadBase[2] - arrowheadDir[2]);
        }
        glEnd();
    }

    // clean up after ourselves
    glColor3f(1.0f,1.0f,1.0f);
    glEnable(GL_LIGHTING);
    glDisable(GL_FOG);
    glDisable(GL_COLOR_MATERIAL);
}

/**
 * @function OnSlider
 * @brief Handles slider changes
 */
void VisualizationTab::OnSlider(wxCommandEvent &evt) {
}

// This function is called when an object is selected in the Tree View or other
// global changes to the GRIP world. Use this to capture events from outside the tab.
void VisualizationTab::GRIPStateChange() {
    if(selectedTreeNode==NULL){
        return;
    }

    switch (selectedTreeNode->dType) {
    case Return_Type_Object: {
        robotics::Object* pObject = (robotics::Object*)(selectedTreeNode->data);
        selectedNode = pObject->mRoot;
        break;
    }
    case Return_Type_Robot: {
        robotics::Robot* pRobot = (robotics::Robot*)(selectedTreeNode->data);
        selectedNode = pRobot->mRoot;
        break;
    }
    case Return_Type_Node: {
        dynamics::BodyNodeDynamics* pBodyNode = (dynamics::BodyNodeDynamics*)(selectedTreeNode->data);
        selectedNode = pBodyNode;
        break;
    }
    default: {
        fprintf(stderr, "someone else's problem.");
        assert(0);
        exit(1);
    }
    }
}



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

#include "VisualTester.h"

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

// **********************
// Drawing Stuff
#include <wx/glcanvas.h>
#include <GUI/Viewer.h>

// **********************
// To draw CoM
#include <GL/glu.h>
#include <GL/glut.h>
// ***********************


/** UI Control IDs */
enum DynamicSimulationTabEvents {
    id_checkbox_showcontacts = wxID_HIGHEST,
    id_button_showCoM,
    id_button_showJointAxis,
    id_offsetX_Slider,
    id_offsetY_Slider,
    id_offsetZ_Slider
};

/** Handlers for events **/
BEGIN_EVENT_TABLE(VisualTester, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, VisualTester::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, VisualTester::OnSlider)
EVT_CHECKBOX(id_checkbox_showcontacts, VisualTester::OnCheckShowContacts)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(VisualTester, GRIPTab)

/**
 * @function VisualTester
 * @brief Constructor
 */
VisualTester::VisualTester(wxWindow *parent,
                         const wxWindowID id,
                         const wxPoint& pos,
                         const wxSize& size,
                         long style) :
GRIPTab(parent, id, pos, size, style) {
    sizerFull = new wxBoxSizer(wxHORIZONTAL);
    wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Display Options"));
    wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);

    checkShowContacts = new wxCheckBox(this, id_checkbox_showcontacts, wxT("Show Contact Forces"));
    ss1BoxS->Add(checkShowContacts, 0, wxALL, 1);

    wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Tester Display Options"));
    wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer( ss2Box, wxHORIZONTAL );
    

    wxBoxSizer* testerButtonsBoxS = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* testerSlidersBoxS = new wxBoxSizer(wxVERTICAL);

    testerButtonsBoxS->Add(new wxButton(this, id_button_showCoM, wxT("Show CoM")), 0, wxALL, 1);
    testerButtonsBoxS->Add(new wxButton(this, id_button_showJointAxis, wxT("Show Joint Axis")), 0, wxALL, 1 );

    ss2BoxS->Add( testerButtonsBoxS, 1, wxALIGN_NOT );

    mOffsetX_Slider = new GRIPSlider("offset X",-1.0,1.0,2000,0,1000,2000,this,id_offsetX_Slider);
    testerSlidersBoxS->Add( mOffsetX_Slider, 1, wxEXPAND | wxALL, 6 );
    mOffsetY_Slider = new GRIPSlider("offset Y",-1.0,1.0,2000,0.5,1000,2000,this,id_offsetY_Slider);
    testerSlidersBoxS->Add( mOffsetY_Slider, 1, wxEXPAND | wxALL, 6 );
    mOffsetZ_Slider = new GRIPSlider("offset Z",-1.0,1.0,2000,0,1000,2000,this,id_offsetZ_Slider);
    testerSlidersBoxS->Add( mOffsetZ_Slider, 1, wxEXPAND | wxALL, 6 );

    ss2BoxS->Add( testerSlidersBoxS, 1, wxALIGN_NOT );

    sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 1);
    sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 1);
    SetSizer(sizerFull);

    // Initialize
    mDrawCoMFlag = false;
    mDrawJointAxisFlag = false;
}


/**
 * @function OnButton
 * @brief Handles button events
 */
void VisualTester::OnButton(wxCommandEvent & _evt) {
    int slnum = _evt.GetId();
  
    switch( slnum ) {
    case id_button_showCoM: {
      printf("Show CoM \n");
      mDrawCoMFlag = !mDrawCoMFlag;
      if( mDrawCoMFlag ) { printf("CoM drawing ON! \n"); }
      else { printf("CoM drawing OFF! \n"); }
    }
      break;
      
    case id_button_showJointAxis: {
      mDrawJointAxisFlag = !mDrawJointAxisFlag;
      if( mDrawJointAxisFlag ) { printf("Joint Axis drawing ON! \n"); }
      else { printf("Joint Axis drawing OFF! \n"); }
    }
      break;

    default: {
      /** Default */
      printf("Default button \n");
      break;
    }

    }
}

void VisualTester::OnCheckShowContacts(wxCommandEvent &evt) {
}

void VisualTester::GRIPEventSceneLoaded() {
}

/**
 * @function GRIPEventSimulationBeforeTimeStep
 * @brief 
 */
void VisualTester::GRIPEventSimulationBeforeTimestep() {
}

/**
 * @function GRIPEventSimulationAfterTimeStep
 * @brief
 */
void VisualTester::GRIPEventSimulationAfterTimestep() {
}

/**
 * @function GRIPEventSimulationStart
 * @brief
 */
void VisualTester::GRIPEventSimulationStart() {
}

/**
 * @function GRIPEventRender
 * @brief
 */
void VisualTester::GRIPEventRender() {
    glDisable(GL_FOG);
    glEnable(GL_COLOR_MATERIAL);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    glLineWidth(1.5f);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);

    // Draw Axis Joint
    if( mDrawJointAxisFlag ) {
 
      Eigen::Vector3d ow, xw, yw, zw; // World Locations of axis
      Eigen::Vector3d ol, xl, yl, zl; // Local locations of axis
      double d = 0.1;
      ol << 0, 0, 0;
      xl << d, 0, 0;
      yl << 0, d, 0;
      zl << 0, 0, d;

      // Draw Axis as lines
      glBegin(GL_LINES);
      
      for( int i = 0; i < mWorld->getRobot(0)->getNumNodes(); ++i ) {
	// Get origin and axis
	ow = mWorld->getRobot(0)->getNode(i)->evalWorldPos(ol);
	xw = mWorld->getRobot(0)->getNode(i)->evalWorldPos(xl);
	yw = mWorld->getRobot(0)->getNode(i)->evalWorldPos(yl);
	zw = mWorld->getRobot(0)->getNode(i)->evalWorldPos(zl);
	// X (red)
	glColor3d(1.0, 0.0, 0.0);
	glVertex3f( ow(0), ow(1), ow(2) );
	glVertex3f( xw(0), xw(1), xw(2) );
	// Y (green)
	glColor3d(0.0, 1.0, 0.0);
	glVertex3f( ow(0), ow(1), ow(2) );
	glVertex3f( yw(0), yw(1), yw(2) );
	// Z (blue)
	glColor3d(0.0, 0.0, 1.0);
	glVertex3f( ow(0), ow(1), ow(2) );
	glVertex3f( zw(0), zw(1), zw(2) );
      }
      glEnd();
    } // end if drawJointAxis


    // Draw CoM
    if( mDrawCoMFlag ) {
      glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
      glEnable( GL_COLOR_MATERIAL );
      glColor3f( 1.0f, 0.0f, 0.0f );
      
      // Create a quadric object to store the sphere
      GLUquadricObj *quadric = gluNewQuadric();

      // Get the CoMs
      double offsetY = 0.8;
      double radius = 0.01;
      Eigen::Vector3d cmPos;

      for( int i = 0; i < mWorld->getRobot(0)->getNumNodes(); ++i ) {
	glPushMatrix();
	cmPos = mWorld->getRobot(0)->getNode(i)->getWorldCOM();
	glTranslatef( cmPos(0) +  mCoM_drawOffsetX, 
		      cmPos(1) + mCoM_drawOffsetY, 
		      cmPos(2) + mCoM_drawOffsetZ );
	// Draw it
	gluSphere( quadric, radius, 5, 5 );
	glPopMatrix();
      }
      
      // The CoM of the whole body
      glColor3f( 0.0f, 0.0f, 1.0f );
      cmPos = mWorld->getRobot(0)->getWorldCOM();
      glPushMatrix();
      glTranslatef( cmPos(0) + mCoM_drawOffsetX, 
		    cmPos(1) + mCoM_drawOffsetY, 
		    cmPos(2) + mCoM_drawOffsetZ );
      // Draw it
      gluSphere( quadric, radius, 5, 5 );
      glPopMatrix();
    }

    // draw contact points
    if (checkShowContacts->IsChecked() && mWorld && mWorld->mCollisionHandle) {
        // some preprocessing. calculate vector lengths and find max
        // length, scale down the force measurements, and figure out
        // which contact points involve to the selected body nodes
        int nContacts = mWorld->mCollisionHandle->getCollisionChecker()->getNumContact();
        vector<Eigen::Vector3d> vs(nContacts);
        vector<Eigen::Vector3d> fs(nContacts);
        vector<float> lens(nContacts);
        vector<bool> selected(nContacts);
        float maxl = 0;
        for (int k = 0; k < nContacts; k++) {
            collision_checking::ContactPoint contact = mWorld->mCollisionHandle->getCollisionChecker()->getContact(k);
            vs[k] = contact.point;
            fs[k] = contact.force.normalized() * .1 * log(contact.force.norm());
            lens[k] = (vs[k] - fs[k]).norm();
            if (lens[k] > maxl) maxl = lens[k];
            selected[k] = false;
            if (contact.bd1 == selectedNode || contact.bd2 == selectedNode) {
                selected[k] = true;
            }
        }
        Eigen::Vector3d v;
        Eigen::Vector3d f;
        Eigen::Vector3d vf;
        Eigen::Vector3d arrowheadDir;
        Eigen::Vector3d arrowheadBase;
        glBegin(GL_LINES);
        for (int k = 0; k < nContacts; k++) {
            if (selected[k]) {
                glColor3d(0.0, 1.0, 0.0);
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
}

/**
 * @function OnSlider
 * @brief Handles slider changes
 */
void VisualTester::OnSlider(wxCommandEvent &evt) {

  int slnum = evt.GetId();
  double pos = *(double*) evt.GetClientData();

  switch( slnum ) {

  case id_offsetX_Slider:
     mCoM_drawOffsetX = pos; 
    break;

  case id_offsetY_Slider:
     mCoM_drawOffsetY = pos; 
    break;

  case id_offsetZ_Slider:
     mCoM_drawOffsetZ = pos; 
    break;

  }

}

// This function is called when an object is selected in the Tree View or other
// global changes to the GRIP world. Use this to capture events from outside the tab.
void VisualTester::GRIPStateChange() {
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
    int type = 0;
    wxCommandEvent evt(wxEVT_GRIP_UPDATE_AND_RENDER,GetId());
    evt.SetEventObject(this);
    evt.SetClientData((void*)&type);
    GetEventHandler()->AddPendingEvent(evt);
}



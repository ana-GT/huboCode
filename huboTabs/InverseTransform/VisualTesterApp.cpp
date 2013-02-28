/**
 * @file VisualTesterApp.h
 * @brief Creates application for VisualTester
 * @author A. Huaman (added the CoM and Joint drawing) - original by S.R-H.
 */
#include "GRIPApp.h"
#include "VisualTester.h"

extern wxNotebook* tabView;

class VisualTesterApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new VisualTester(tabView), wxT("VisualTester"));
	}
};

IMPLEMENT_APP(VisualTesterApp)

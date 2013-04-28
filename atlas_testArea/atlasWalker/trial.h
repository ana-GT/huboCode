/**
 * @file trial.h
 * @author A. Huaman (Based on original file of M. Zucker)
 */

#ifndef _TRIAL_ATLAS_DEMO_H_
#define _TRIAL_ATLAS_DEMO_H_

#include <iostream>
#include <string>
#include <tinyWalker/zmp/zmpwalkgenerator.h>

/** Walk types */
enum walktype {
  walk_canned,
  walk_line,
  walk_circle
};

/// Helper functions for parsing
walktype getwalktype(const std::string& s);
ZMPWalkGenerator::ik_error_sensitivity getiksense(const std::string& s);
double getdouble(const char* str);
long getlong(const char* str);
void usage(std::ostream& ostr);

#endif /** _TRIAL_ATLAS_DEMO_H_ */

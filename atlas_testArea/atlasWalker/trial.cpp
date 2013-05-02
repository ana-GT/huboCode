/**
 * @file trial.cpp
 */
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include "trial.h"


/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  if( argc < 1 ) {
    usage( std::cerr );
    return 1;
  }

  // Variables
  walktype walk_type = walk_canned;
  double walk_circle_radius = 5.0;
  double walk_dist = 2.0;

  double foot_separation_y = 0.141; // half of total separation
  double foot_liftoff_z = 0.05;

  double step_length = 0.30; // Total step 2X
  bool walk_sideways = false;

  double com_height = 0.8438;
  double com_ik_ascl = 0;

  
  double zmpoff_y = 0; // lateral displacement between zmp and ankle
  double zmpoff_x = 0;

  double lookahead_time = 2; 
  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.15;
  double single_support_time = 0.85;

  size_t max_step_count = 10;
  double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller
  
  ZMPWalkGenerator::ik_error_sensitivity ik_sense = ZMPWalkGenerator::ik_strict;
  
  const struct option long_options[] = {
    { "show-gui",            no_argument,       0, 'g' },
    { "use-ach",             no_argument,       0, 'A' },
    { "ik-errors",           required_argument, 0, 'I' },
    { "walk-type",           required_argument, 0, 'w' },
    { "walk-distance",       required_argument, 0, 'D' },
    { "walk-circle-radius",  required_argument, 0, 'r' },
    { "max-step-count",      required_argument, 0, 'c' },
    { "foot-separation-y",   required_argument, 0, 'y' },
    { "foot-liftoff-z",      required_argument, 0, 'z' },
    { "step-length",         required_argument, 0, 'l' },
    { "walk-sideways",       no_argument,       0, 'S' },
    { "com-height",          required_argument, 0, 'h' },
    { "comik-angle-weight",  required_argument, 0, 'a' },
    { "zmp-offset-y",        required_argument, 0, 'Y' },
    { "zmp-offset-x",        required_argument, 0, 'X' },
    { "lookahead-time",      required_argument, 0, 'T' },
    { "startup-time",        required_argument, 0, 'p' },
    { "shutdown-time",       required_argument, 0, 'n' },
    { "double-support-time", required_argument, 0, 'd' },
    { "single-support-time", required_argument, 0, 's' },
    { "zmp-jerk-penalty",    required_argument, 0, 'R' },
    { "help",                no_argument,       0, 'H' },
    { 0,                     0,                 0,  0  },
  };

  
  const char* short_options = "gAI:w:D:r:c:y:z:l:Sh:a:Y:X:T:p:n:d:s:R:H";
  
  int opt, option_index;
  
  
  while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 ) {

    switch (opt) {
    case 'I': ik_sense = getiksense(optarg); break;
    case 'w': walk_type = getwalktype(optarg); break;
    case 'D': walk_dist = getdouble(optarg); break;
    case 'r': walk_circle_radius = getdouble(optarg); break;
    case 'c': max_step_count = getlong(optarg); break;
    case 'y': foot_separation_y = getdouble(optarg); break;
    case 'z': foot_liftoff_z = getdouble(optarg); break;
    case 'l': step_length = getdouble(optarg); break;
    case 'S': walk_sideways = true; break;
    case 'h': com_height = getdouble(optarg); break;
    case 'a': com_ik_ascl = getdouble(optarg); break;
    case 'Y': zmpoff_y = getdouble(optarg); break;
    case 'X': zmpoff_x = getdouble(optarg); break;
    case 'T': lookahead_time = getdouble(optarg); break;
    case 'p': startup_time = getdouble(optarg); break;
    case 'n': shutdown_time = getdouble(optarg); break;
    case 'd': double_support_time = getdouble(optarg); break;
    case 's': single_support_time = getdouble(optarg); break;
    case 'R': zmp_jerk_penalty = getdouble(optarg); break;
    case 'H': usage(std::cout); exit(0); break;
    default:  usage(std::cerr); exit(1); break;
    }
  }

  //////////////////////////////////////////////////////////////////////
  // build initial state

  // the actual state
  ZMPWalkGenerator walker(ik_sense,
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
			  lookahead_time);
  printf("COM Height: %f single st: %f double st: %f startup time: %f, shutdown time: %f foot liftoff: %f lookahead: %f \n", com_height, single_support_time,
	 double_support_time, startup_time, shutdown_time, foot_liftoff_z, lookahead_time );
  ZMPReferenceContext initContext;

  // helper variables and classes
  double deg = M_PI/180; // for converting from degrees to radians

  // fill in the kstate
  //initContext.state.body_pos = vec3(0, 0, 0.85);
  //initContext.state.body_rot = quat();
  
  // build and fill in the initial foot positions
  Transform3 starting_location(quat::fromAxisAngle(vec3(0,0,1), 0));
  initContext.feet[0] = Transform3(starting_location.rotation(), starting_location * vec3(0, foot_separation_y, 0));
  initContext.feet[1] = Transform3(starting_location.rotation(), starting_location * vec3(0, -foot_separation_y, 0));

  // fill in the rest
  initContext.stance = DOUBLE_LEFT;
  initContext.comX = Eigen::Vector3d(zmpoff_x, 0.0, 0.0);
  initContext.comY = Eigen::Vector3d(0.0, 0.0, 0.0);
  initContext.eX = 0.0;
  initContext.eY = 0.0;
  initContext.pX = 0.0;
  initContext.pY = 0.0;
  
  // apply COM IK for init context
  //walker.applyComIK(initContext);

  
  walker.traj.resize(1);
  walker.refToTraj(initContext, walker.traj.back());
  


  walker.initialize(initContext);

  
  //////////////////////////////////////////////////////////////////////
  // build ourselves some footprints
  
  Footprint initLeftFoot = Footprint(initContext.feet[0], true);
  /* Footprint initRightFoot = Footprint(initContext.feet[1], false); */

  std::vector<Footprint> footprints;

  switch (walk_type) {
  case walk_circle: {
    
    double circle_max_step_angle = M_PI / 12.0; // maximum angle between steps TODO: FIXME: add to cmd line??
    
    footprints = walkCircle(walk_circle_radius,
			    walk_dist,
			    foot_separation_y,
			    step_length,
			    circle_max_step_angle,
			    initLeftFoot);

    break;

  }
    
  case walk_line: {
    
    footprints = walkLine(walk_dist, foot_separation_y,
			  step_length,
			  initLeftFoot);
	printf("Size footpint: %d \n", footprints.size());
    break;
    
  }

  default: {
printf("Default \n");
    double cur_x[2] = { 0, 0 };
    double cur_y[2] = { 0, 0 };

    cur_y[0] =  foot_separation_y;
    cur_y[1] = -foot_separation_y;
    
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
    
  } // default
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
 
  printf("FREQ: %d \n", TRAJ_FREQ_HZ);
  printf("Num traj points: %d \n", walker.ref.size() );

  // Store
  FILE* zmpF; FILE* lfF; FILE* rfF;
  lfF = fopen( "leftFootMatt.txt", "w" );
  rfF = fopen( "rightFootMatt.txt", "w" );
  zmpF = fopen( "zmpMatt.txt", "w" );

  for( int i = 0; i < walker.ref.size(); ++i ) {
    fprintf( zmpF, "%d %f %f \n", i, walker.ref[i].pX, walker.ref[i].pY );
    fprintf( lfF, "%d %f %f %f \n", i, walker.ref[i].feet[0].matrix()(0,3), walker.ref[i].feet[0].matrix()(1,3), walker.ref[i].feet[0].matrix()(2,3)  );
    fprintf( rfF, "%d %f %f %f \n", i, walker.ref[i].feet[1].matrix()(0,3), walker.ref[i].feet[1].matrix()(1,3), walker.ref[i].feet[1].matrix()(2,3)  );
  }
  
  fclose( lfF );
  fclose( rfF );
  fclose( zmpF );

}

/**
 * @function getwalktype
 */
walktype getwalktype(const std::string& s) {
  if (s == "canned") {
    return walk_canned;
  } else if (s == "line") {
    return walk_line;
  } else if (s == "circle") {
    return walk_circle;
  } else {
    std::cerr << "bad walk type " << s << "\n";
    usage(std::cerr);
    exit(1);
  }
}

/**
 * @function ik_error_sensitivity
 */
ZMPWalkGenerator::ik_error_sensitivity getiksense(const std::string& s) {
  printf("GET IK SENSE \n");
  if (s == "strict") {
    return ZMPWalkGenerator::ik_strict; 
  } else if (s == "sloppy") {
    return ZMPWalkGenerator::ik_sloppy; 
  } else if (s == "permissive") {
    return ZMPWalkGenerator::ik_swing_permissive;
  } else {
    std::cerr << "bad ik error sensitivity " << s << "\n";
    usage(std::cerr);
    exit(1);
  }
}

/**
 * @function getdouble
 * @brief
 */
double getdouble(const char* str) {
  char* endptr;
  double d = strtod(str, &endptr);
  if (!endptr || *endptr) {
    std::cerr << "Error parsing number on command line!\n\n";
    usage(std::cerr);
    exit(1);
  }
  return d;
}

/**
 * @function getlong
 * @brief
 */
long getlong(const char* str) {
  char* endptr;
  long d = strtol(str, &endptr, 10);
  if (!endptr || *endptr) {
    std::cerr << "Error parsing number on command line!\n\n";
    usage(std::cerr);
    exit(1);
  }
  return d;
}

/**
 * @function usage
 */
void usage(std::ostream& ostr) {
  ostr << 
    "usage: zmpdemo [OPTIONS] HUBOFILE.xml\n"
    "\n"
    "OPTIONS:\n"
    "\n"
    "  -g, --show-gui                    Show a GUI after computing trajectories.\n"
    "  -A, --use-ach                     Send trajectory via ACH after computing.\n"
    "  -I, --ik-errors                   IK error handling: strict/sloppy\n"
    "  -w, --walk-type                   Set type: canned/line/circle\n"
    "  -D, --walk-distance               Set maximum distance to walk\n"
    "  -r, --walk-circle-radius          Set radius for circle walking\n"
    "  -c, --max-step-count=NUMBER       Set maximum number of steps\n"
    "  -y, --foot-separation-y=NUMBER    Half-distance between feet\n"
    "  -z, --foot-liftoff-z=NUMBER       Vertical liftoff distance of swing foot\n"
    "  -l, --step-length=NUMBER          Max length of footstep\n"
    "  -S, --walk-sideways               Should we walk sideways? (canned gait only)\n"
    "  -h, --com-height=NUMBER           Height of the center of mass\n"
    "  -a, --comik-angle-weight=NUMBER   Angle weight for COM IK\n"
    "  -Y, --zmp-offset-y=NUMBER         Lateral distance from ankle to ZMP\n"
    "  -X, --zmp-offset-x=NUMBER         Forward distance from ankle to ZMP\n"
    "  -T, --lookahead-time=NUMBER       Lookahead window for ZMP preview controller\n"
    "  -p, --startup-time=NUMBER         Initial time spent with ZMP stationary\n"
    "  -n, --shutdown-time=NUMBER        Final time spent with ZMP stationary\n"
    "  -d, --double-support-time=NUMBER  Double support time\n"
    "  -s, --single-support-time=NUMBER  Single support time\n"
    "  -R, --zmp-jerk-penalty=NUMBER     R-value for ZMP preview controller\n"
    "  -H, --help                        See this message\n";
    
}



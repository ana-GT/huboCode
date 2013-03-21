#include "hubo-zmp.h"
#include "hubomz/HuboPlus.h"
#include "hubomz/ZmpPreview.h"
#include <math.h>
#include "mzcommon/TimeUtil.h"
#include <getopt.h>

using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;

size_t seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}

double sigmoid(double x) {
  return 3*x*x - 2*x*x*x;
}

const int stance_foot_table[4] = { 0, 1, 0, 1 };
const int swing_foot_table[4] = { -1, -1, 1, 0 };

const stance_t next_stance_table[4] = {
  SINGLE_LEFT,
  SINGLE_RIGHT,
  DOUBLE_RIGHT,
  DOUBLE_LEFT
};



/*
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
  
/*
 * @function: validateOutputData(TrajVector& traj)
 * @brief: validation of joint angle output trajectory data
 * @return: void
*/
void validateOutputData(TrajVector& traj) {
    const double dt = 1.0/TRAJ_FREQ_HZ;
    double maxJointVel=0;
    double jointVel;
    const double jointVelTol = 6.0; // radians/s
    for (int n=0; n<(traj.size()-1); n++) {
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

/*
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
*/


int
computeTrajectory_zmp(TrajVector &traj, const char* hubofile, fakerave::KinBody &my_kbody)
{

  bool show_gui = false;
  bool use_ach = false;

  double fy = 0.085; // half of horizontal separation distance between feet
  double zmpy = 0; // lateral displacement between zmp and ankle
  double zmpx = 0;
  double fz = 0.05; // foot liftoff height
  double fx = 0.0; // step length

  double lookahead_time = 2.5;
  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.05;
  double single_support_time = 0.70;

  double com_height = 0.48; // height of COM above ANKLE
  double com_ik_ascl = 0;

  double zmp_R = 1e-8; // jerk penalty on ZMP controller

  size_t n_steps = 8; // how many steps?

  size_t num_lookahead = seconds_to_ticks(lookahead_time); // lookahead window size
  size_t startup_ticks = seconds_to_ticks(startup_time); // hold ZMP at center for 1s at start
  size_t shutdown_ticks = seconds_to_ticks(shutdown_time); // hold ZMP at center for 1s at end
  size_t double_support_ticks = seconds_to_ticks(double_support_time); // .05s of double support each step
  size_t single_support_ticks = seconds_to_ticks(single_support_time); // .70s of single support each step



  HuboPlus hplus(hubofile);

  my_kbody = hplus.kbody;

  const double& l6 = hplus.footAnkleDist;
  


  size_t step_ticks = double_support_ticks + single_support_ticks;
  size_t total_ticks = startup_ticks + n_steps * step_ticks + shutdown_ticks;

  double zmp_dt = 1.0/TRAJ_FREQ_HZ; // delta t for ZMP preview controller
  
  TimeStamp t0 = TimeStamp::now();

  ZmpPreview preview(zmp_dt, com_height, num_lookahead, zmp_R);

  //////////////////////////////////////////////////////////////////////
  // fill up buffers with zmp reference and foot info

  // some space to hold zmp reference, foot trajectory, and stance info
  Eigen::ArrayXd zmprefX(total_ticks);
  Eigen::ArrayXd zmprefY(total_ticks);
  Eigen::ArrayXd footz(total_ticks);
  Eigen::ArrayXXd footx(total_ticks,2);
  std::vector<stance_t> stance(total_ticks);

  size_t cur_tick = 0;

  double p = 0;
  double p_next = fy-zmpy;

  double cur_foot_x[2] = { 0, 0 };
  
  stance_t s = DOUBLE_LEFT; // start in double left phase

  // set variables for startup phase
  for (size_t i=0; i<startup_ticks; ++i) {
    stance[cur_tick] = s;
    footz(cur_tick) = 0;
    for (int f=0; f<2; ++f) { footx(cur_tick,f) = cur_foot_x[f]; }
    zmprefX(cur_tick) = zmpx; // set zmprefX x to zero
    zmprefY(cur_tick++) = p; // set zmprefY y to zero
  }

  // set variables for walking phase
  for (size_t k=0; k<n_steps; ++k) {

    p = p_next; // set zmp y to sway distance
    p_next = -p_next; // set next zmp y to sway distance in opposite direction
    
    // set variables for double support phase during walking
    for (size_t i=0; i<double_support_ticks; ++i) {
      stance[cur_tick] = s; // set stance phase for current tick
      
      footz(cur_tick) = 0; // set stance foot z to zero for current tick
      
      for (int f=0; f<2; ++f) {
        footx(cur_tick,f) = cur_foot_x[f]; // set both feet constant here
      }
      
      int stance_foot = stance_foot_table[s];
      zmprefX(cur_tick) =  cur_foot_x[stance_foot] + zmpx; // set zmprefX x for current tick to sway distance
      zmprefY(cur_tick++) = p; // set zmprefY y for current tick to sway distance
    }
    
    
    s = next_stance_table[s]; // switch to next (single support) phase
    // set variables for single support phase during walking
    
    int stance_foot = stance_foot_table[s];
    int swing_foot = 1-stance_foot;
    real fdist = cur_foot_x[stance_foot] + fx - cur_foot_x[swing_foot];

    
    for (size_t i=0; i<single_support_ticks; ++i) {
      
      double u = double(i)/double(single_support_ticks-1); // compute portion of step phase
      double a = u*2*M_PI; // angle in radians the circle has rotated
      
      stance[cur_tick] = s; // set stance mode for current tick
      int stance_foot = stance_foot_table[s];
      int swing_foot = 1-stance_foot;
      
      footz(cur_tick) = 0.5*fz*(1 - cos(a)); // the Y component of a cycloid
      
      footx(cur_tick, stance_foot) = cur_foot_x[stance_foot];
      
      
      footx(cur_tick, swing_foot) = cur_foot_x[swing_foot] + fdist*(a - sin(a))/(2*M_PI); // the X component of a cycloid
      zmprefX(cur_tick) = cur_foot_x[stance_foot] + zmpx; // set zmprefX x for current tick
      zmprefY(cur_tick++) = p; // set zmprefY y for current tick to sway distance
    }
    s = next_stance_table[s]; // go to next stance phase
    stance_foot = stance_foot_table[s];
    cur_foot_x[stance_foot] += fdist;
  }

  // set variables for shutdown phase after finishing walking
  p = 0;
  for (size_t i=0; i<shutdown_ticks; ++i) {
    stance[cur_tick] = s;
    footz(cur_tick) = 0;
    for (int f=0; f<2; ++f) { footx(cur_tick,f) = cur_foot_x[f]; }
    zmprefX(cur_tick) = 0.5*(cur_foot_x[0]+cur_foot_x[1]) + zmpx; // set zmprefX x to 0
    zmprefY(cur_tick++) = p; // set zmprefY y to 0
  }

  assert(cur_tick == total_ticks);

  TimeStamp t1 = TimeStamp::now();

  //////////////////////////////////////////////////////////////////////
  // We are now running our ZMP preview controller to generate a COM 
  // trajectory

  Eigen::Vector3d X(0.0, 0.0, 0.0);
  Eigen::Vector3d Y(0.0, 0.0, 0.0);
  double eX = 0;
  double eY = 0;
  double pX = 0;
  double pY = 0;

  Eigen::MatrixXd comX(total_ticks,3); // x,dx,ddx
  Eigen::ArrayXd zmpX(total_ticks); // x of zmp

  Eigen::MatrixXd comY(total_ticks,3); // y,dy,ddy
  Eigen::ArrayXd zmpY(total_ticks); // y of zmp

  // generate COM position for each tick using zmp preview update
  for (size_t i=0; i<total_ticks; ++i) {
    comX.row(i) = X.transpose();
    comY.row(i) = Y.transpose();
    zmpX(i) = pX;
    zmpY(i) = pY;

    pX = preview.update(X, eX, zmprefX.block(i, 0, total_ticks-i, 1));
    pY = preview.update(Y, eY, zmprefY.block(i, 0, total_ticks-i, 1));
  }
  
  TimeStamp t2 = TimeStamp::now();

  //validateCOMTraj(comX, comY);
 
  //////////////////////////////////////////////////////////////////////
  // fill up a full body trajectory using COM & footstep info
  // generated above.
  //
  // NOTE: I am assuming a fixed world frame independent of the robot
  // in this section of the code, which is kind of a big nono

  Transform3Array xforms;
  const KinBody& kbody = hplus.kbody;
  HuboPlus::KState state;
  
  state.body_pos = vec3(0, 0, 0.85); // body position
  state.body_rot = quat(); //body rotation (straight forward)
  
  state.jvalues.resize(kbody.joints.size(), 0.0); // initialize joint values
  
  real deg = M_PI/180; // conversion from degress to radians
  const JointLookup& jl = hplus.jl;
  state.jvalues[jl("LSR")] =  15*deg;
  state.jvalues[jl("RSR")] = -15*deg;
  state.jvalues[jl("LSP")] =  20*deg;
  state.jvalues[jl("RSP")] =  20*deg;
  state.jvalues[jl("LEP")] = -40*deg;
  state.jvalues[jl("REP")] = -40*deg;

  kbody.transforms(state.jvalues, xforms);

  Transform3 desired[4];
  vec3 desiredCom;

  // set initial positions of the feet  
  desired[0].setTranslation(vec3(0, fy, 0)); // left foot
  desired[1].setTranslation(vec3(0, -fy, 0)); // right foot

  HuboPlus::IKMode mode[4] = { 
    HuboPlus::IK_MODE_FIXED,
    HuboPlus::IK_MODE_FIXED,
    HuboPlus::IK_MODE_FIXED,
    HuboPlus::IK_MODE_FIXED,
  };

  //TrajVector traj;
  
  // loop thru trajectory and make full-body joint trajectory
  desiredCom = vec3(comX(0), comY(0), com_height+l6);

  for (size_t i=0; i<total_ticks; ++i) {
    // loop through stance and swing foot tables
    int stance_foot = stance_foot_table[stance[i]];
    int swing_foot = swing_foot_table[stance[i]];
    
    vec3 old[2];
    for (int f=0; f<2; ++f) { old[f] = desired[f].translation(); }

    // if we're in double support mode
    if (swing_foot < 0) {

      mode[0] = HuboPlus::IK_MODE_SUPPORT;
      mode[1] = HuboPlus::IK_MODE_SUPPORT;
      
      desired[0].setTranslation(vec3(footx(i, 0),  fy, 0));
      desired[1].setTranslation(vec3(footx(i, 1), -fy, 0));

      // else if we're in swing mode
    } else {


      mode[swing_foot] = HuboPlus::IK_MODE_WORLD;
      mode[stance_foot] = HuboPlus::IK_MODE_SUPPORT;
      
      const double sy[2] = { 1, -1 };

      double z = footz[i]; // create swing foot z-direction variable

      vec3 newstance = vec3(footx(i,stance_foot), sy[stance_foot]*fy, 0);
      vec3 newswing = vec3(footx(i, swing_foot), sy[swing_foot]*fy, z);
  
      // set stance foot desired position to (0, fixed-pos from center, 0)
      desired[stance_foot].setTranslation(newstance);
      
      // set swing foot desired position equal to location set above for tick #i
      desired[swing_foot].setTranslation(newswing);

    }
    
    for (int f=0; i && f<2; ++f) {
      assert( (old[f] - desired[f].translation()).norm() < 0.05 );
    }
    
    // set com desired position
    vec3 desiredComTmp(desiredCom);
    desiredCom = vec3(comX(i), comY(i), com_height+l6);

    if ((desiredCom - desiredComTmp).norm() > .01 ) {
      assert( 0 && "Bad desiredCom" );
    }

    bool ikvalid[4];
    
    bool ok = hplus.comIK( state, desiredCom, desired, mode, 
			   HuboPlus::noGlobalIK(), xforms, 
			   com_ik_ascl, 0, ikvalid );

    if (!ok) {

      Transform3 actual[4];
      vec3 dp[4], dq[4];

      kbody.transforms(state.jvalues, xforms);

      vec3 actualCom = state.xform() * kbody.com(xforms);

      for (int i=0; i<4; ++i) {

	if (mode[i] != HuboPlus::IK_MODE_FIXED &&
	    mode[i] != HuboPlus::IK_MODE_FREE) {
	  
	  actual[i] = kbody.manipulatorFK(xforms, i);
	  if (mode[i] == HuboPlus::IK_MODE_WORLD || 
	      mode[i] == HuboPlus::IK_MODE_SUPPORT) {
	    actual[i] = state.xform() * actual[i];
	  }

	  deltaTransform(desired[i], actual[i], dp[i], dq[i]);

	  // TODO: allow feet far away from ground to have IK failures

	}

      }
 
      if (!ok) {
	std::cerr << "IK FAILURE!\n\n";
	std::cerr << "  body:        " << state.xform() << "\n";
	std::cerr << "  desired com: " << desiredCom << "\n";
	std::cerr << "  actual com:  " << actualCom << "\n\n";
	for (int i=0; i<4; ++i) { 
	  if (mode[i] != HuboPlus::IK_MODE_FIXED &&
	      mode[i] != HuboPlus::IK_MODE_FREE) {
	    std::cerr << "  " << kbody.manipulators[i].name << ":\n";
	    std::cerr << "    valid:   " << ikvalid[i] << "\n";
	    std::cerr << "    desired: " << desired[i] << "\n";
	    std::cerr << "    actual:  " << actual[i] << "\n";
	    std::cerr << "    dp:      " << dp[i] << " with norm " << dp[i].norm() << "\n";
	    std::cerr << "    dq:      " << dq[i] << " with norm " << dq[i].norm() << "\n";
	    std::cerr << "\n";
	  }
	}
	exit(1);
      }

    }

    zmp_traj_element_t cur;
    memset(&cur, 0, sizeof(cur));

    for (size_t hi=0; hi<hplus.huboJointOrder.size(); ++hi) {
      size_t ji = hplus.huboJointOrder[hi];
      if (ji != size_t(-1)) {
	      cur.angles[hi] = state.jvalues[ji];
      }
      cur.stance = stance[i];
    }

    Transform3 stanceInv = desired[stance_foot].inverse();

    vec3 zmp(zmprefX(i), zmprefY(i), 0);
    zmp = stanceInv * zmp;

    cur.zmp[0] = zmp[0];
    cur.zmp[1] = zmp[1];

    vec3 forces[2], torques[2];

    hplus.computeGroundReaction( vec3(comX(i,0), comY(i,0), com_height),
				 vec3(comX(i,2), comY(i,2), 0),
				 desired, mode,
				 forces, torques );

    for (int f=0; f<2; ++f) {
      for (int axis=0; axis<3; ++axis) {
	cur.forces[f][axis] = forces[f][axis];
	cur.torque[f][axis] = torques[f][axis]; // TODO: FIXME: pluralization WTF?
      }
    }

    for (int deriv=0; deriv<3; ++deriv) {
      vec3 cv(comX(i,deriv), comY(i,deriv), deriv==0 ? com_height : 0);
      if (deriv == 0) {
	cv = stanceInv * cv;
      } else {
	cv = stanceInv.rotFwd() * cv;
      }
      for (int axis=0; axis<3; ++axis) {
	cur.com[axis][deriv] = cv[axis];
      }
    }

    traj.push_back(cur);

  }

  //validateOutputData(traj);

  //////////////////////////////////////////////////////////////////////
  // ach_put goes after this line

  TimeStamp t3 = TimeStamp::now();

  double sim_time = total_ticks * zmp_dt;
  double d0 = (t1-t0).toDouble();
  double d1 = (t2-t1).toDouble();
  double d2 = (t3-t2).toDouble();
  double total_time = (t3-t0).toDouble();

  std::cerr << "Generated reference trajectories in:      " << d0 << "s\n";
  std::cerr << "Generated center of mass trajectories in: " << d1 << "s\n";
  std::cerr << "Generated full-body trajectories in:      " << d2 << "s\n";
  std::cerr << "Total time:                               " << total_time << "s\n";
  std::cerr << "Execution time:                           " << sim_time << "s\n";
  std::cerr << "Speedup over real-time:                   " << sim_time/total_time << "\n";

  //////////////////////////////////////////////////////////////////////
  // now deal with ach

#ifdef HAVE_HUBO_ACH

  if (use_ach) {

    ach_channel_t zmp_chan;
    ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );

    zmp_traj_t trajectory;
    memset( &trajectory, 0, sizeof(trajectory) );

    int N;
    if( (int)traj.size() > MAX_TRAJ_SIZE )
      N = MAX_TRAJ_SIZE;
    else
      N = (int)traj.size();

    trajectory.count = N;
    for(int i=0; i<N; i++)
      memcpy( &(trajectory.traj[i]), &(traj[i]), sizeof(zmp_traj_element_t) );

    ach_put( &zmp_chan, &trajectory, sizeof(trajectory) );
    fprintf(stdout, "Message put\n");

  }

#endif


  return 0;

}


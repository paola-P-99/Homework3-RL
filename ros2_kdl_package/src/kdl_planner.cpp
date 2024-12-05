#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    
    // Se il raggio è maggiore di zero, significa che è una traiettoria circolare
    if (_trajRadius > 0.0) {
        trajRadius_ = _trajRadius;  // Traiettoria circolare
    } else {
        trajRadius_ = 0.0;  // Traiettoria rettilinea
    }
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}


void KDLPlanner::trapezoidal_vel(double time, double accDuration_, double &s, double &s_dot, double &s_ddot)
{

  double s_c_ddot = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_);

  if(time <= accDuration_)
  {
    s = 0.5*s_c_ddot*std::pow(time,2);
    s_dot = s_c_ddot*time;
    s_ddot = s_c_ddot;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    s = s_c_ddot*accDuration_*(time-accDuration_/2);
    s_dot = s_c_ddot*accDuration_;
    s_ddot = 0.0;
  }
  else
  {
    s = 1-0.5*s_c_ddot*std::pow(trajDuration_-time,2);
    s_dot = s_c_ddot*(trajDuration_-time);
    s_ddot = -s_c_ddot;
  }

}

void KDLPlanner::cubic_polynomial(double time, double &s, double &s_dot, double &s_ddot)
{
  // Coefficients computed imposing boundary conditions
  double a0 = 0; 
  double a1 = 0; 
  double a2 = 3 / std::pow(trajDuration_,2);
  double a3 = - 2 / std::pow(trajDuration_, 3);
    
  // s, s_dot, s_ddot computation
  s = a3 * std::pow(time, 3) + a2 * std::pow(time, 2) + a1 * time + a0; 
  s_dot = 3 * a3 * std::pow(time, 2) + 2 * a2 * time + a1; 
  s_ddot = 6 * a3 * time + 2 * a2;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}


trajectory_point KDLPlanner::compute_circular_trajectory_with_trapez(double time)
{
  double s, s_dot, s_ddot;
  trapezoidal_vel(time, accDuration_, s, s_ddot, s_ddot); // definition of the curvilinear abscissa

  trajectory_point traj;

  traj.pos[0] = trajInit_[0];
  traj.pos[1] = trajInit_[1] - trajRadius_*cos(2*M_PI*s);
  traj.pos[2] = trajInit_[2] - trajRadius_*sin(2*M_PI*s);

  traj.vel[0] = 0;
  traj.vel[1] = 2*M_PI*trajRadius_*s_dot*sin(2*M_PI*s);
  traj.vel[2] = -2*M_PI*trajRadius_*s_dot*cos(2*M_PI*s);

  traj.acc[0] = 0;
  traj.acc[1] = 2*M_PI*trajRadius_*s_ddot*sin(2*M_PI*s) + std::pow(2*M_PI*trajRadius_*s_dot,2)*cos(2*M_PI*s);
  traj.acc[2] = -2*M_PI*trajRadius_*s_ddot*cos(2*M_PI*s) + std::pow(2*M_PI*trajRadius_*s_dot,2)*sin(2*M_PI*s);

  return traj;
}

trajectory_point KDLPlanner::compute_rectilinear_trajectory_with_trapez(double time)
{
  double s, s_dot, s_ddot;
  trapezoidal_vel(time, accDuration_, s, s_dot, s_ddot);

  trajectory_point traj;

  Eigen::Vector3d p0=trajInit_;
  Eigen::Vector3d pf=trajEnd_;
  Eigen::Vector3d dp=pf-p0;

  traj.pos= p0+s*dp;
  traj.vel=dp*s_dot;
  traj.acc=dp*s_ddot;

  return traj;

}

trajectory_point KDLPlanner::compute_circular_trajectory_with_cubic(double time)
{
  double s, s_dot, s_ddot;
  cubic_polynomial(time, s, s_dot, s_ddot); // definition of the curvilinear abscissa

  trajectory_point traj;

  traj.pos[0] = trajInit_[0];
  traj.pos[1] = trajInit_[1] - trajRadius_*cos(2*M_PI*s);
  traj.pos[2] = trajInit_[2] - trajRadius_*sin(2*M_PI*s);

  traj.vel[0] = 0;
  traj.vel[1] = 2*M_PI*trajRadius_*s_dot*sin(2*M_PI*s);
  traj.vel[2] = -2*M_PI*trajRadius_*s_dot*cos(2*M_PI*s);

  traj.acc[0] = 0;
  traj.acc[1] = 2*M_PI*trajRadius_*s_ddot*sin(2*M_PI*s) + std::pow(2*M_PI*trajRadius_*s_dot,2)*cos(2*M_PI*s);
  traj.acc[2] = -2*M_PI*trajRadius_*s_ddot*cos(2*M_PI*s) + std::pow(2*M_PI*trajRadius_*s_dot,2)*sin(2*M_PI*s);

  return traj;
}

trajectory_point KDLPlanner::compute_rectilinear_trajectory_with_cubic(double time)
{
  double s, s_dot, s_ddot;
  cubic_polynomial(time, s, s_dot, s_ddot);

  trajectory_point traj;

  Eigen::Vector3d p0=trajInit_;
  Eigen::Vector3d pf=trajEnd_;
  Eigen::Vector3d dp=pf-p0;

  traj.pos= p0+s*dp;
  traj.vel=dp*s_dot;
  traj.acc=dp*s_ddot;

  return traj;

}


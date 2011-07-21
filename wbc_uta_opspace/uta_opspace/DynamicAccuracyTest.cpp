/*
 * Shared copyright notice and LGPLv3 license statement.
 *
 * Copyright (C) 2011 The Board of Trustees of The Leland Stanford Junior University. All rights reserved.
 * Copyright (C) 2011 University of Texas at Austin. All rights reserved.
 *
 * Authors: Roland Philippsen (Stanford) and Luis Sentis (UT Austin)
 *          http://cs.stanford.edu/group/manips/
 *          http://www.me.utexas.edu/~hcrl/
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#include "DynamicAccuracyTest.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;

#ifndef M_PI
#define M_PI 3.1415926535897932846
#endif

namespace uta_opspace {


  DynamicAccuracyTest::
  DynamicAccuracyTest(std::string const & name)
    : Skill(name),
      posture_task_(0),
      goal_eepos_task_(0),
      eepos_goal_(0),
      eevel_goal_(0),
      center_position_(0),
      radius_(-1),
      omega_(-1),
      start_time_(0),
      offset_(Vector::Zero(3))
  {
    ros::start();
    start_time_ = ros::Time::now();
    declareSlot("posture", &posture_task_);
    declareSlot("goal_eepos", &goal_eepos_task_);

    declareParameter("center_position", &center_position_);
    declareParameter("radius", &radius_);
    declareParameter("omega", &omega_);
    
  }
  
  
  Status DynamicAccuracyTest::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    
    //////////////////////////////////////////////////
    // init parameter channels
    
    // XXXX to do: could read the name of the parameter from a
    // parameter (also for other tasks)
    
    eepos_goal_ = goal_eepos_task_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);
    if ( ! eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in goal position task";
      return st;
    }

    // XXXX not being used currently
    eevel_goal_ = goal_eepos_task_->lookupParameter("goalvel", PARAMETER_TYPE_VECTOR);
    if ( ! eevel_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in goal velocity task";
      return st;
    }
    
    //////////////////////////////////////////////////
    // init task tables
     
    goal_task_table_.push_back(goal_eepos_task_);
    goal_task_table_.push_back(posture_task_);
    
    //////////////////////////////////////////////////
    // check / initialize task parameters
    
   
    
    if (3 != center_position_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing center_position parameter";
      return st;
    }

    if (radius_ < 0) {
      st.ok = false;
      st.errstr = "invalid or missing radius parameter";
      return st;
    }

    if (omega_ < 0) {
      st.ok = false;
      st.errstr = "invalid or missing omega parameter";
      return st;
    }
    updateCurGoal(ros::Time::now());
    st = eepos_goal_->set(cur_goal_);
    if ( ! st) {
      return st;
    }

    return st;
  }

  void DynamicAccuracyTest::
  updateCurGoal(ros::Time current_time) {
    ros::Duration t_d = current_time - start_time_;
    double t = t_d.toSec();
    double theta = omega_*t;
    offset_[0] = radius_*cos(theta);
    //shouldn't ever be less than 0...
    while(theta > 2*M_PI) {
      theta -= 2*M_PI;
    }
    bool quad3 = theta>M_PI && theta <3*M_PI/2;
    if (theta<M_PI/2 || quad3) {
      offset_[2] = sqrt(-pow(radius_,2)-2*pow(offset_[0],2)+radius_*sqrt(8*pow(offset_[0],2)+pow(radius_,2)))/2;
    }
    else {
      offset_[2] = -sqrt(-pow(radius_,2)-2*pow(offset_[0],2)+radius_*sqrt(8*pow(offset_[0],2)+pow(radius_,2)))/2;
    }

    cur_goal_ = Vector(center_position_ + offset_);
  }
  
  
  Status DynamicAccuracyTest::
  update(Model const & model)
  {
    Status st;
    
    for (size_t ii(0); ii < goal_task_table_.size(); ++ii) {
      st = goal_task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
  
    updateCurGoal(ros::Time::now());
    st = eepos_goal_->set(cur_goal_);
    if ( ! st) {
      return st;
    }  
    Vector actual(goal_eepos_task_->getActual());
    Vector jpos(model.getState().position_);
    //Prints actual, current goal, jacobian
    ROS_ERROR("%f %f %f %f %f %f",actual[0],actual[1],actual[2],cur_goal_[0],cur_goal_[1],cur_goal_[2]);    
    return st;
  }
  
  
  Skill::task_table_t const * DynamicAccuracyTest::
  getTaskTable()
  {
      return &goal_task_table_;
  }
  
  
  Status DynamicAccuracyTest::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    Status ok;
    return ok;
  }
  
  
  void DynamicAccuracyTest::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os, title, prefix);
    os << prefix << "Current end effector goal is: \n"; 
      for (int i=0; i<3; i++) {
	os << prefix << cur_goal_[i] << " ";
      }
    os<< prefix << "\n";
   os << prefix << "Current end effector position is: \n"; 
      for (int j=0; j<3; j++) {
	os << prefix << goal_eepos_task_->getActual()[j] << " ";
      }
    os<< prefix << "\n";
   os << prefix << "Current offset is: \n"; 
      for (int k=0; k<3; k++) {
	os << prefix << offset_[k] << " ";
      }
    os<< prefix << "\n";
 
  }
  
}

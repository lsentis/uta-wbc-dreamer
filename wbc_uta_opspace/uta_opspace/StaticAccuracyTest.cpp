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

#include "StaticAccuracyTest.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace uta_opspace {


  StaticAccuracyTest::
  StaticAccuracyTest(std::string const & name)
    : Skill(name),
      state_(STATE_AWAY),
      last_state_(STATE_AWAY),
      goal_eepos_task_(0),
      goal_eepos_goal_(0),
      away_eepos_goal_(0),
      hold_time_(-1),
      current_time_(0)
  {
    ros::start();
    declareSlot("posture", &posture_task_);

    declareSlot("goal_eepos", &goal_eepos_task_);
    declareParameter("goal_position", &goal_position_);

    declareSlot("away_eepos", &away_eepos_task_);
    declareParameter("away_position", &away_position_);

    declareParameter("hold_time", &hold_time_);
  }
  
  
  Status StaticAccuracyTest::
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
    
    goal_eepos_goal_ = goal_eepos_task_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);
    if ( ! goal_eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in goal position task";
      return st;
    }
    
    away_eepos_goal_ = away_eepos_task_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);
    if ( ! away_eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in away position task";
      return st;
    }
    
    //////////////////////////////////////////////////
    // init task tables
     
    goal_task_table_.push_back(goal_eepos_task_);
    goal_task_table_.push_back(posture_task_);
    
    away_task_table_.push_back(away_eepos_task_);
    away_task_table_.push_back(posture_task_);
    
    //////////////////////////////////////////////////
    // check / initialize task parameters
    
    
    if (0 > hold_time_) {
      hold_time_ = 5000;
    }
    
    if (3 != goal_position_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing goal_position parameter";
      return st;
    }
    if (3 != away_position_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing away_position parameter";
      return st;
    }

    //////////////////////////////////////////////////
    // set those goals which we need at the very beginning, others get
    // filled in as we transition through the state machine... for
    // which we need lazy init flags.
    
    st = goal_eepos_goal_->set(goal_position_);
    if ( ! st) {
      return st;
    }
    st = away_eepos_goal_->set(away_position_);
    if ( ! st) {
      return st;
    }

    
    //////////////////////////////////////////////////
    // ready to rock
    
    state_ = STATE_AWAY;
    return st;
  }
  
  
  Status StaticAccuracyTest::
  update(Model const & model)
  {
    Status st;
    
    for (size_t ii(0); ii < goal_task_table_.size(); ++ii) {
      st = goal_task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
    for (size_t ii(0); ii < away_task_table_.size(); ++ii) {
      st = away_task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
    
    current_time_++;
    last_state_ = state_;
    switch (state_) {

    case STATE_AWAY:
      if (current_time_>=hold_time_) {
	current_time_=0;
	state_ = STATE_GOAL;
	}
    break;

    case STATE_GOAL:
      if (current_time_>=hold_time_) {
	ROS_ERROR("%f %f %f",goal_eepos_task_->getActual()[0],goal_eepos_task_->getActual()[1],goal_eepos_task_->getActual()[2]);
	current_time_=0;
	state_ = STATE_AWAY;
	}
    break;
      
    default:
      return Status(false, "invalid state");
    }
    
    return st;
  }
  
  
  Skill::task_table_t const * StaticAccuracyTest::
  getTaskTable()
  {
    switch (state_) {
    case STATE_GOAL:
      return &goal_task_table_;
    case STATE_AWAY:
      return &away_task_table_;
    }
    return 0;
  }
  
  
  Status StaticAccuracyTest::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    Status ok;
    return ok;
  }
  
  
  void StaticAccuracyTest::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os, title, prefix);
    os << prefix << "Current end effector position is: \n";
    switch (state_) {
    case STATE_GOAL: 
      for (int i=0; i<3; i++) {
	os << prefix << goal_eepos_task_->getActual()[i] << " ";
      }
      break;
    case STATE_AWAY:
      for (int i=0; i<3; i++) {
	os << prefix << away_eepos_task_->getActual()[i] << " ";
      }
    break;
    }
    os<< prefix << "\n";
   os << prefix << "Current end effector goal is: \n";
    switch (state_) {
    case STATE_GOAL: 
      for (int i=0; i<3; i++) {
	os << prefix << goal_position_[i] << " ";
      }
      break;
    case STATE_AWAY:
      for (int i=0; i<3; i++) {
	os << prefix << away_position_[i] << " ";
      }
    break;
    }
    os<< prefix << "\n";
 
  }
  
}

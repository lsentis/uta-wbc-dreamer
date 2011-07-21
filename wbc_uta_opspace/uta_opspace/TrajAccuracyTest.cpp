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

#include "TrajAccuracyTest.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;

namespace uta_opspace {


  TrajAccuracyTest::
  TrajAccuracyTest(std::string const & name)
    : Skill(name),
      posture_task_(0),
      goal_eepos_task_(0),
      eepos_goal_(0),
      hold_time_(-1),
      state_(STATE_ORIGIN),
      elapsed_(0)
  {
    declareSlot("posture", &posture_task_);
    declareSlot("eepos", &goal_eepos_task_);

    declareParameter("eepos_goal", &goal_);
    declareParameter("eepos_origin", &origin_);
    declareParameter("hold_time", &hold_time_);
    
  }
  
  
  Status TrajAccuracyTest::
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
    
    eepos_goal_ = goal_eepos_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in goal position task";
      return st;
    }
    
    //////////////////////////////////////////////////
    // init task tables
     
    goal_task_table_.push_back(goal_eepos_task_);
    goal_task_table_.push_back(posture_task_);
    
    //////////////////////////////////////////////////
    // check / initialize task parameters
    
   
    
    if (3 != goal_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing goal parameter";
      return st;
    }

    if (3 != origin_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing origin parameter";
      return st;
    }

    if (hold_time_ < 0) {
      st.ok = false;
      st.errstr = "invalid or missing hold time parameter";
      return st;
    }

    for (size_t ii(0); ii < goal_task_table_.size(); ++ii) {
      st = goal_task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }
    st = eepos_goal_->set(origin_);
    if ( ! st) {
      return st;
    }


    return st;
  }

  
  
  Status TrajAccuracyTest::
  update(Model const & model)
  {
    Status st;
    
    for (size_t ii(0); ii < goal_task_table_.size(); ++ii) {
      st = goal_task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }
    elapsed_++;
    if (elapsed_ > hold_time_) {
      elapsed_ = 0;
      switch (state_) {
      case STATE_ORIGIN:
	state_=STATE_GOAL;
	st = eepos_goal_->set(goal_);
	if ( ! st) {
	  return st;
	}  
	break;
      case STATE_GOAL:
	state_=STATE_ORIGIN;
	st = eepos_goal_->set(origin_);
	if ( ! st) {
	  return st;
	}  
	break;
      }
      
    }
 
    Vector actual(goal_eepos_task_->getActual());
    //Prints actual
    if (state_ == STATE_GOAL) {
      ROS_ERROR("%f %f %f",actual[0],actual[1],actual[2]);    
    }
    return st;
  }
  
  
  Skill::task_table_t const * TrajAccuracyTest::
  getTaskTable()
  {
      return &goal_task_table_;
  }
  
  
  Status TrajAccuracyTest::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    Status ok;
    return ok;
  }
  
  
  void TrajAccuracyTest::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os, title, prefix);
 
  }
  
}

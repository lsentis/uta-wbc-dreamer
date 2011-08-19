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

#include "JointTest.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace uta_opspace {


  JointTest::
  JointTest(std::string const & name)
    : Skill(name),
      jpos_task_(0),
      jpos_goal_(0),
      amplitude_(-1),
      time_(0)
  {
    declareSlot("jpos", &jpos_task_);
    declareParameter("jpos", &jpos_);
    declareParameter("selection", &selection_);
    declareParameter("amplitude", &amplitude_);
  }
  
  
  Status JointTest::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
    }
    
    //////////////////////////////////////////////////
    // init parameter channels
    jpos_goal_ = jpos_task_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);
    if ( ! jpos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in shake position task";
      return st;
    }
    
 
    //////////////////////////////////////////////////
    // init task tables
    
    task_table_.push_back(jpos_task_);
 
    
    //////////////////////////////////////////////////
    // check / initialize task parameters
    
    if (7 != jpos_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing jpos parameter";
      return st;
    }
    
    if (0 > amplitude_) {
      amplitude_ = 5.0 * M_PI / 180.0;
    }
    
    if (7 != selection_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing selection parameter";
      return st;
    }
 
    //////////////////////////////////////////////////
    // set those goals which we need at the very beginning, others get
    // filled in as we transition through the state machine... for
    // which we need lazy init flags.
    
    st = jpos_goal_->set(jpos_);
    if ( ! st) {
      return st;
    }

    return st;
  }
  
  
  Status JointTest::
  update(Model const & model)
  {
    Status st;
    time_++;
    
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }   
    
    st = jpos_goal_->set(Vector(jpos_ + selection_*amplitude_*sin(time_/1000)));
    if ( ! st) {
      return st;
    }   
      
    return st;
  }
  
  
  Skill::task_table_t const * JointTest::
  getTaskTable()
  {
      return &task_table_;
  }
  
  
  Status JointTest::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    Status ok;
    return ok;
  }
  
  
  void JointTest::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os, title, prefix);
    os << prefix << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";

      for (size_t ii(0); ii < task_table_.size(); ++ii) {
	task_table_[ii]->dump(os, "", prefix + "  ");
      }
   

  }
  
}

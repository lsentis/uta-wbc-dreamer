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

#include "CircleTest.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace uta_opspace {


  CircleTest::
  CircleTest(std::string const & name)
    : Skill(name),
      posture_task_(0),
      goal_eepos_task_(0)
  {
    declareSlot("eepos", &goal_eepos_task_);
    declareSlot("posture", &posture_task_);
  }
  
  
  Status CircleTest::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
      }

    // init task tables

    goal_task_table_.push_back(goal_eepos_task_);
    goal_task_table_.push_back(posture_task_);

    return st;
  }
  
  
  Status CircleTest::
  update(Model const & model)
  {
    Status st;
    
    for (size_t ii(0); ii < goal_task_table_.size(); ++ii) {
      st = goal_task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }    
    return st;
  }
  
  
  Skill::task_table_t const * CircleTest::
  getTaskTable()
  {
     return &goal_task_table_;
  }
  
  
  Status CircleTest::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    Status ok;
    return ok;
  }
  
  
  void CircleTest::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os, title, prefix);
 
  }
  
}

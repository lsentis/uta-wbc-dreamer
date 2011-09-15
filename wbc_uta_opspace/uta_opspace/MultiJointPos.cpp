
#include "MultiJointPos.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;

namespace uta_opspace {


  MultiJointPos::
  MultiJointPos(std::string const & name)
    : Skill(name),
      jpos_task_(0),
      jpos_(Vector::Zero(7)),
      threshold_(-1),
      vel_threshold_(-1),
      cur_row_(0)
    {
      declareSlot("jpos", &jpos_task_);
      declareParameter("jpos", &jpos_);
      declareParameter("threshold", &threshold_);
      declareParameter("vel_threshold", &vel_threshold_);
  }

 Status MultiJointPos::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }


     if (0 > threshold_) {
       threshold_ = 0.08;
     }

     if (0 > vel_threshold_) {
       vel_threshold_ = 0.08;
     }

    jpos_goal_ = jpos_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);

    if ( ! jpos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in joint";
      return st;
    }

    task_table_.push_back(jpos_task_);
     
   for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   Vector cur_jpos(Vector::Zero(7));
   for(int jj=0; jj<7; jj++) {
     cur_jpos[jj] = jpos_[7*cur_row_+jj];
   }

   st = jpos_goal_->set(cur_jpos);
   if (!st) { return st; }

   return st;
  }

 Status MultiJointPos::
  update(Model const & model)
  {
    Status st;    
    Vector cur_jpos(Vector::Zero(7));
    Vector delta;
    Vector v_delta;

    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->update(model);
      if ( ! st) { return st; }
    }
    for(int ii=0; ii<7; ii++) {
      cur_jpos[ii] = jpos_[7*cur_row_+ii];
    }
    
    delta = cur_jpos - model.getState().position_;
    v_delta = model.getState().velocity_;
    
    if (delta.norm() < threshold_ && v_delta.norm() < vel_threshold_) {
	if (cur_row_ < (jpos_.rows()/7)-1) {
	  ++cur_row_;
	  for(int jj=0; jj<7; jj++) {
	    cur_jpos[jj] = jpos_[7*cur_row_+jj];
	  }
	  st = jpos_goal_->set(cur_jpos);
	  if (! st) { return st; } 
	}
      }
 
      return st;
    }
    
    Skill::task_table_t const * MultiJointPos::
      getTaskTable()
    {
      return &task_table_;
    }
    
    
  Status MultiJointPos::
      checkJStarSV(Task const * task, Vector const & sv)
    {
      /*
	if (sv.rows() != 3) {
	return Status(false, "dimension mismatch");
	}
    if (sv[2] < task->getSigmaThreshold()) {
    return Status(false, "singular");
    }*/
      Status ok;
    return ok;
  }


  void MultiJointPos::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os,title,prefix);
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      task_table_[ii]->dump(os, "", prefix + "  ");
    }
  }
}

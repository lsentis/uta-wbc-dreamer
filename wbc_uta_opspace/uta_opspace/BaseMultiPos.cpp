
#include "BaseMultiPos.hpp"
#include <opspace/task_library.hpp>
#include <jspace/constraint_library.hpp>

using boost::shared_ptr;

namespace uta_opspace {


  BaseMultiPos::
  BaseMultiPos(std::string const & name)
    : Skill(name),
      ee_task_(0),
      ee_pos_(Vector::Zero(3)),
      threshold_(-1),
      vel_threshold_(-1),
      cur_row_(0),
      forward_(true)
    {
      declareSlot("eepos", &ee_task_);
      declareParameter("eepos", &ee_pos_);
      declareParameter("threshold", &threshold_);
      declareParameter("vel_threshold", &vel_threshold_);
  }

 Status BaseMultiPos::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }
    
    if (0 != ee_pos_.rows()%3){
      st.ok = false;
      st.errstr = "wrong number of rows in ee_pos vector";
      return st;
    }

    if (0 > threshold_) {
      threshold_ = 0.08;
    }
    
    if (0 > vel_threshold_) {
      vel_threshold_ = 0.08;
    }

    ee_goal_ = ee_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);

    if ( ! ee_goal_ ) {
      st.ok = false;
      st.errstr = "no appropriate goalpos parameter in ee_task";
      return st;
    }

    task_table_.push_back(ee_task_);
     
   for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   Vector cur_eepos(Vector::Zero(3));
   for (size_t ii(0); ii < 3; ++ii) {
     cur_eepos[ii] = ee_pos_[3*cur_row_+ii];
   }

   st = ee_goal_->set(cur_eepos);
   if (!st) { return st; }

   return st;
  }

 Status BaseMultiPos::
  update(Model const & model)
  {
    Status st;    
    Vector cur_eepos(Vector::Zero(3));
    Vector delta;
    Vector v_delta;

    size_t ndof(model.getUnconstrainedNDOF());

    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->update(model);
      if ( ! st) { return st; }
    }
    for(int ii=0; ii<3; ii++) {
      cur_eepos[ii] = ee_pos_[3*cur_row_+ii];
    }
    
    delta = cur_eepos - ee_task_->getActual();
    jspace::Constraint* constraint = model.getConstraint();
    if (constraint) {
      jspace::State fullState(model.getNDOF(),model.getNDOF(),6);
      constraint->getFullState(model.getState(),fullState);
      v_delta = ee_task_->getJacobian()*fullState.velocity_;
    }
    else {
      v_delta = ee_task_->getJacobian()*model.getState().velocity_;
    }
    
    if (delta.norm() < threshold_ && v_delta.norm() < vel_threshold_) {
      if(forward_) {
	if (cur_row_ < (ee_pos_.rows()/3)-1) {
	  ++cur_row_;
	  for(size_t jj(0); jj<3; ++jj) {
	    cur_eepos[jj] = ee_pos_[3*cur_row_+jj];
	  }
	  st = ee_goal_->set(cur_eepos);
	  if (! st) { return st; } 
	}
	else { forward_ = false; }
      }
      else {
	if (cur_row_ > 0) {
	  --cur_row_;
	  for(size_t jj(0); jj<3; ++jj) {
	    cur_eepos[jj] = ee_pos_[3*cur_row_+jj];
	  }
	  st = ee_goal_->set(cur_eepos);
	  if (! st) { return st; } 
	}
	else { forward_ = true; }
      }
    }
 
      return st;
    }
    
    Skill::task_table_t const * BaseMultiPos::
      getTaskTable()
    {
      return &task_table_;
    }
    
    
  Status BaseMultiPos::
      checkJStarSV(Task const * task, Vector const & sv)
    {
      Status ok;
    return ok;
  }


  void BaseMultiPos::
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

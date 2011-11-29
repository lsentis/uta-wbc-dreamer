
#include "CartMultiPos.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;

namespace uta_opspace {


  CartMultiPos::
  CartMultiPos(std::string const & name)
    : Skill(name),
      ee_task_(0),
      posture_(0),
      ee_pos_(Vector::Zero(3)),
      posture_pos_(Vector::Zero(9)),
      threshold_(-1),
      vel_threshold_(-1),
      cur_row_(0),
      forward_(true)
    {
      declareSlot("eepos", &ee_task_);
      declareSlot("posture", &posture_);
      declareParameter("eepos", &ee_pos_);
      declareParameter("posture", &posture_pos_);
      declareParameter("threshold", &threshold_);
      declareParameter("vel_threshold", &vel_threshold_);
  }

 Status CartMultiPos::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }
    
    if (0 != ee_pos_.rows()%3){
      st.ok = false;
      st.errstr = "wrong number of rows in ee_pos vector";
      return st;
    }

    if (0 != posture_pos_.rows()%model.getUnconstrainedNDOF() ){
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

    if ( ! ee_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in ee_task";
      return st;
    }

    posture_goal_ = posture_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);

    if ( ! posture_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in posture";
      return st;
    }

    task_table_.push_back(ee_task_);
    task_table_.push_back(posture_);
     
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

   size_t ndof(model.getUnconstrainedNDOF());

   Vector cur_posture(Vector::Zero(ndof));
   for(size_t jj(0); jj<7; jj++) {
     cur_posture[jj] = posture_pos_[ndof*cur_row_+jj];
   }

   st = posture_goal_->set(cur_posture);
   if (!st) { return st; }

   return st;
  }

 Status CartMultiPos::
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
    if (model.getNDOF() != model.getUnconstrainedNDOF()) {
      Vector qdot(Vector::Zero(model.getNDOF()));
      for (size_t ii(0); ii<model.getNDOF(); ++ii) {
	if (ii<2) {
	  qdot[ii] = model.getState().velocity_[ii];
	}
	else {
	  qdot[ii] = model.getState().velocity_[ii-1];
	}
      }
      v_delta = ee_task_->getJacobian()*qdot;
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
	  Vector cur_posture(Vector::Zero(ndof));
	  for(size_t ii(0); ii<7; ++ii) {
	    cur_posture[ii] = posture_pos_[ndof*cur_row_+ii];
	  }
	  st = posture_goal_->set(cur_posture);
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
	  Vector cur_posture(Vector::Zero(ndof));
	  for(size_t ii(0); ii<7; ++ii) {
	    cur_posture[ii] = posture_pos_[ndof*cur_row_+ii];
	  }
	  st = posture_goal_->set(cur_posture);
	  if (! st) { return st; } 
	}
	else { forward_ = true; }
      }
    }
 
      return st;
    }
    
    Skill::task_table_t const * CartMultiPos::
      getTaskTable()
    {
      return &task_table_;
    }
    
    
  Status CartMultiPos::
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


  void CartMultiPos::
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

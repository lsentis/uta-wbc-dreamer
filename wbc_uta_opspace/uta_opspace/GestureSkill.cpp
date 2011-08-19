
#include "GestureSkill.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace uta_opspace {


  GestureSkill::
  GestureSkill(std::string const & name)
    : Skill(name),
      state_(STATE_OP_),
      eepos_task_(0),
      eeori_task_(0),
      posture_task_(0),
      joint_task_(0),
      eepos_goal_(0),
      eeori_goal_x_(0),
      eeori_goal_y_(0),
      eeori_goal_z_(0),
      posture_goal_(0),
      joint_goal_(0),
      eepos_(Vector::Zero(3)),
      eeori_x_(Vector::Zero(3)),
      eeori_y_(Vector::Zero(3)),
      eeori_z_(Vector::Zero(3)),
      posture_(Vector::Zero(7)),
      joint_pos_(Vector::Zero(7)),
      threshold_(-1),
      cur_row_(0),
      hold_count_(Vector::Zero(1)),
      count_(0),
      vel_threshold_(-1)
    {
    declareSlot("eepos", &eepos_task_);
    declareSlot("eeori", &eeori_task_);
    declareSlot("posture", &posture_task_);
    declareSlot("joint", &joint_task_);
    declareParameter("eepos", &eepos_);
    declareParameter("eeori_x", &eeori_x_);
    declareParameter("eeori_y", &eeori_y_);
    declareParameter("eeori_z", &eeori_z_);
    declareParameter("posture", &posture_);
    declareParameter("joint_positions", &joint_pos_);
    declareParameter("threshold", &threshold_);
    declareParameter("hold_count", &hold_count_);
    declareParameter("vel_threshold", &vel_threshold_);
  }

 Status GestureSkill::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }

    if (eepos_.rows()%3 != 0  || eepos_.rows() != eeori_x_.rows() ||
	eeori_x_.rows() != eeori_y_.rows() || eeori_y_.rows() != eeori_z_.rows()
	|| eeori_z_.rows()/3 != posture_.rows()/7) {
      return Status(false,"Mismatched number of rows in input vectors");
    }

     if (0 > threshold_) {
       threshold_ = 0.08;
     }
     if (0 > vel_threshold_) {
       vel_threshold_ = 0.03;
     }

    eepos_goal_ = eepos_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! eepos_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in eepos task";
      return st;
    }

    eeori_goal_x_ = eeori_task_->lookupParameter("goal_x", PARAMETER_TYPE_VECTOR);
    if ( ! eeori_goal_x_) {
      st.ok = false;
      st.errstr = "no appropriate x goal parameter in eeori task";
      return st;
    }

    eeori_goal_y_ = eeori_task_->lookupParameter("goal_y", PARAMETER_TYPE_VECTOR);
    if ( ! eeori_goal_y_) {
      st.ok = false;
      st.errstr = "no appropriate y goal parameter in eeori task";
      return st;
    }

    eeori_goal_z_ = eeori_task_->lookupParameter("goal_z", PARAMETER_TYPE_VECTOR);
    if ( ! eeori_goal_z_) {
      st.ok = false;
      st.errstr = "no appropriate z goal parameter in eeori task";
      return st;
    }

    posture_goal_ = posture_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! posture_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in posture";
      return st;
    }

    joint_goal_ = joint_task_->lookupParameter("trjgoal", PARAMETER_TYPE_VECTOR);
    if ( ! joint_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in joint";
      return st;
    }

    op_task_table_.push_back(eepos_task_);   
    op_task_table_.push_back(eeori_task_);
    op_task_table_.push_back(posture_task_);

    //    joint_task_table_.push_back(joint_task_);
        joint_task_table_.push_back(posture_task_);
     
   for (size_t ii(0); ii < op_task_table_.size(); ++ii) {
      st = op_task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   for (size_t ii(0); ii < joint_task_table_.size(); ++ii) {
      st = joint_task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   Vector cur_eepos(Vector::Zero(3));
   Vector cur_eeori_x(Vector::Zero(3));
   Vector cur_eeori_y(Vector::Zero(3));
   Vector cur_eeori_z(Vector::Zero(3));
   for(int ii=0; ii<3; ii++) {
     cur_eepos[ii] = eepos_[3*cur_row_+ii];
     cur_eeori_x[ii] = eeori_x_[3*cur_row_+ii];
     cur_eeori_y[ii] = eeori_y_[3*cur_row_+ii];
     cur_eeori_z[ii] = eeori_z_[3*cur_row_+ii];
   }
   Vector cur_posture(Vector::Zero(7));
   Vector cur_joint_pos(Vector::Zero(7));
   for(int jj=0; jj<7; jj++) {
     cur_posture[jj] = posture_[7*cur_row_+jj];
     cur_joint_pos[jj] = joint_pos_[7*cur_row_+jj];
   }

   st = eepos_goal_->set(cur_eepos);
   if (! st) { return st; }
   st = eeori_goal_x_->set(cur_eeori_x);
   if (! st) { return st; }
   st = eeori_goal_y_->set(cur_eeori_y);
   if (! st) { return st; }
   st = eeori_goal_z_->set(cur_eeori_z);
   if (! st) { return st; }
   st = posture_goal_->set(cur_posture);
   if (! st) { return st; }   
   st = joint_goal_->set(cur_joint_pos);
   if (!st) { return st; }

   return st;
  }

 Status GestureSkill::
  update(Model const & model)
  {
    Status st;    
    Vector cur_eepos(Vector::Zero(3));
    Vector delta;
    Vector op_vel;

    switch (state_) {
    case STATE_OP_:
      for (size_t ii(0); ii < op_task_table_.size(); ++ii) {
	st = op_task_table_[ii]->update(model);
	if ( ! st) { return st; }
      }
      for(int ii=0; ii<3; ii++) {
	cur_eepos[ii] = eepos_[3*cur_row_+ii];
      }

      delta = cur_eepos - eepos_task_->getActual();
      op_vel = eepos_task_->getJacobian()*model.getState().velocity_;
      
      if (delta.norm() < threshold_ && op_vel.norm() < vel_threshold_) {
	count_++;
	if (cur_row_ < (eepos_.rows()/3)-1 && count_>hold_count_[cur_row_]) {
	  count_=0;
	  ++cur_row_;
	  Vector cur_eeori_x(Vector::Zero(3));
	  Vector cur_eeori_y(Vector::Zero(3));
	  Vector cur_eeori_z(Vector::Zero(3));
	  for(int ii=0; ii<3; ii++) {
	    cur_eepos[ii] = eepos_[3*cur_row_+ii];
	    cur_eeori_x[ii] = eeori_x_[3*cur_row_+ii];
	    cur_eeori_y[ii] = eeori_y_[3*cur_row_+ii];
	    cur_eeori_z[ii] = eeori_z_[3*cur_row_+ii];
	  }
	  Vector cur_posture(Vector::Zero(7));
	  for(int jj=0; jj<7; jj++) {
	    cur_posture[jj] = posture_[7*cur_row_+jj];
	  }
	  
	  st = eepos_goal_->set(cur_eepos);
	  if (! st) { return st; }
	  st = eeori_goal_x_->set(cur_eeori_x);
	  if (! st) { return st; }
	  st = eeori_goal_y_->set(cur_eeori_y);
	  if (! st) { return st; }
	  st = eeori_goal_z_->set(cur_eeori_z);
	  if (! st) { return st; }
	  st = posture_goal_->set(cur_posture);
	  if (! st) { return st; } 
	}
	else  {
	  // cur_row_=0;
	  // state_=STATE_JOINT_;	  
	}
      }
      break;
    case STATE_JOINT_:
	for (size_t ii(0); ii < joint_task_table_.size(); ++ii) {
	  st = joint_task_table_[ii]->update(model);
	  if ( ! st) { return st; }
	}
	Vector cur_pos(Vector::Zero(7));
	for(int ii=0; ii<7; ii++) {
	  cur_pos[ii] = joint_pos_[7*cur_row_+ii];
	}

	Vector joint_diff( cur_pos - posture_task_->getActual());

	if (joint_diff.norm() < threshold_*5) {
	  if (cur_row_ < (joint_pos_.rows()/7)-1) {
	    ++cur_row_;
	    for(int ii=0; ii<7; ii++) {
	      cur_pos[ii] = joint_pos_[7*cur_row_+ii];
	    }
	    //st = joint_goal_->set(cur_pos);
	    if (! st) { return st; } 
	  }
	}

	    st = posture_goal_->set(cur_pos);
	break;
      }
      
      
      return st;
    }
    
    Skill::task_table_t const * GestureSkill::
      getTaskTable()
    {
      switch (state_) {
      case STATE_OP_:
	return &op_task_table_;
	break;
      case STATE_JOINT_:
	return &joint_task_table_;
	break;
	
    }
    }
    
    
    Status GestureSkill::
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


  void GestureSkill::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os,title,prefix);
    switch (state_) {
    case STATE_OP_:
      for (size_t ii(0); ii < op_task_table_.size(); ++ii) {
	op_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      break;
    case STATE_JOINT_:
      for (size_t ii(0); ii < joint_task_table_.size(); ++ii) {
	joint_task_table_[ii]->dump(os, "", prefix + "  ");
      }
      break;
    }
 }
}

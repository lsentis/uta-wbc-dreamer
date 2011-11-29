
#include "TaskOriPostureSkill.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace uta_opspace {


  TaskOriPostureSkill::
  TaskOriPostureSkill(std::string const & name)
    : Skill(name),
      eepos_task_(0),
      eeori_task_(0),
      posture_task_(0),
      eepos_goal_(0),
      eeori_goal_x_(0),
      eeori_goal_y_(0),
      eeori_goal_z_(0),
      posture_goal_(0),
      eepos_(Vector::Zero(3)),
      eeori_x_(Vector::Zero(3)),
      eeori_y_(Vector::Zero(3)),
      eeori_z_(Vector::Zero(3)),
      posture_(Vector::Zero(7))
    {
    declareSlot("eepos", &eepos_task_);
    declareSlot("eeori", &eeori_task_);
    declareSlot("posture", &posture_task_);
    declareParameter("eepos", &eepos_);
    declareParameter("eeori_x", &eeori_x_);
    declareParameter("eeori_y", &eeori_y_);
    declareParameter("eeori_z", &eeori_z_);
    declareParameter("posture", &posture_);
  }

 Status TaskOriPostureSkill::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) {
      return st;
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
      st.errstr = "no appropriate x  goal parameter in eeori task";
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

   
    task_table_.push_back(eeori_task_);
    task_table_.push_back(eepos_task_);
    task_table_.push_back(posture_task_);


     if (7 != posture_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing posture parameter";
      return st;
    }


   for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

     st = eepos_goal_->set(eepos_);
     if (! st) {
       return st;
     }

    st = eeori_goal_x_->set(eeori_x_);
     if (! st) {
       return st;
     }

    st = eeori_goal_y_->set(eeori_y_);
     if (! st) {
       return st;
     }

    st = eeori_goal_z_->set(eeori_z_);
     if (! st) {
       return st;
     }
     
     st = posture_goal_->set(posture_);
     if (! st) {
       return st;
       }     

 

     return st;
  }

 Status TaskOriPostureSkill::
  update(Model const & model)
  {
    Status st;

    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }

    return st;
  }
     
  Skill::task_table_t const * TaskOriPostureSkill::
  getTaskTable()
  {
    return &task_table_;
  }

 
  Status TaskOriPostureSkill::
  checkJStarSV(Task const * task, Vector const & sv)
  {/*
    if (sv.rows() != 3) {
      return Status(false, "dimension mismatch");
    }
    if (sv[2] < task->getSigmaThreshold()) {
      return Status(false, "singular");
      }*/
    Status ok;
    return ok;
  }


  void TaskOriPostureSkill::
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


#include "WriteSkill.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;


namespace uta_opspace {


  WriteSkill::
  WriteSkill(std::string const & name)
    : Skill(name),
      eepos_task_(0),
      eeori_task_(0),
      posture_task_(0),
      eepos_goal_(0),
      eeori_goal_x_(0),
      eeori_goal_y_(0),
      eeori_goal_z_(0),
      posture_goal_(0),
      letters_(""),
      letter_centers_(0),
      eeori_x_(0),
      eeori_y_(0),
      eeori_z_(0),
      posture_(0),
      letter_manager_(0),
      threshold_(-1),
      cur_row_(0),
      scale_(-1),
      file_name_("")
    {
    declareSlot("eepos", &eepos_task_);
    declareSlot("eeori", &eeori_task_);
    declareSlot("posture", &posture_task_);
    declareParameter("letters", &letters_);
    declareParameter("letter_centers", &letter_centers_);
    declareParameter("eeori_x", &eeori_x_);
    declareParameter("eeori_y", &eeori_y_);
    declareParameter("eeori_z", &eeori_z_);
    declareParameter("posture", &posture_);
    declareParameter("threshold", &threshold_);
    declareParameter("scale", &scale_);
    declareParameter("file_name", &file_name_);
  }

 Status WriteSkill::
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

    task_table_.push_back(eepos_task_);   
    task_table_.push_back(eeori_task_);
    task_table_.push_back(posture_task_);
    
    if ( letter_centers_.rows() / 3 != letters_.length()) {
      st.ok = false;
      st.errstr = "number of letters not the same as points in the letter centers vector";
	return st;
    }
    
     if (7 != posture_.rows()) {
      st.ok = false;
      st.errstr = "invalid or missing posture parameter";
      return st;
    }

     if (0 > threshold_) {
       threshold_ = 0.04;
     }

     if (0 >= scale_) {
       scale_ = 1.0;
     }
     
     if (file_name_ == "") {
       st.ok = false;
       st.errstr = "no letter database file name provided";
       return st;
       }
     
   for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->init(model);
      if ( ! st) {
	return st;
      }
    }

   letter_manager_ = new LetterManager(threshold_,scale_);
   st = letter_manager_->load(file_name_);
   if ( ! st) {
     return st;
   }

  Vector firstCenter(Vector::Zero(3));
   for(int ii=0; ii<firstCenter.size(); ii++) {
     firstCenter[ii] = letter_centers_[3*cur_row_+ii];
   }

   st = letter_manager_->idIs(letters_.substr(0,1), firstCenter);
   if ( ! st) {
     return st;
   }

   Vector firstPoint(Vector::Zero(3));
   letter_manager_->curPoint(Vector((eepos_task_->getActual() - firstCenter)/scale_ ),firstPoint);

   st = eepos_goal_->set(Vector(firstCenter + scale_ * firstPoint));
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

 Status WriteSkill::
  update(Model const & model)
  {
    Status st;    
    
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->update(model);
      if ( ! st) {
	return st;
      }
    }

    Vector curPoint(Vector::Zero(3));
    Vector curCenter(Vector::Zero(3));
    for(int ii=0; ii<curCenter.size(); ii++) {
       curCenter[ii] = letter_centers_[3*cur_row_+ii];
    }

    if ( letter_manager_->curPoint(Vector((eepos_task_->getActual() - curCenter)/scale_ ),curPoint) ) {
      if (cur_row_< (letter_centers_.rows()/3) - 1 ) {
	++cur_row_;
	st = letter_manager_->idIs(letters_.substr(cur_row_,1),curCenter);
	if ( ! st) {
	  return st;
	}
	for(int ii=0; ii<curCenter.size(); ii++) {
	  curCenter[ii] = letter_centers_[3*cur_row_+ii];
	}
	letter_manager_->curPoint(Vector((eepos_task_->getActual() - curCenter)/scale_),curPoint);
      }
    }

    st = eepos_goal_->set(Vector(curCenter + scale_ * curPoint));
    return st;
  }
     
  Skill::task_table_t const * WriteSkill::
  getTaskTable()
  {
    return &task_table_;
  }

 
  Status WriteSkill::
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


  void WriteSkill::
  dbg(std::ostream & os,
      std::string const & title,
      std::string const & prefix) const
  {
    Skill::dbg(os,title,prefix);
    for (size_t ii(0); ii < task_table_.size(); ++ii) {
	task_table_[ii]->dump(os, "", prefix + "  ");
      }
    os << prefix << "Writing : " + letters_ + "\n";
    os << prefix << "Current letter is : " + letter_manager_->id() + "\n";
 }
}

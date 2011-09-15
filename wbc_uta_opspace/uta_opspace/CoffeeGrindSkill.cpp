
#include "CoffeeGrindSkill.hpp"
#include <opspace/task_library.hpp>

using boost::shared_ptr;

namespace uta_opspace {


  CoffeeGrindSkill::
  CoffeeGrindSkill(std::string const & name)
    : Skill(name),
      ee_task_(0),
      posture_(0),
      init_pos_(Vector::Zero(3)),
      posture_pos_(Vector::Zero(7)),
      radius_(-1),
      omega_(-1),
      init_(true)
    {
      declareSlot("eepos", &ee_task_);
      declareSlot("posture", &posture_);
      declareParameter("init_pos", &init_pos_);
      declareParameter("posture", &posture_pos_);
      declareParameter("pos_div", &pos_div_);
      declareParameter("radius", &radius_);
      declareParameter("omega", &omega_);
  }

 Status CoffeeGrindSkill::
  init(Model const & model)
  {
    Status st(Skill::init(model));
    if ( ! st) { return st; }
    
    if (7 != init_pos_.rows()){
      st.ok = false;
      st.errstr = "wrong number of rows in init_pos vector";
      return st;
    }

    if (0 != posture_pos_.rows()%7  ){
      st.ok = false;
      st.errstr = "wrong number of rows in posture_pos vector";
      return st;
    }

    if (pos_div_.rows() != posture_pos_.rows() - 7) {
      st.ok = false;
      st.errstr = "wrong number of rows in div vector";
      return st;
    }

    if (0 > radius_) {
      radius_ = 0.1;
    }
    
    if (0 > omega_) {
      omega_ = 1.0;
    }

    ee_goal_ = ee_task_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);

    if ( ! ee_goal_) {
      st.ok = false;
      st.errstr = "no appropriate goal parameter in ee_task";
      return st;
    }

    posture_goal_ = posture_->lookupParameter("goalpos", PARAMETER_TYPE_VECTOR);

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

   st = ee_goal_->set(init_pos_);
   if (!st) { return st; }

   Vector cur_posture(Vector::Zero(7));
   for ( size_t ii(0); ii<pos_div_.rows(); ++ii) {
     if(init_pos_[2]<pos_div_[ii]) {
       for (size_t jj(ii); jj<7; ++jj) {
	 cur_posture[jj-ii] = pos_div_[jj];
       }
       break;
     }
   }
   st = posture_goal_->set(cur_posture);
   if (!st) { return st; }

   return st;
  }

  Status CoffeeGrindSkill::
  update(Model const & model)
  {
    Status st;    
    Vector cur_posture(Vector::Zero(7));

    for (size_t ii(0); ii < task_table_.size(); ++ii) {
      st = task_table_[ii]->update(model);
      if ( ! st) { return st; }
    }

    for ( size_t ii(0); ii<pos_div_.rows(); ++ii) {
      if(ee_task_->getActual()[2]<pos_div_[ii]) {
	for (size_t jj(ii); jj<7; ++jj) {
	  cur_posture[jj-ii] = pos_div_[jj];
	}
	break;
      }
    }
    st = posture_goal_->set(cur_posture);
    if (!st) { return st; }
    
    if(init_) {
      t++;
      if (t*0.0025>10.0) {
	init_ = false;
      }
    }
    else {
      Vector center(ee_task_->getActual());
      center[0] -= radius_*sin(omega_*(t-1)*0.0025);
      center[1] -= radius_*cos(omega_*(t-1)*0.0025);
      
      Vector curgoal(center);
      curgoal[0] += radius_*sin(omega_*t*0.0025);
      curgoal[1] += radius_*cos(omega_*t*0.0025);
      st = ee_goal_->set(curgoal);
    }
 
    return st;
  }
    
  Skill::task_table_t const * CoffeeGrindSkill::
  getTaskTable()
  {
    return &task_table_;
  }
  
  
  Status CoffeeGrindSkill::
  checkJStarSV(Task const * task, Vector const & sv)
  {
    Status ok;
    return ok;
  }
  

  void CoffeeGrindSkill::
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

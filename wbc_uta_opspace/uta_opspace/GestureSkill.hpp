
#ifndef UTA_OPSPACE_GESTURE_SKILL_HPP
#define UTA_OPSPACE_GESTURE_SKILL_HPP

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

namespace uta_opspace {
  
  using namespace opspace;
  
  
  class GestureSkill
    : public Skill
  {
  public:
    GestureSkill(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
    void dbg(std::ostream & os,
	     std::string const & title,
	     std::string const & prefix) const;
    
  protected:

    enum {
      STATE_OP_,
      STATE_JOINT_
    } state_;
    
    CartPosTrjTask * eepos_task_;
    OrientationTask * eeori_task_;
    JPosTrjTask * posture_task_;
    JPosTrjTask * joint_task_;
    task_table_t op_task_table_;
    task_table_t joint_task_table_;
    
    Parameter * eepos_goal_;
    Parameter * eeori_goal_x_;
    Parameter * eeori_goal_y_;
    Parameter * eeori_goal_z_;
    Parameter * posture_goal_;
    Parameter * joint_goal_;

    Vector eepos_;
    Vector eeori_x_;
    Vector eeori_y_;
    Vector eeori_z_;
    Vector posture_;
    Vector joint_pos_;
    double threshold_;
    int cur_row_;
    Vector hold_count_;
    int count_;
    double vel_threshold_;

  };
  
}

#endif // UTA_OPSPACE_GESTURE_SKILL_HPP

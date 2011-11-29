
#ifndef UTA_OPSPACE_TASK_ORI_POSTURE_SKILL_HPP
#define UTA_OPSPACE_TASK_ORI_POSTURE_SKILL_HPP

#include <opspace/Skill.hpp>
#include <opspace/task_library.hpp>

namespace uta_opspace {
  
  using namespace opspace;
  
  
  class TaskOriPostureSkill
    : public Skill
  {
  public:
    TaskOriPostureSkill(std::string const & name);
    
    virtual Status init(Model const & model);
    virtual Status update(Model const & model);
    virtual task_table_t const * getTaskTable();
    virtual Status checkJStarSV(Task const * task, Vector const & sv);
    
    void dbg(std::ostream & os,
	     std::string const & title,
	     std::string const & prefix) const;
    
  protected:
    
    CartPosTrjTask * eepos_task_;
    OrientationTask * eeori_task_;
    JPosTrjTask * posture_task_;
    task_table_t task_table_;
    
    Parameter * eepos_goal_;
    Parameter * eeori_goal_x_;
    Parameter * eeori_goal_y_;
    Parameter * eeori_goal_z_;
    Parameter * posture_goal_;

    Vector eepos_;
    Vector eeori_x_;
    Vector eeori_y_;
    Vector eeori_z_;
    Vector posture_;

  };
  
}

#endif // UTA_OPSPACE_TASK_ORI_POSTURE_SKILL_HPP

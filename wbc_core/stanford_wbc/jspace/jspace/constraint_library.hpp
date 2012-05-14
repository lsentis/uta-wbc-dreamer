#ifndef CONSTRAINT_LIBRARY_H
#define CONSTRAINT_LIBRARY_H

#include <jspace/Constraint.hpp>

namespace jspace {

  /*
    Constraints for the Dreamer Base only
   */

  class Dreamer_Base
    : public Constraint {
  public:
    Dreamer_Base();
    Status updateJc(Model const & model);
    void getFullState(State const & state,
		      State & fullState);
  protected:
    double wheel_radius_;
    Vector x_prev;
    Vector jpos_prev;
    Matrix Jxyz;
    Matrix Jabg;
    Matrix Ainv;
    
  };


  /*
    Constraint for the Dreamer Torso with 
    only the torso and arm together
   */

  class Dreamer_Torso
    : public Constraint {
  public:
    Dreamer_Torso();
    void getFullState(State const & state,
		      State & fullState);
    
  };

  /*
    Constraints for the Full Dreamer
   */

  class Dreamer_Full
    : public Constraint {
  public:
    Dreamer_Full();
    Status updateJc(Model const & model);
    void getFullState(State const & state,
		      State & fullState);
  protected:
    double wheel_radius_;
    Vector x_prev;
    Vector jpos_prev;
    Matrix Jxyz;
    Matrix Jabg;
    Matrix Ainv;		   
  };

}

#endif

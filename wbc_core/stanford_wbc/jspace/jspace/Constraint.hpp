#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <jspace/Status.hpp>
#include <jspace/State.hpp>

namespace jspace {
  
  class Model;

  class Constraint {
  public:
    Constraint() {}
    virtual Status getU(Matrix & U);
    virtual Status updateJc(Model const & model);
    virtual Status getJc(Matrix & Jc);
    virtual Status getJcBar(Matrix const Ainv,
			    Matrix & JcBar);
    virtual Status getNc(Matrix const Ainv,
			 Matrix & Nc);
    virtual Status getUNc(Matrix const Ainv,
			  Matrix & UNc);
    virtual Status getUNcBar(Matrix const Ainv,
			     Matrix & UNcBar);
    virtual void getFullState(State const & state,
			      State & fullState) = 0;

  protected:
    Matrix U_;
    Matrix Jc_;
    double sigmaThreshold_;
  };
}

#endif

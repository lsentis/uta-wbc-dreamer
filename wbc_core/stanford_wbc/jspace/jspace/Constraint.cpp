#include <jspace/pseudo_inverse.hpp>
#include <jspace/Constraint.hpp>

namespace jspace {

  Status Constraint::getU(Matrix & U) 
  {
    U = U_;
    Status ok;
    return ok;
  }

  Status Constraint::updateJc(Model const & model) {
    Status ok;
    return ok;
  }
  
  Status Constraint::getJc(Matrix & Jc)
  { 
    Jc = Jc_;
    Status ok;
    return ok;
  }
  
  Status Constraint::getJcBar(Matrix const Ainv,
				 Matrix & JcBar) {
    Matrix lambda;
    pseudoInverse(Jc_ * Ainv * Jc_.transpose(),
		  sigmaThreshold_,
		  lambda, 0);
    JcBar = Ainv * Jc_.transpose() * lambda;
    
    Status ok;
    return ok;
  }

  Status Constraint::getNc(Matrix const Ainv,
			      Matrix & Nc) {
    Matrix JcBar;
    if (!getJcBar(Ainv,JcBar)) {
      return Status(false,"failed to get JcBar");
    }
    
    Nc = Matrix::Identity(Ainv.rows(),Ainv.cols())-JcBar*Jc_;
    
    Status ok;
    return ok;
  }

  Status Constraint::getUNc(Matrix const Ainv,
			       Matrix & UNc) {
    Matrix Nc;
    if (!getNc(Ainv,Nc)) {
      return Status(false,"failed to get Nc");
    }
    
    UNc = U_*Nc;
    
    Status ok;
    return ok;
  }
  
  Status Constraint::getUNcBar(Matrix const Ainv,
				  Matrix & UNcBar) {
    Matrix UNc;
    if (!getUNc(Ainv,UNc)) {
      return Status(false,"failed to get UNc");
    }
    
    Matrix lambda;
    pseudoInverse(UNc * Ainv * UNc.transpose(),
		    sigmaThreshold_,
		  lambda, 0);
    UNcBar = Ainv * UNc.transpose() * lambda;
    
    Status ok;
    return ok;
  }

}

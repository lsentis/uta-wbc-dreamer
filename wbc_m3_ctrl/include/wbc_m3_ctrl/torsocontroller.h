#ifndef WBC_M3_CTRL_TORSO_CONTROLLER_H
#define WBC_M3_CTRL_TORSO_CONTROLLER_H

#include <jspace/State.hpp>
#include <jspace/Status.hpp>

namespace wbc_m3_ctrl {
  	
  class TorsoController
  {
  public:
   jspace::Status init(jspace::State const & state) {
	state_ = state;
	goalpos_ = jspace::Vector::Zero(2);
	kp_ = 500*jspace::Vector::Ones(2);
	kp_[1] = 1200;
	kd_ = jspace::Vector::Ones(2);
	jspace::Status ok;
	return ok;
   }
    
    jspace::Status update(jspace::State const & state) {
	state_ = state;
	jspace::Status ok;
	return ok;
    }
    
    jspace::State getState() { return state_; }

    jspace::Status computeCommand(jspace::Vector & command) {
	command = getCommand();
	jspace::Status ok;
	return ok;
    }
    
    jspace::Vector getCommand() {
	return kp_.cwise()*(goalpos_ - state_.position_) - kd_.cwise()*state_.velocity_;
   }
    
  protected:
    jspace::State state_;
    jspace::Vector goalpos_;
    jspace::Vector kp_;
    jspace::Vector kd_;
  };
}
#endif 

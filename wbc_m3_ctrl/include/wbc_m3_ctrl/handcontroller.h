#ifndef WBC_M3_CTRL_HAND_CONTROLLER_H
#define WBC_M3_CTRL_HAND_CONTROLLER_H

#include <jspace/State.hpp>
#include <jspace/Status.hpp>

namespace wbc_m3_ctrl {
  	
  class HandController
  {
  public:
    jspace::Status init(jspace::State const & state) {
	state_ = state;
	goalpos_ = jspace::Vector::Zero(5);
	goalpos_[0] = 0;
	kp_ = 20*jspace::Vector::Ones(5);
	kd_ = jspace::Vector::Ones(5);
	Status ok;
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
	//1st slot commands position of the thumb, last 4 torques of the fingers
	jspace::Vector command(5);
	command[0] = goalpos_[0];
	for (size_t ii(1); ii<5; ++ii) {
	   command[ii] = kp_[ii]*(goalpos_[ii] - state_.position_[ii]) - kd_[ii]*state_.velocity_[ii];
	}
	return command;
    }
    
  protected:
    jspace::State state_;
    jspace::Vector goalpos_;
    jspace::Vector kp_; 
    jspace::Vector kd_;
  };
}
#endif 

#ifndef WBC_M3_CTRL_HEAD_CONTROLLER_H
#define WBC_M3_CTRL_HEAD_CONTROLLER_H

#include <jspace/State.hpp>
#include <jspace/Status.hpp>

namespace wbc_m3_ctrl {
  	
  class HeadController
  {
  public:
    jspace::Status init(jspace::State const & state) {
	state_ = state;
	goalpos_ = jspace::Vector::Zero(12);
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
	return goalpos_;
   }
    
  protected:
    jspace::State state_;
    jspace::Vector goalpos_;
  };
}
#endif 

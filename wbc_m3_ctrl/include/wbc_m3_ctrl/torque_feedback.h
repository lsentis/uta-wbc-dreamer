#ifndef WBC_M3_CTRL_TORQUE_FEEDBACK_H
#define WBC_M3_CTRL_TORQUE_FEEDBACK_H

#include <jspace/State.hpp>

using namespace jspace;

namespace wbc_m3_ctrl {
  	
  class TorqueFeedback
  {
  public:
    TorqueFeedback() {
      kp_ = Vector::Zero(3);
    }
    Vector computeFeedback(Vector & command, Vector & tau_sensor) {
      return command + kp_.cwise()*(command - tau_sensor);
    }
    
  protected:
    Vector kp_;
  };
}
#endif 

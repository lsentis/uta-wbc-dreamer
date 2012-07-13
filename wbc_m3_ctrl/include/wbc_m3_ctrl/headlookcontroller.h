#ifndef WBC_M3_CTRL_HEAD_LOOK_CONTROLLER_H
#define WBC_M3_CTRL_HEAD_LOOK_CONTROLLER_H

#include <jspace/State.hpp>
#include <jspace/Status.hpp>

namespace wbc_m3_ctrl {
  	
  class HeadLookController
  {
  public:
    jspace::Status init(jspace::State const & state,
			jspace::Vector const & kp) {
	
    	actual_x_ = jspace::Vector::Zero(3);		
        actual_y_ = jspace::Vector::Zero(3);
        actual_z_ = jspace::Vector::Zero(3);
        jac_ = jspace::Matrix::Identity(kp.rows(),state.position_.rows());
      
      state_ = state;
      kp_ = kp;
      jspace::Status ok;
      return ok;
    }
    
    jspace::Status update(jspace::State const & state) {
      state_ = state;
      double q1 = state_.position_[0];
      double q2 = state_.position_[1];
      double q3 = state_.position_[2];
      double q4 = state_.position_[3];
      double q5 = state_.position_[4];
      double q6 = state_.position_[5];


      //update jacobian and actual vectors
      actual_x_[0] =  ((-(-cos(q1)*sin(q2)*sin(q3)+sin(q1)*cos(q3))*sin(q4)+cos(q1)*cos(q2)*cos(q4))*cos(q5)+(-(-cos(q1)*sin(q2)*sin(q3)+sin(q1)*cos(q3))*cos(q4)-cos(q1)*cos(q2)*sin(q4))*sin(q5))*cos(q6)-(cos(q1)*sin(q2)*cos(q3)+sin(q1)*sin(q3))*sin(q6);
      actual_x_[1]= ((-cos(q2)*sin(q3)*sin(q4)+sin(q2)*cos(q4))*cos(q5)+(-cos(q2)*sin(q3)*cos(q4)-sin(q2)*sin(q4))*sin(q5))*cos(q6)+cos(q2)*cos(q3)*sin(q6);
      actual_x_[2] =  ((-(-sin(q1)*sin(q2)*sin(q3)-cos(q1)*cos(q3))*sin(q4)+sin(q1)*cos(q2)*cos(q4))*cos(q5)+(-(-sin(q1)*sin(q2)*sin(q3)-cos(q1)*cos(q3))*cos(q4)-sin(q1)*cos(q2)*sin(q4))*sin(q5))*cos(q6)-(sin(q1)*sin(q2)*cos(q3)-cos(q1)*sin(q3))*sin(q6);
      actual_y_[0]  = ((-(-cos(q1)*sin(q2)*sin(q3)+sin(q1)*cos(q3))*sin(q4)+cos(q1)*cos(q2)*cos(q4))*cos(q5)+(-(-cos(q1)*sin(q2)*sin(q3)+sin(q1)*cos(q3))*cos(q4)-cos(q1)*cos(q2)*sin(q4))*sin(q5))*sin(q6)-(cos(q1)*sin(q2)*cos(q3)+sin(q1)*sin(q3))*cos(q6);
      actual_y_[1] = ((-cos(q2)*sin(q3)*sin(q4)+sin(q2)*cos(q4))*cos(q5)+(-cos(q2)*sin(q3)*cos(q4)-sin(q2)*sin(q4))*sin(q5))*sin(q6)+cos(q2)*cos(q3)*cos(q6);
      actual_y_[2] = ((-(-sin(q1)*sin(q2)*sin(q3)-cos(q1)*cos(q3))*sin(q4)+sin(q1)*cos(q2)*cos(q4))*cos(q5)+(-(-sin(q1)*sin(q2)*sin(q3)-cos(q1)*cos(q3))*cos(q4)-sin(q1)*cos(q2)*sin(q4))*sin(q5))*sin(q6)-(sin(q1)*sin(q2)*cos(q3)-cos(q1)*sin(q3))*cos(q6);
      actual_z_[0] = -(-(-cos(q1)*sin(q2)*sin(q3)+sin(q1)*cos(q3))*sin(q4)+cos(q1)*cos(q2)*cos(q4))*sin(q5)+(-(-cos(q1)*sin(q2)*sin(q3)+sin(q1)*cos(q3))*cos(q4)-cos(q1)*cos(q2)*sin(q4))*cos(q5);                                                                                                                                                                             
      actual_z_[1] = -(-cos(q2)*sin(q3)*sin(q4)+sin(q2)*cos(q4))*sin(q5)+(-cos(q2)*sin(q3)*cos(q4)-sin(q2)*sin(q4))*cos(q5);
      actual_z_[2] = -(-(-sin(q1)*sin(q2)*sin(q3)-cos(q1)*cos(q3))*sin(q4)+sin(q1)*cos(q2)*cos(q4))*sin(q5)+(-(-sin(q1)*sin(q2)*sin(q3)-cos(q1)*cos(q3))*cos(q4)-sin(q1)*cos(q2)*sin(q4))*cos(q5);
      
      jac_(0,0) = 0;
      jac_(1,0) = -1;
      jac_(2,0) = 0;
      jac_(0,1) = -sin(q1);
      jac_(1,1) = 0;
      jac_(2,1) = cos(q1);
      jac_(0,2) = cos(q1)*cos(q2);
      jac_(1,2) = sin(q2);
      jac_(2,2) = sin(q1)*cos(q2);
      jac_(0,3) = cos(q1)*sin(q2)*cos(q3)+sin(q1)*sin(q3);
      jac_(1,3) = -cos(q2)*cos(q3);
      jac_(2,3) = sin(q1)*sin(q2)*cos(q3)-cos(q1)*sin(q3);
      jac_(0,4) = cos(q1)*sin(q2)*cos(q3)+sin(q1)*sin(q3);
      jac_(1,4) = -cos(q2)*cos(q3);
      jac_(2,4) = sin(q1)*sin(q2)*cos(q3)-cos(q1)*sin(q3);
      jac_(0,5) = -(-(-cos(q1)*sin(q2)*sin(q3)+sin(q1)*cos(q3))*sin(q4)+cos(q1)*cos(q2)*cos(q4))*sin(q5)+(-(-cos(q1)*sin(q2)*sin(q3)+sin(q1)*cos(q3))*cos(q4)-cos(q1)*cos(q2)*sin(q4))*cos(q5);                                                                                                                                                                             
      jac_(1,5) = -(-cos(q2)*sin(q3)*sin(q4)+sin(q2)*cos(q4))*sin(q5)+(-cos(q2)*sin(q3)*cos(q4)-sin(q2)*sin(q4))*cos(q5);
      jac_(2,5) = -(-(-sin(q1)*sin(q2)*sin(q3)-cos(q1)*cos(q3))*sin(q4)+sin(q1)*cos(q2)*cos(q4))*sin(q5)+(-(-sin(q1)*sin(q2)*sin(q3)-cos(q1)*cos(q3))*cos(q4)-sin(q1)*cos(q2)*sin(q4))*cos(q5);


      jspace::Status ok;
      return ok;
    }
    
    jspace::State getState() { return state_; }
    
    jspace::Status computeCommand(jspace::Vector const & eepos, jspace::Vector & command) {
      command = getCommand(eepos);
      jspace::Status ok;
      return ok;
    }
    
    jspace::Vector getCommand(jspace::Vector const & eepos) {
      if(eepos[2] < -0.10) { return state_.position_;}

      //this is fixed. could be setup to be the center of joints 1,2,3
      jspace::Vector origin(jspace::Vector::Zero(3));
      origin[2] = 0.13849;
      
      jspace::Vector goal_x(eepos - origin);
      goal_x /= goal_x.norm();
      jspace::Vector zdir(jspace::Vector::Zero(3));
      zdir[2] = 1;
      jspace::Vector goal_y(jspace::Vector::Zero(3));
      goal_y = cross(zdir,goal_x);
      goal_y /= goal_y.norm();
      jspace::Vector goal_z(jspace::Vector::Zero(3)); 
      goal_z = cross(goal_x,goal_y);
      jspace::Vector error(cross(actual_x_,goal_x) + cross(actual_y_,goal_y) + cross(actual_z_,goal_z));
      
      jspace::Vector ori(kp_.cwise() * error);
      jspace::Vector tau(state_.position_ + 0.0025 * jac_.transpose() * 0.5 *ori);      

      Vector command(jspace::Vector::Zero(12));
      for ( size_t ii(0); ii<6; ++ii) {
	command[ii] = tau[ii];
      }
      command[6] = tau[5];
    }

    jspace::Vector cross(jspace::Vector v1, jspace::Vector v2) {
	jspace::Vector result(jspace::Vector::Zero(3));
	result[0] = v1[1]*v2[2]-v1[2]*v2[1];
        result[1] = -v1[0]*v2[2]+v1[2]*v2[0];
        result[2] = v1[0]*v2[1]-v1[1]*v2[0];
	return result;
    }
    
  protected:
    jspace::State state_;
    jspace::Vector kp_;
    jspace::Matrix jac_;
    jspace::Vector actual_x_;
    jspace::Vector actual_y_;
    jspace::Vector actual_z_;
  };
}
#endif 

#include <jspace/constraint_library.hpp>
#include <jspace/Model.hpp>

namespace jspace {
  
  Dreamer_Base::Dreamer_Base() {
    sigmaThreshold_ = 0.0001;
    U_ = Matrix::Zero(3,9);
    U_(0,6) = 1; U_(1,7) = 1; U_(2,8) = 1;
    wheel_radius_ = 4 * 0.0254;
    x_prev = Vector::Zero(3);
    jpos_prev = Vector::Zero(3);
    Ainv = Matrix::Zero(9,9);
    Jc_ = Matrix::Zero(6,9);

    //Jacobian for the linear velocity of the base
    Jxyz = Matrix::Zero(3,9);
    Jxyz(0,0) = 1;
    Jxyz(1,1) = 1;
    Jxyz(2,2) = 1;

    //Jacobian for the rotational velocity of the base
    Jabg = Matrix::Zero(3,9);
    Jabg(0,3) = 1;
    Jabg(1,4) = 1;
    Jabg(2,5) = 1;
  }

  Status Dreamer_Base::updateJc(Model const & model) {
    Jc_ = Matrix::Zero(6,9);

    taoDNode* node = model.getNode(5);
    Vector wx; Vector wz; Vector r_vec;

    jspace::Transform ee_transform;
    Matrix base_ori;

    //Calculation of vector between the center of the wheel 
    // and the contact point
    model.computeGlobalFrame(node,0,0,-wheel_radius_,ee_transform);
    r_vec = ee_transform.translation();
    model.computeGlobalFrame(node,0,0,0,ee_transform);
    r_vec -= ee_transform.translation();

    Matrix r_cross(Matrix::Zero(3,3));
    r_cross(0,1) = r_vec(2); r_cross(0,2) = -r_vec(1);
    r_cross(1,0) = -r_vec(2); r_cross(1,2) = r_vec(0);
    r_cross(2,1) = r_vec(1); r_cross(2,1) = -r_vec(0);

    base_ori = ee_transform.linear();
    
    wz = base_ori.block(0,2,3,1); 
    
    Matrix Jfull; Matrix Jv; Matrix Jw;

    for (size_t ii(0); ii < 3; ++ii) {

      node = model.getNode(ii+6);
      model.computeGlobalFrame(node,0,0,0,ee_transform);
      Vector actual(ee_transform.translation());
      if ( ! model.computeJacobian(node, actual[0], actual[1], actual[2], Jfull)) {
	return Status(false, "failed to compute Jacobian (unsupported joint type?)");
      }
      Jv  = Jfull.block(0, 0, 3, Jfull.cols());
      Jw  = Jfull.block(3, 0, 3, Jfull.cols());

	  model.computeGlobalFrame(node,0,-1,0,ee_transform);
      wx = ee_transform.translation();
      model.computeGlobalFrame(node,0,0,0,ee_transform);
      wx -= ee_transform.translation();
      
      Jc_.block(2*ii,0,1,9) = wx.transpose()*(Jv + r_cross * Jw);
      Jc_.block(2*ii+1,0,1,9) = wz.transpose()*(Jv + r_cross * Jw);

    }
    
    //Save a copy for fullstate
    if(!model.getInverseMassInertia(Ainv)) {
      return Status(false,"failed to get mass inertia");
    }
    //Catch for first run where Ainv has not been initialized yet
    if (Ainv.rows() == 0) {
      Ainv = Matrix::Zero(9,9);
    }

    Status ok;
    return ok;
  }

  void Dreamer_Base::getFullState(State const & state,
				  State & fullState) {

    Matrix UNcBar;
    getUNcBar(Ainv,UNcBar);

    //Estimate base position and find base velocity from wheel states
    x_prev = x_prev + Jxyz * UNcBar * (state.position_ - jpos_prev);
    jpos_prev = state.position_;
    Vector x_vel(Jxyz * UNcBar * state.velocity_);

    for (size_t ii(0); ii < 3; ++ii) {
      fullState.position_[ii] = x_prev[ii];
      fullState.velocity_[ii] = x_vel[ii];
    }

    //Find alpha, beta, gamma velocities
    Vector ang_vel(Jabg * UNcBar * state.velocity_);
    for (size_t ii(0); ii < 3; ++ii) {
      fullState.velocity_[ii+3] = ang_vel[ii];
    }

    //Find alpha, beta, gamma from orientation matrix
    Matrix const ori_mtx(state.orientation_mtx_);

    if ( ori_mtx(2,0) < 0.99 && ori_mtx(2,0) > -0.99) {
      fullState.position_[4] = -asin(ori_mtx(2,0));
      fullState.position_[3] = atan2(ori_mtx(2,1)/cos(fullState.position_[4]), ori_mtx(2,2)/cos(fullState.position_[4]));
      fullState.position_[5] = atan2(ori_mtx(1,0)/cos(fullState.position_[4]), ori_mtx(0,0)/cos(fullState.position_[4]));
    }
    else {
      fullState.position_[5] = 0;
      if ( ori_mtx(2,0) < -0.99) {
	fullState.position_[4] = M_PI/2;
	fullState.position_[3] = fullState.position_[5] + atan2(ori_mtx(0,1), ori_mtx(0,2));
      }
      else {
	fullState.position_[4] = -M_PI/2;
	fullState.position_[3] = -fullState.position_[5] + atan2(-ori_mtx(0,1),-ori_mtx(0,2));
      }
    } 

    //Add wheels
    for (size_t ii(0); ii < 3; ++ii) {
      fullState.position_[ii+6] = state.position_[ii];
      fullState.velocity_[ii+6] = state.velocity_[ii];
    }

  }


  Dreamer_Torso::Dreamer_Torso() {
    sigmaThreshold_ = 0.0001;
    U_ = Matrix::Zero(9,10);
    U_(0,0) = 1; U_(1,1) = 1; U_(2,3) = 1; U_(3,4) = 1; U_(4,5) = 1;
    U_(5,6) = 1; U_(6,7) = 1; U_(7,8) = 1; U_(8,9) = 1;
    Jc_ = Matrix::Zero(1,10);
    Jc_(0,1) = 1; Jc_(0,2) = -1;
  }

  void Dreamer_Torso::getFullState(State const & state,
				   State & fullState) {
    for (size_t ii(0); ii < 10; ++ii) {
      if ( ii <2 ) {
	fullState.position_[ii] = state.position_[ii];
	fullState.velocity_[ii] = state.velocity_[ii];
      }
      else {
	fullState.position_[ii] = state.position_[ii-1];
	fullState.velocity_[ii] = state.velocity_[ii-1];
      }
    }
  }

  
  Dreamer_Full::Dreamer_Full() {
    sigmaThreshold_ = 0.0001;
    U_ = Matrix::Zero(12,19);
    U_(0,6) = 1; U_(1,7) = 1; U_(2,8) = 1; U_(3,9) = 1;
    U_(4,10) = 1; U_(5,12) = 1; U_(6,13) = 1; U_(7,14) = 1;
    U_(8,15) = 1; U_(9,16) = 1; U_(10,17) = 1; U_(11,18) = 1;
    wheel_radius_ = 4 * 0.0254;
    x_prev = Vector::Zero(3);
    jpos_prev = Vector::Zero(12);
    Ainv = Matrix::Zero(19,19);
    Jc_ = Matrix::Zero(7,19);

    //Jacobian for the linear velocity of the base
    Jxyz = Matrix::Zero(3,19);
    Jxyz(0,0) = 1;
    Jxyz(1,1) = 1;
    Jxyz(2,2) = 1;

    //Jacobian for the rotational velocity of the base
    Jabg = Matrix::Zero(3,19);
    Jabg(0,3) = 1;
    Jabg(1,4) = 1;
    Jabg(2,5) = 1;
  }

  Status Dreamer_Full::updateJc(Model const & model) {
    Jc_ = Matrix::Zero(7,19);

    taoDNode* node = model.getNode(5);
    Vector wx; Vector wz; Vector r_vec; Vector ay;

    jspace::Transform ee_transform;
    Matrix base_ori;

    //Calculation of vector between the center of the wheel 
    // and the contact point
    model.computeGlobalFrame(node,0,0,-wheel_radius_,ee_transform);
    r_vec = ee_transform.translation();
    model.computeGlobalFrame(node,0,0,0,ee_transform);
    r_vec -= ee_transform.translation();

    Matrix r_cross(Matrix::Zero(3,3));
    r_cross(0,1) = r_vec(2); r_cross(0,2) = -r_vec(1);
    r_cross(1,0) = -r_vec(2); r_cross(1,2) = r_vec(0);
    r_cross(2,1) = r_vec(1); r_cross(2,1) = -r_vec(0);

    base_ori = ee_transform.linear();
    
    ay = Vector::Zero(3);
    ay(1) = 1;
    wz = base_ori.block(0,2,3,1); 
    
    Matrix rot(Matrix::Identity(3,3));
    rot(0,0) = cos(2*M_PI/3);
    rot(0,1) = -sin(2*M_PI/3);
    rot(1,0) = sin(2*M_PI/3);
    rot(1,1) = cos(2*M_PI/3);
    
    Matrix Jfull; Matrix Jv; Matrix Jw;

    for (size_t ii(0); ii < 3; ++ii) {

      node = model.getNode(ii+6);
      model.computeGlobalFrame(node,0,0,0,ee_transform);
      Vector actual(ee_transform.translation());
      if ( ! model.computeJacobian(node, actual[0], actual[1], actual[2], Jfull)) {
	return Status(false, "failed to compute Jacobian (unsupported joint type?)");
      }
      Jv  = Jfull.block(0, 0, 3, Jfull.cols());
      Jw  = Jfull.block(3, 0, 3, Jfull.cols());

      ay = rot * ay;
      wx = base_ori * ay;
      
      Jc_.block(2*ii,0,1,19) = wx.transpose()*(Jv + r_cross * Jw);
      Jc_.block(2*ii+1,0,1,19) = wz.transpose()*(Jv + r_cross * Jw);

    }

    //Add the torso contraint
    Jc_(6,10) = 1; Jc_(6,11) = -1;
    
    //Save a copy for fullstate
    if(!model.getInverseMassInertia(Ainv)) {
      return Status(false,"failed to get mass inertia");
    }
    //Catch for first run where Ainv has not been initialized yet
    if (Ainv.rows() == 0) {
      Ainv = Matrix::Zero(19,19);
    }

    Status ok;
    return ok;

  }

  void Dreamer_Full::getFullState(State const & state,
				   State & fullState) {
    Matrix UNcBar;
    getUNcBar(Ainv,UNcBar);

    //Estimate base position and find base velocity from wheel states
    x_prev = x_prev + Jxyz * UNcBar * (state.position_ - jpos_prev);
    jpos_prev = state.position_;
    Vector x_vel(Jxyz * UNcBar * state.velocity_);

    for (size_t ii(0); ii < 3; ++ii) {
      fullState.position_[ii] = x_prev[ii];
      fullState.velocity_[ii] = x_vel[ii];
    }

    //Find alpha, beta, gamma velocities from wheel velocities
    Vector ang_vel(Jabg * UNcBar * state.velocity_);
    for (size_t ii(0); ii < 3; ++ii) {
      fullState.velocity_[ii+3] = ang_vel[ii];
    }

    //Find alpha, beta, gamma from orientation matrix
    Matrix const ori_mtx(state.orientation_mtx_);

    if ( ori_mtx(2,0) < 0.99 && ori_mtx(2,0) > -0.99) {
      fullState.position_[4] = -asin(ori_mtx(2,0));
      fullState.position_[3] = atan2(ori_mtx(2,1)/cos(fullState.position_[4]), ori_mtx(2,2)/cos(fullState.position_[4])) - M_PI;
      fullState.position_[5] = -atan2(ori_mtx(1,0)/cos(fullState.position_[4]), ori_mtx(0,0)/cos(fullState.position_[4])) + M_PI;

      if (fullState.position_[3] > M_PI) {
	fullState.position_[3] -= 2*M_PI;
      }
      else if (fullState.position_[3] < -M_PI) {
	fullState.position_[3] += 2*M_PI;
      }

      if (fullState.position_[5] > M_PI) {
	fullState.position_[5] -= 2*M_PI;
      }
      else if (fullState.position_[5] < -M_PI) {
	fullState.position_[5] += 2*M_PI;
      }

    }
    else {
      fullState.position_[5] = 0;
      if ( ori_mtx(2,0) < -0.99) {
	fullState.position_[4] = M_PI/2;
	fullState.position_[3] = fullState.position_[5] + atan2(ori_mtx(0,1), ori_mtx(0,2));
      }
      else {
	fullState.position_[4] = -M_PI/2;
	fullState.position_[3] = -fullState.position_[5] + atan2(-ori_mtx(0,1),-ori_mtx(0,2));
      }
    } 

    //Add wheels
    for (size_t ii(0); ii < 3; ++ii) {
      fullState.position_[ii+6] = state.position_[ii];
      fullState.velocity_[ii+6] = state.velocity_[ii];
    }

    //Torso DOF
    for (size_t ii(0); ii < 10; ++ii) {
      if ( ii <2 ) {
	fullState.position_[ii+9] = state.position_[ii+3];
	fullState.velocity_[ii+9] = state.velocity_[ii+3];
      }
      else {
	fullState.position_[ii+9] = state.position_[ii-1+3];
	fullState.velocity_[ii+9] = state.velocity_[ii-1+3];
      }
    }
  }

}

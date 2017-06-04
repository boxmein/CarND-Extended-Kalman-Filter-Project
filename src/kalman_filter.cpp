#include "kalman_filter.h"
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

double const pi = M_PI;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

VectorXd radar_h(VectorXd);

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  // Lesson 5: Part 12
  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // Lesson 5: Part 12
	VectorXd z_pred = radar_h(x_);
	VectorXd y = z - z_pred;

  {
    float phi = y(1);

    if (phi < -pi || phi > pi) {
      while (phi > pi) 
        phi -= 2 * pi;
      while (phi < -pi) 
        phi += 2 * pi;

      y(1) = phi;
    }


  }

	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

VectorXd radar_h(VectorXd x) {
  VectorXd ret(3);

  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  float rho = sqrt(px * px + py * py);

  float atan2_py_px = atan2(py, px);

  ret <<
        rho,
        atan2_py_px,
        (px*vx + py*vy) / rho;
  
  return ret;
}

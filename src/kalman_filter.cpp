#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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

  cout << "KalmanFilter initilized" << endl;
}

void KalmanFilter::Predict() {
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd y = z - H_ * x_;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;

  MatrixXd K = P_ * H_.transpose() * S.inverse();


  MatrixXd unitMatrix4 = MatrixXd(4,4);
  unitMatrix4 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

  x_ = x_ + K * y;

  P_ = (unitMatrix4 - K * H_) * P_;




}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //cout << "EKF started";
  const double pi = 3.1415926535897;

  VectorXd h = VectorXd(3);

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float c1 = sqrt(px*px+py*py);
  float angle = atan2(py, px);

  h << c1, atan2(py, px), (px*vx + py*vy)/c1;

  VectorXd y = z - h;

  while(y(1) <= -pi){
    y(1) += 2*pi;
  }

  while(y(1) >= pi){
    y(1) -= 2*pi;
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;

  MatrixXd K = P_ * H_.transpose() * S.inverse();


  MatrixXd unitMatrix4 = MatrixXd(4,4);
  unitMatrix4 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

  x_ = x_ + K * y;

  P_ = (unitMatrix4 - K * H_) * P_;
}

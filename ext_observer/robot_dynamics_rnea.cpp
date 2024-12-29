// Copyright 2020-2024 Stanislav Mikhel

#include "external_observer.hpp"

#define DELTA_INIT 1E-7

RobotDynamicsRnea::RobotDynamicsRnea()
  : RobotDynamicsBase()
  , _qext(Vec(1))
  , _p0(Vec(1))
  , _zero(Vec(1))
  , _sum(Vec(1))
  , _M(Mat(1, 1))
  , delta(DELTA_INIT)
{
}

// Find C^T * qd
Vec RobotDynamicsRnea::tranCqd(Vec& q, Vec& qd)
{
  int N = jointNo();
  _zero.resize(N);
  _zero.setZero();
  _p0 = rnea(q, _zero, qd);  // M * qd
  _sum.resize(N);
  _sum.setZero();

  for(int i = 0; i < N; i++) {
    _qext = q;
    _qext(i) += delta;
    _sum += (rnea(_qext, _zero, qd) - _p0) * (qd(i) / delta);
  }
  _sum -= rnea(q, qd, _zero);  // M'*qd - C*qd

  return _sum;
}

// Create M matrix from sequence of RNEA calls
Mat RobotDynamicsRnea::getM(Vec& q)
{
  int N = jointNo();
  _zero.resize(N); _zero.setZero();
  _M.resize(N, N);
  _qext.resize(N);

  for(int i = 0; i < N; i++) {
    _qext.setZero();  // use _qext to choose column
    _qext(i) = 1;
    _p0 = rnea(q, _zero, _qext);  // use _p0 to save matrix column
    for(int j = 0; j < N; j++)
      _M(j, i) = _p0(j);
  }

  return _M;
}

/**
 * @file momentum_observer_rnea.hpp
 *
 * @brief Momentum observer class.
 *
 * Expected robot dynamics in form of RNEA.
 */
#ifndef MOMENTUM_OBSERVER_RNEA_HPP
#define MOMENTUM_OBSERVER_RNEA_HPP

#include "external_observer.hpp"

#define ID_MomentumObserverRnea 25

/**
 * @brief Momentum observer from De Luca et al.
 *
 * Find dynamics using RNEA technique.
 */
class MomentumObserverRnea : public ExternalObserverRnea {
public:
  /**
   * @brief Object constructor.
   * @param rd pointer to robot object.
   * @param k vector of joint gains.
   */
  MomentumObserverRnea(RobotDynamicsRnea *rd, Vec& k);
  /**
   * @brief External torque estimation.
   * @param q joint angle vector.
   * @param qd joint velocity vector.
   * @param tau joint torque vector.
   * @param dt time step.
   * @return external torque vector.
   */
  Vec getExternalTorque(Vec& q, Vec& qd, Vec& tau, double dt);
  /**
   * @brief Observer settings.
   * @param k vector of the joint gains.
   */
  void settings(Vec& k);

private:
  // accumulators
  Vec sum, r, zero;
  // intermediate variables
  Vec p, beta, torque, tprev;
  // coefficients
  Vec ko;

}; // MomentumObserverRnea

// Initialization
MomentumObserverRnea::MomentumObserverRnea(RobotDynamicsRnea *rd, Vec& k)
  : ExternalObserverRnea(rd,ID_MomentumObserverRnea)
  , sum(Vec(jointNo))
  , r(Vec(jointNo))
  , zero(Vec::Zero(jointNo))
  , p(Vec(jointNo))
  , beta(Vec(jointNo))
  , torque(Vec(jointNo))
  , tprev(Vec(jointNo))
  , ko(k)
{
  settings(k);
}

// Torque estimation
Vec MomentumObserverRnea::getExternalTorque(Vec& q, Vec& qd, Vec& tau, double dt)
{
  p = dyn->rnea(q,zero,qd);     // M * qd
  beta = dyn->rnea(q,zero,zero,GRAVITY)- dyn->tranCqd(q,qd);  // G - C' * qd

  torque = tau - dyn->getFriction(qd); // exclude friction
  if(isRun) {
    torque += r - beta;      // tau + r - beta
    sum += 0.5 * dt * (torque + tprev);
  } else {
    torque -= beta;
    r.setZero();
    sum = p;
    isRun = true;
  }
  tprev = torque;

  p -= sum;                 // p - integral - p0

  // elementwise product
  for(int i = 0; i < jointNo; i++) {
    r(i) = ko(i) * p(i);
    torque(i) = r(i);   // reuse variable
  }

  return torque;
}

// Set gains
void MomentumObserverRnea::settings(Vec& k)
{
  ko = k;
}

#endif // MOMENTUM_OBSERVER_RNEA_HPP

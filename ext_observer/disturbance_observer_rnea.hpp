// Copyright 2020-2024 Stanislav Mikhel
/**
 * @file disturbance_observer_rnea.h
 *
 * @brief Disturbance observer class.
 *
 * Expected robot dynamics in form of RNEA.
 */
#ifndef EXT_OBSERVER__DISTURBANCE_OBSERVER_RNEA_HPP_
#define EXT_OBSERVER__DISTURBANCE_OBSERVER_RNEA_HPP_

#include "external_observer.hpp"

#define ID_DisturbanceObserverRnea 24

/**
 * @brief Disturbance observer from Mohammadi et. al.
 *
 * Use RNEA technique for dynamics.
 */
class DisturbanceObserverRnea : public ExternalObserverRnea {
public:
  /**
   * @brief Object constructor.
   * @param rd pointer to the robot object.
   * @param sigma ...
   * @param xeta ...
   * @param beta ...
   */
  DisturbanceObserverRnea(RobotDynamicsRnea *rd, double sigma, double xeta, double beta);
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
   * @param sigma ...
   * @param xeta ...
   * @param beta ...
   */
  void settings(double sigma, double xeta, double beta);

private:
  // temporary objects
  Mat Y, L, I;
  Mat lft, rht;
  Vec p, z, torque, zeros;
};  // DisturbanceObserverRnea

// Initialization
DisturbanceObserverRnea::DisturbanceObserverRnea(
  RobotDynamicsRnea *rd, double sigma, double xeta, double beta)
  : ExternalObserverRnea(rd, ID_DisturbanceObserverRnea)
  , Y(Mat(jointNo, jointNo))
  , L(Mat(jointNo, jointNo))
  , I(Mat::Identity(jointNo, jointNo))
  , lft(Mat(jointNo, jointNo))
  , rht(Mat(jointNo, jointNo))
  , p(Vec(jointNo))
  , z(Vec(jointNo))
  , torque(Vec(jointNo))
  , zeros(Vec::Zero(jointNo))
{
  settings(sigma, xeta, beta);
}

// Get torque
Vec DisturbanceObserverRnea::getExternalTorque(Vec& q, Vec& qd, Vec& tau, double dt)
{
  L = Y * dyn->getM(q).inverse();
  L *= dt;
  p = Y * qd;

  if(isRun) {
    torque = tau - dyn->getFriction(qd);
    lft = I + L;
    rht = z + L*(dyn->rnea(q, qd, zeros, GRAVITY) - torque - p);
    z = lft.inverse() * rht;
  } else {
    z = -p;
    isRun = true;
  }

  p += z;

  return p;
}

// Update parameters
void DisturbanceObserverRnea::settings(double sigma, double xeta, double beta)
{
  double k = 0.5*(xeta + 2*beta*sigma);
  Y = k * Mat::Identity(jointNo, jointNo);
}

#endif  // EXT_OBSERVER__DISTURBANCE_OBSERVER_RNEA_HPP_

/**
 * @file sliding_mode_observer_rnea.hpp
 *
 * @brief Sliding mode observer class.
 *
 * Expected robot dynamics in form of RNEA.
 */
#ifndef SLIDING_MODE_OBSERVER_RNEA_HPP
#define SLIDING_MODE_OBSERVER_RNEA_HPP


#include "external_observer.hpp"

#define BIGR 50   /**< Map tanh to sign. */
#define ID_SlidingModeObserverRnea 23

/**
 * @brief Sliding mode observer from ...
 *
 * Use RNEA for dynamics.
 */
class SlidingModeObserverRnea : public ExternalObserverRnea {
public:
  /**
   * @brief Object constructor.
   * @param rd pointer to the robot object.
   * @param t1 ...
   * @param s1 ...
   * @param t2 ...
   * @param s2 ...
   */
  SlidingModeObserverRnea(RobotDynamicsRnea* rd, Vec& t1, Vec& s1, Vec& t2, Vec& s2);
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
   * @param t1 ...
   * @param s1 ...
   * @param t2 ...
   * @param s2 ...
   */
  void settings(Vec& t1, Vec& s1, Vec& t2, Vec& s2);

private:
 // Temporary objects
 Vec sigma, p_hat;
 Vec p, spp, dp_hat, torque, dsigma, zeros;
 Vec T1, S1, T2, S2;

}; // SlidingModeObserverRnea

// Initialization
SlidingModeObserverRnea::SlidingModeObserverRnea(RobotDynamicsRnea* rd,
                                Vec& t1, Vec& s1, Vec& t2, Vec& s2)
  : ExternalObserverRnea(rd,ID_SlidingModeObserverRnea)
  , sigma(Vec(jointNo))
  , p_hat(Vec(jointNo))
  , p(Vec(jointNo))
  , spp(Vec(jointNo))
  , dp_hat(Vec(jointNo))
  , torque(Vec(jointNo))
  , dsigma(Vec(jointNo))
  , zeros(Vec::Zero(jointNo))
  , T1(Vec(jointNo)), S1(Vec(jointNo))
  , T2(Vec(jointNo)), S2(Vec(jointNo))
{
  settings(t1,s1,t2,s2);
}

// Update settings
void SlidingModeObserverRnea::settings(Vec& t1, Vec& s1, Vec& t2, Vec& s2)
{
  T1 = t1;
  S1 = s1;
  T2 = t2;
  S2 = s2;
}

// External torque
Vec SlidingModeObserverRnea::getExternalTorque(Vec& q, Vec& qd, Vec& tau, double dt)
{
  p = dyn->rnea(q,zeros,qd);
  torque = tau - dyn->getFriction(qd);

  if(!isRun) {
    sigma.setZero();
    p_hat = p;
    isRun = true;
  }

  p = p_hat - p;  // reuse p
  for(int i = 0; i < jointNo; i++) spp(i) = tanh(p(i)*BIGR);

  dp_hat = torque + dyn->tranCqd(q,qd) - dyn->rnea(q,zeros,zeros,GRAVITY) + sigma;
  for(int i = 0; i < jointNo; i++) {
    // - T2*p
    dp_hat(i) -= T2(i)*p(i);
    // - sqrt(abs(p)).*T1*spp
    dp_hat(i) -= sqrt( fabs(p(i)) ) * T1(i) * spp(i);
  }
  // dsigma
  for(int i = 0; i < jointNo; i++) {
    dsigma(i) = -S1(i)*spp(i) - S2(i)*p(i);
  }

  p = sigma;     // reuse to save result
  p_hat += dt*dp_hat;
  sigma += dt*dsigma;

  return p;
}

#endif  // SLIDING_MODE_OBSERVER_RNEA_HPP

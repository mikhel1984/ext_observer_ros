// Copyright 2020-2024 Stanislav Mikhel

#ifndef EXT_OBSERVER__FILTERED_DYN_OBSERVER_RNEA_HPP_
#define EXT_OBSERVER__FILTERED_DYN_OBSERVER_RNEA_HPP_

#include "external_observer.hpp"
#include "iir_filter.hpp"

#define ID_FDynObserverRnea 22

class FDynObserverRnea : public ExternalObserverRnea {
public:
  FDynObserverRnea(RobotDynamicsRnea *rd, double cutOffHz, double sampHz);

  Vec getExternalTorque(Vec& q, Vec& qd, Vec& tau, double dt);

  void settings(double cutOffHz, double sampHz);

private:
  FilterF1 f1;
  FilterF2 f2;
  Vec p, res, zero;
};  // FDynObserver

FDynObserverRnea::FDynObserverRnea(RobotDynamicsRnea *rd, double cutOffHz, double sampHz)
  : ExternalObserverRnea(rd, ID_FDynObserverRnea)
  , f1(FilterF1(cutOffHz, sampHz, jointNo))
  , f2(FilterF2(cutOffHz, sampHz, jointNo))
  , p(Vec(jointNo))
  , res(Vec(jointNo))
  , zero(Vec::Zero(jointNo))
{
}

void FDynObserverRnea::settings(double cutOffHz, double sampHz)
{
  f1.update(cutOffHz, sampHz);
  f2.update(cutOffHz, sampHz);
}

Vec FDynObserverRnea::getExternalTorque(Vec& q, Vec& qd, Vec& tau, double dt)
{
  p = dyn->rnea(q, zero, qd);     // M * qd

  if(isRun) {
    res = f2.filt(p, dt) + f2.getOmega() * p ;
    p = dyn->getFriction(qd) + dyn->rnea(q, zero, zero, GRAVITY) - dyn->tranCqd(q, qd);  // reuse
    p -= tau;
    res += f1.filt(p, dt);
  } else {
    f2.set(p);
    p = dyn->getFriction(qd) + dyn->rnea(q, zero, zero, GRAVITY) - dyn->tranCqd(q, qd);  // reuse
    p -= tau;
    f1.set(p);
    res.setZero();
    isRun = true;
  }

  return res;
}

#endif  // EXT_OBSERVER__FILTERED_DYN_OBSERVER_RNEA_HPP_

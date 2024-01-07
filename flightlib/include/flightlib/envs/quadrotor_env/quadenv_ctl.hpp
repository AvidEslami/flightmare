#pragma once

#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/timer.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/envs/env_base.hpp"
#include "flightlib/objects/quadrotor.hpp"

namespace flightlib {
namespace quadenv {

enum Ctl : int {
  // observations
  kObs = 0,
  kPos = 0,
  kNPos = 3,
  kOri = 3,
  kNOri = 3,
  kLinVel = 6,
  kNLinVel = 3,
  kAngVel = 9,
  kNAngVel = 3,
  kNObs = 12,
  // control actions
  kAct = 0,
  kNAct = 4,
};

}  // namespace quadenv
}  // namespace flightlib


// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// flightlib
#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_hover_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_continuous_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env_bydata.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_angled_env.hpp"
#include "flightlib/envs/test_env.hpp"
#include "flightlib/envs/vec_env.hpp"

namespace py = pybind11;
using namespace flightlib;

PYBIND11_MODULE(flightgym, m) {
  py::class_<VecEnv<QuadrotorEnv>>(m, "QuadrotorEnv_v1")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset", &VecEnv<QuadrotorEnv>::reset)
    .def("reset_range", &VecEnv<QuadrotorEnv>::resetRange)
    .def("step", &VecEnv<QuadrotorEnv>::step)
    .def("testStep", &VecEnv<QuadrotorEnv>::testStep)
    .def("setSeed", &VecEnv<QuadrotorEnv>::setSeed)
    .def("close", &VecEnv<QuadrotorEnv>::close)
    .def("isTerminalState", &VecEnv<QuadrotorEnv>::isTerminalState)
    .def("curriculumUpdate", &VecEnv<QuadrotorEnv>::curriculumUpdate)
    .def("connectUnity", &VecEnv<QuadrotorEnv>::connectUnity)
    .def("disconnectUnity", &VecEnv<QuadrotorEnv>::disconnectUnity)
    .def("getNumOfEnvs", &VecEnv<QuadrotorEnv>::getNumOfEnvs)
    .def("getObsDim", &VecEnv<QuadrotorEnv>::getObsDim)
    .def("getActDim", &VecEnv<QuadrotorEnv>::getActDim)
    .def("getExtraInfoNames", &VecEnv<QuadrotorEnv>::getExtraInfoNames)
    // .def("getEpisodeLength", &VecEnv<QuadrotorEnv>::getEpisodeLength)
    .def("__repr__", [](const VecEnv<QuadrotorEnv>& a) {
      return "RPG Drone Racing Environment";
    });

  py::class_<TestEnv<QuadrotorEnv>>(m, "TestEnv_v0")
    .def(py::init<>())
    .def("reset", &TestEnv<QuadrotorEnv>::reset)
    .def("__repr__", [](const TestEnv<QuadrotorEnv>& a) { return "Test Env"; });

  py::class_<VecEnv<QuadrotorHoverEnv>>(m, "QuadrotorHoverEnv_v1")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset", &VecEnv<QuadrotorHoverEnv>::reset)
    .def("step", &VecEnv<QuadrotorHoverEnv>::step)
    .def("testStep", &VecEnv<QuadrotorHoverEnv>::testStep)
    .def("setSeed", &VecEnv<QuadrotorHoverEnv>::setSeed)
    .def("close", &VecEnv<QuadrotorHoverEnv>::close)
    .def("isTerminalState", &VecEnv<QuadrotorHoverEnv>::isTerminalState)
    .def("curriculumUpdate", &VecEnv<QuadrotorHoverEnv>::curriculumUpdate)
    .def("connectUnity", &VecEnv<QuadrotorHoverEnv>::connectUnity)
    .def("disconnectUnity", &VecEnv<QuadrotorHoverEnv>::disconnectUnity)
    .def("getNumOfEnvs", &VecEnv<QuadrotorHoverEnv>::getNumOfEnvs)
    .def("getObsDim", &VecEnv<QuadrotorHoverEnv>::getObsDim)
    .def("getActDim", &VecEnv<QuadrotorHoverEnv>::getActDim)
    .def("getExtraInfoNames", &VecEnv<QuadrotorHoverEnv>::getExtraInfoNames)
    // .def("getEpisodeLength", &VecEnv<QuadrotorHoverEnv>::getEpisodeLength)
    .def("__repr__", [](const VecEnv<QuadrotorHoverEnv>& a) {
      return "RPG Test Environment";
    });

  py::class_<VecEnv<QuadrotorContinuousEnv>>(m, "QuadrotorContinuousEnv_v1")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset", &VecEnv<QuadrotorContinuousEnv>::reset)
    .def("step", &VecEnv<QuadrotorContinuousEnv>::step)
    .def("testStep", &VecEnv<QuadrotorContinuousEnv>::testStep)
    .def("setSeed", &VecEnv<QuadrotorContinuousEnv>::setSeed)
    .def("close", &VecEnv<QuadrotorContinuousEnv>::close)
    .def("isTerminalState", &VecEnv<QuadrotorContinuousEnv>::isTerminalState)
    .def("curriculumUpdate", &VecEnv<QuadrotorContinuousEnv>::curriculumUpdate)
    .def("connectUnity", &VecEnv<QuadrotorContinuousEnv>::connectUnity)
    .def("disconnectUnity", &VecEnv<QuadrotorContinuousEnv>::disconnectUnity)
    .def("getNumOfEnvs", &VecEnv<QuadrotorContinuousEnv>::getNumOfEnvs)
    .def("getObsDim", &VecEnv<QuadrotorContinuousEnv>::getObsDim)
    .def("getActDim", &VecEnv<QuadrotorContinuousEnv>::getActDim)
    .def("getExtraInfoNames", &VecEnv<QuadrotorContinuousEnv>::getExtraInfoNames)
    // .def("getEpisodeLength", &VecEnv<QuadrotorContinuousEnv>::getEpisodeLength)
    .def("__repr__", [](const VecEnv<QuadrotorContinuousEnv>& a) {
      return "RPG Continuous Test Environment";
    });

  py::class_<VecEnv<QuadrotorEnvByData>>(m, "QuadrotorEnv_Bydata")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset", &VecEnv<QuadrotorEnvByData>::reset)
    .def("step", &VecEnv<QuadrotorEnvByData>::step)
    .def("testStep", &VecEnv<QuadrotorEnvByData>::testStep)
    .def("setSeed", &VecEnv<QuadrotorEnvByData>::setSeed)
    .def("close", &VecEnv<QuadrotorEnvByData>::close)
    .def("isTerminalState", &VecEnv<QuadrotorEnvByData>::isTerminalState)
    .def("curriculumUpdate", &VecEnv<QuadrotorEnvByData>::curriculumUpdate)
    .def("connectUnity", &VecEnv<QuadrotorEnvByData>::connectUnity)
    .def("disconnectUnity", &VecEnv<QuadrotorEnvByData>::disconnectUnity)
    .def("getNumOfEnvs", &VecEnv<QuadrotorEnvByData>::getNumOfEnvs)
    .def("getObsDim", &VecEnv<QuadrotorEnvByData>::getObsDim)
    .def("getActDim", &VecEnv<QuadrotorEnvByData>::getActDim)
    .def("getExtraInfoNames", &VecEnv<QuadrotorEnvByData>::getExtraInfoNames)
    // .def("getEpisodeLength", &VecEnv<QuadrotorEnvByData>::getEpisodeLength)
    .def("__repr__", [](const VecEnv<QuadrotorEnvByData>& a) {
      return "RPG Continuous Test Environment";
    });

  py::class_<VecEnv<QuadrotorAngledEnv>>(m, "QuadrotorAngledEnv_v1")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset", &VecEnv<QuadrotorAngledEnv>::reset)
    .def("reset_range", &VecEnv<QuadrotorAngledEnv>::resetRange)
    .def("step", &VecEnv<QuadrotorAngledEnv>::step)
    .def("testStep", &VecEnv<QuadrotorAngledEnv>::testStep)
    .def("setSeed", &VecEnv<QuadrotorAngledEnv>::setSeed)
    .def("close", &VecEnv<QuadrotorAngledEnv>::close)
    .def("isTerminalState", &VecEnv<QuadrotorAngledEnv>::isTerminalState)
    .def("curriculumUpdate", &VecEnv<QuadrotorAngledEnv>::curriculumUpdate)
    .def("connectUnity", &VecEnv<QuadrotorAngledEnv>::connectUnity)
    .def("disconnectUnity", &VecEnv<QuadrotorAngledEnv>::disconnectUnity)
    .def("getNumOfEnvs", &VecEnv<QuadrotorAngledEnv>::getNumOfEnvs)
    .def("getObsDim", &VecEnv<QuadrotorAngledEnv>::getObsDim)
    .def("getActDim", &VecEnv<QuadrotorAngledEnv>::getActDim)
    .def("getExtraInfoNames", &VecEnv<QuadrotorAngledEnv>::getExtraInfoNames)
    // .def("getEpisodeLength", &VecEnv<QuadrotorEnv>::getEpisodeLength)
    .def("__repr__", [](const VecEnv<QuadrotorAngledEnv>& a) {
      return "RPG Drone Racing Environment";
    });
}
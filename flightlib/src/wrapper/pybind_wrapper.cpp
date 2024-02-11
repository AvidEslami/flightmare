
// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// flightlib
#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_hover_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_continuous_env.hpp"
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
}
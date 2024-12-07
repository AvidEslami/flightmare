
// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// flightlib
#include "flightlib/envs/env_base.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_hover_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_continuous_env.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env_bydata_traj.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env_bydata_prog.hpp"
#include "flightlib/envs/quadrotor_env/quadrotor_env_bydata.hpp"
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

  py::class_<VecEnv<QuadrotorEnvByDataTraj>>(m, "QuadrotorEnv_Bydata_Traj")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset", &VecEnv<QuadrotorEnvByDataTraj>::reset)
    .def("step", &VecEnv<QuadrotorEnvByDataTraj>::step)
    .def("testStep", &VecEnv<QuadrotorEnvByDataTraj>::testStep)
    .def("setSeed", &VecEnv<QuadrotorEnvByDataTraj>::setSeed)
    .def("close", &VecEnv<QuadrotorEnvByDataTraj>::close)
    .def("isTerminalState", &VecEnv<QuadrotorEnvByDataTraj>::isTerminalState)
    .def("curriculumUpdate", &VecEnv<QuadrotorEnvByDataTraj>::curriculumUpdate)
    .def("connectUnity", &VecEnv<QuadrotorEnvByDataTraj>::connectUnity)
    .def("disconnectUnity", &VecEnv<QuadrotorEnvByDataTraj>::disconnectUnity)
    .def("getNumOfEnvs", &VecEnv<QuadrotorEnvByDataTraj>::getNumOfEnvs)
    .def("getObsDim", &VecEnv<QuadrotorEnvByDataTraj>::getObsDim)
    .def("getActDim", &VecEnv<QuadrotorEnvByDataTraj>::getActDim)
    .def("getExtraInfoNames", &VecEnv<QuadrotorEnvByDataTraj>::getExtraInfoNames)
    // .def("getEpisodeLength", &VecEnv<QuadrotorEnvByDataTraj>::getEpisodeLength)
    .def("__repr__", [](const VecEnv<QuadrotorEnvByDataTraj>& a) {
      return "RPG Continuous Test Environment";
    });

  py::class_<VecEnv<QuadrotorEnvByDataProg>>(m, "QuadrotorEnv_Bydata_Prog")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset", &VecEnv<QuadrotorEnvByDataProg>::reset)
    .def("step", &VecEnv<QuadrotorEnvByDataProg>::step)
    .def("testStep", &VecEnv<QuadrotorEnvByDataProg>::testStep)
    .def("setSeed", &VecEnv<QuadrotorEnvByDataProg>::setSeed)
    .def("close", &VecEnv<QuadrotorEnvByDataProg>::close)
    .def("isTerminalState", &VecEnv<QuadrotorEnvByDataProg>::isTerminalState)
    .def("curriculumUpdate", &VecEnv<QuadrotorEnvByDataProg>::curriculumUpdate)
    .def("connectUnity", &VecEnv<QuadrotorEnvByDataProg>::connectUnity)
    .def("disconnectUnity", &VecEnv<QuadrotorEnvByDataProg>::disconnectUnity)
    .def("getNumOfEnvs", &VecEnv<QuadrotorEnvByDataProg>::getNumOfEnvs)
    .def("getObsDim", &VecEnv<QuadrotorEnvByDataProg>::getObsDim)
    .def("getActDim", &VecEnv<QuadrotorEnvByDataProg>::getActDim)
    .def("getExtraInfoNames", &VecEnv<QuadrotorEnvByDataProg>::getExtraInfoNames)
    // .def("getEpisodeLength", &VecEnv<QuadrotorEnvByDataProg>::getEpisodeLength)
    .def("__repr__", [](const VecEnv<QuadrotorEnvByDataProg>& a) {
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
}
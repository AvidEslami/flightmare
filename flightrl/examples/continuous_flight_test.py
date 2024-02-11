#!/usr/bin/env python3
from ruamel.yaml import YAML, dump, RoundTripDumper

#
import os
import math
import argparse
import numpy as np
import tensorflow as tf

#
from stable_baselines import logger

#
from rpg_baselines.common.policies import MlpPolicy
from rpg_baselines.ppo.ppo2 import PPO2
from rpg_baselines.ppo.ppo2_test import test_model
from rpg_baselines.envs import vec_env_wrapper as wrapper
import rpg_baselines.common.util as U
#
from flightgym import QuadrotorContinuousEnv_v1


def configure_random_seed(seed, env=None):
    if env is not None:
        env.seed(seed)
    np.random.seed(seed)
    tf.set_random_seed(seed)


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--render', type=int, default=0,
                        help="Enable Unity Render")
    parser.add_argument('--seed', type=int, default=0,
                        help="Random seed")
    parser.add_argument('-w', '--weight', type=str, default='./saved/quadrotor_env.zip',
                        help='trained weight path')
    return parser


def main():
    args = parser().parse_args()
    cfg = YAML().load(open(os.environ["FLIGHTMARE_PATH"] +
                           "/flightlib/configs/vec_env.yaml", 'r'))

    cfg["env"]["num_envs"] = 1
    cfg["env"]["num_threads"] = 1
    cfg["env"]["max_episode_steps"] = 2000

    if args.render:
        cfg["env"]["render"] = "yes"
    else:
        cfg["env"]["render"] = "no"

    env = wrapper.FlightEnvVec(QuadrotorContinuousEnv_v1(
        dump(cfg, Dumper=RoundTripDumper), False))
    
    env.max_episode_steps = 2000
    # env = DynamicGate_v0(dump(cfg["env"], Dumper=RoundTripDumper))
    # env = EnvWrapper(env)

    # set random seed
    configure_random_seed(args.seed, env=env)

    # Begin Test:
    model = PPO2.load(args.weight)
    test_model(env, model, render=args.render)


if __name__ == "__main__":
    main()

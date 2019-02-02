import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)
print("parentdir=",parentdir)
import json
from pybullet_envs.deep_mimic.learning.rl_world import RLWorld
from learning.ppo_agent import PPOAgent

import pybullet_data
from pybullet_utils.arg_parser import ArgParser
from pybullet_utils.logger import Logger
from pybullet_envs.deep_mimic.env.pybullet_deep_mimic_env import PyBulletDeepMimicEnv
import sys
import random

def build_arg_parser(args):
    arg_parser = ArgParser()
    arg_parser.load_args(args)

    arg_file = arg_parser.parse_string('arg_file', '')
    if (arg_file != ''):
        path = pybullet_data.getDataPath()+"/args/"+arg_file
        succ = arg_parser.load_file(path)
        Logger.print2(arg_file)
        assert succ, Logger.print2('Failed to load args from: ' + arg_file)
    return arg_parser

args = sys.argv[1:]
arg_parser = build_arg_parser(args)

render=False#True
env = PyBulletDeepMimicEnv (args,render)
 
world = RLWorld(env, arg_parser)

motion_file = arg_parser.parse_string("motion_file")
print("motion_file=",motion_file)
bodies = arg_parser.parse_ints("fall_contact_bodies")
print("bodies=",bodies)
int_output_path = arg_parser.parse_string("int_output_path")
print("int_output_path=",int_output_path)

agent_files = pybullet_data.getDataPath()+"/"+arg_parser.parse_string("agent_files")

AGENT_TYPE_KEY = "AgentType"

print("agent_file=",agent_files)
with open(agent_files) as data_file:
    json_data = json.load(data_file)
    print("json_data=",json_data)
    assert AGENT_TYPE_KEY in json_data
    agent_type = json_data[AGENT_TYPE_KEY]
    print("agent_type=",agent_type)
    agent = PPOAgent(world, id, json_data)
    
    agent.set_enable_training(True)
    world.reset()
    while (world.env._pybullet_client.isConnected()):

      timeStep = 1./600.
      world.update(timeStep)
      reward = world.env.calc_reward(agent_id=0)
      #print("reward=",reward)
      
      end_episode = world.env.is_episode_end()
      if (end_episode):
        world.end_episode()
        world.reset()

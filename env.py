import numpy as np
import quaternion
import torch
import sys
import habitat
from habitat import logger
import gym
import utils as vu
import matplotlib
import os
if sys.platform == 'darwin':
    matplotlib.use("tkagg")
else:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt


class exploration_env(habitat.RLEnv):
    def __init__(self, args, rank, config_env, dataset):
        if args.visualize:
            plt.ion()
        if args.print_images or args.visualize:
            self.figure, self.ax = plt.subplots(1,2, figsize=(6*16/9, 6),
                                                facecolor="whitesmoke",
                                                num="Thread {}".format(rank))
        self.args = args
        self.num_actions = 4
        self.rank = rank
        self.timestep = 0
        self.episode_no = 0
        super().__init__(config_env, dataset)

        self.action_space = gym.spaces.Discrete(self.num_actions)

        self.observation_space = gym.spaces.Box(0, 255,
                                                (3, args.frame_height,
                                                    args.frame_width),
                                                dtype='uint8')

    def randomize_env(self):
        self._env._episode_iterator._shuffle_iterator()

    def save_position(self):
        self.agent_state = self._env.sim.get_agent_state()
        self.trajectory_states.append([self.agent_state.position,
                                       self.agent_state.rotation])


    def reset(self):

        obs = super().reset()
        self.episode_no += 1
        # Additional info from the environment
        self.info = {
            'time': self.timestep,
        }


        return obs, self.info

    def step(self, action):

        self.timestep += 1
        if (action==0):
            action = 1
        obs, rew, done, info = super().step(action)

        # Set info
        self.info['time'] = self.timestep

        if self.info['time'] >= self.args.max_episode_length:
            done = True
        else:
            done = False

        # Data Logging 
        dump_dir = "{}/data/{}/".format(self.args.dump_location,
                                                self.args.exp_name)
       
        ep_dir = '{}/episodes/{}/{}/'.format(
                        dump_dir, self.rank+1, self.episode_no)
            
        if not os.path.exists(ep_dir):
            os.makedirs(ep_dir)  
        vu.visualize(self.figure, self.ax, obs['rgb'], obs['depth'],
                            dump_dir, self.rank, self.episode_no,
                            self.timestep, self.args.visualize,
                            self.args.print_images) 
        return obs, rew, done, self.info

    def get_reward_range(self):
        # This function is not used, Habitat-RLEnv requires this function
        return (0., 1.0)

    def get_reward(self, observations):
        # This function is not used, Habitat-RLEnv requires this function
        return 0.

    def get_done(self, observations):
        # This function is not used, Habitat-RLEnv requires this function
        return False

    def get_info(self, observations):
        # This function is not used, Habitat-RLEnv requires this function
        return self.info

    def seed(self, seed):
        self.rng = np.random.RandomState(seed)

    def get_spaces(self):
        return self.observation_space, self.action_space

    def get_sim_location(self):
        agent_state = super().habitat_env.sim.get_agent_state(0)
        x = -agent_state.position[2]
        y = -agent_state.position[0]
        axis = quaternion.as_euler_angles(agent_state.rotation)[0]
        if (axis%(2*np.pi)) < 0.1 or (axis%(2*np.pi)) > 2*np.pi - 0.1:
            o = quaternion.as_euler_angles(agent_state.rotation)[1]
        else:
            o = 2*np.pi - quaternion.as_euler_angles(agent_state.rotation)[1]
        if o > np.pi:
            o -= 2 * np.pi
        return x, y, o




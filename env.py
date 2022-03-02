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
import habitat_sim
if sys.platform == 'darwin':
    matplotlib.use("tkagg")
else:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
from habitat.utils.visualizations import maps


class exploration_env(habitat.RLEnv):
    def __init__(self, args, rank, config_env, dataset):
        if args.visualize:
            plt.ion()
        if args.print_images or args.visualize:
            self.figure, self.ax = plt.subplots(1,3, figsize=(6*16/9, 6),
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
        self.top_down_map = maps.get_topdown_map_from_sim(self._env.sim,agent_id=0,meters_per_pixel=0.05,draw_border=True)
        self.top_down_map = maps.colorize_topdown_map(top_down_map=self.top_down_map)
        self.add_objects_random()
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
        #print(self._env.sim.get_agent_state().position)
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
        
        # Agents pose in x,y frame 
        x, y, o = self.get_sim_location()
        # Agents pose in map grid image 
        x_grid,y_grid = maps.to_grid(realworld_x=x,realworld_y=y,grid_resolution=[self.top_down_map.shape[0],self.top_down_map.shape[1]],
                                sim=self._env.sim,pathfinder=self._env.sim.pathfinder)
        # Draw agent in the map 
        self.top_down_map = maps.draw_agent(self.top_down_map,(x_grid,y_grid),o)  
        if not os.path.exists(ep_dir):
            os.makedirs(ep_dir)  
        if self.args.print_images or self.args.visualize:    
            vu.visualize(self.figure, self.ax, obs['rgb'], obs['depth'],self.top_down_map,
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
        x = agent_state.position[2]
        y = agent_state.position[0]
        axis = quaternion.as_euler_angles(agent_state.rotation)[0]
        if (axis%(2*np.pi)) < 0.1 or (axis%(2*np.pi)) > 2*np.pi - 0.1:
            o = quaternion.as_euler_angles(agent_state.rotation)[1]
        else:
            o = 2*np.pi - quaternion.as_euler_angles(agent_state.rotation)[1]
        if o > np.pi:
            o -= 2 * np.pi
        return x, y, o + np.pi

    def add_objects_random(self):
        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        # Get the physics object attributes manager
        obj_templates_mgr = super().habitat_env.sim.get_object_template_manager()
        existing_object_ids = super().habitat_env.sim.get_existing_object_ids()
        # Remove existing objects from the scene
        if len(existing_object_ids) > 0:
            for obj_id in existing_object_ids:
                super().habitat_env.sim.remove_object(obj_id)
        # Agent size based on the scene
        agent_size = int(self.top_down_map.shape[0] / 30) * 1
        for i in range(5):
            # Load banana templates from configuration files
            banana_template_id = obj_templates_mgr.load_configs(
                str("data/object_datasets/banana"))[0]
            # add an object to the scene
            banana_id = super().habitat_env.sim.add_object(banana_template_id)
            super().habitat_env.sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, banana_id)
            banana_pos = super().habitat_env.sim.sample_navigable_point()
            super().habitat_env.sim.set_translation(banana_pos, banana_id)
            banana_x_grid,banana_y_grid = maps.to_grid(realworld_x=banana_pos[2],realworld_y=banana_pos[0],grid_resolution=[self.top_down_map.shape[0],self.top_down_map.shape[1]],
                                sim=self._env.sim,pathfinder=self._env.sim.pathfinder)
            source_square = np.zeros((agent_size * 2, agent_size * 2, 3))
            # Blue square patch
            source_square[:, :, 2] = 255
            maps.utils.paste_overlapping_image(self.top_down_map, source_square, [banana_x_grid,banana_y_grid])


            # Load cheezit templates from configuration files
            cheezit_template_id = obj_templates_mgr.load_configs(
                str("data/object_datasets/cheezit"))[0]
            # add an object to the scene
            cheezit_id = super().habitat_env.sim.add_object(cheezit_template_id)
            super().habitat_env.sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, cheezit_id)
            cheezit_pos = super().habitat_env.sim.sample_navigable_point()
            super().habitat_env.sim.set_translation(cheezit_pos, cheezit_id)
            cheezit_x_grid,cheezit_y_grid = maps.to_grid(realworld_x=cheezit_pos[2],realworld_y=cheezit_pos[0],grid_resolution=[self.top_down_map.shape[0],self.top_down_map.shape[1]],
                                sim=self._env.sim,pathfinder=self._env.sim.pathfinder)
            source_square = np.zeros((agent_size * 2, agent_size * 2, 3))
            # Green square patch 
            source_square[:, :, 1] = 255
            maps.utils.paste_overlapping_image(self.top_down_map, source_square, [cheezit_x_grid,cheezit_y_grid])


            # Load chefcan templates from configuration files
            chefcan_template_id = obj_templates_mgr.load_configs(
                str("data/object_datasets/chefcan"))[0]
            # add an object to the scene
            chefcan_id = super().habitat_env.sim.add_object(chefcan_template_id)
            super().habitat_env.sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, chefcan_id)
            chefcan_pos = super().habitat_env.sim.sample_navigable_point()
            super().habitat_env.sim.set_translation(chefcan_pos, chefcan_id)
            chefcan_x_grid,chefcan_y_grid = maps.to_grid(realworld_x=chefcan_pos[2],realworld_y=chefcan_pos[0],grid_resolution=[self.top_down_map.shape[0],self.top_down_map.shape[1]],
                                sim=self._env.sim,pathfinder=self._env.sim.pathfinder)
            source_square = np.zeros((agent_size * 2, agent_size * 2, 3))
            # Red square patch
            source_square[:, :, 0] = 255
            maps.utils.paste_overlapping_image(self.top_down_map, source_square, [chefcan_x_grid,chefcan_y_grid])
        super().habitat_env.sim.recompute_navmesh(super().habitat_env.sim.pathfinder, navmesh_settings, True)




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
from habitat.core.env import Env
from habitat.core.vector_env import VectorEnv
from habitat.datasets.pointnav.pointnav_dataset import PointNavDatasetV1
from habitat.config.default import get_config as cfg_env
from habitat_baselines.config.default import get_config as cfg_baseline
from PIL import Image
from torchvision import datasets, models, transforms

def make_vec_envs(args):
    envs = construct_envs(args)
    envs = VecPyTorch(envs, args.device)
    return envs

# Adapted from https://github.com/ikostrikov/pytorch-a2c-ppo-acktr-gail/blob/master/a2c_ppo_acktr/envs.py#L159
class VecPyTorch():
    def __init__(self, venv, device):
        self.venv = venv
        self.num_envs = venv.num_envs
        self.observation_space = venv.observation_spaces[0]
        self.action_space = venv.action_spaces[0]
        self.device = device

    def reset(self):
        results = self.venv.reset()
        obs, info = zip(*results)
        obs_=[]
        for scene in range(len(obs)):
            rgb = np.array(obs[scene]['rgb'])
            depth = np.array(obs[scene]['depth'])*255
            obs_.append(np.concatenate((rgb,depth), axis=2))
        obs = torch.from_numpy(np.array(obs_)).float().to(self.device)
        return obs, info

    def step_async(self, actions):
        actions = actions.cpu().numpy()
        self.venv.step_async(actions)

    def step_wait(self):
        results = self.venv.step_wait()
        obs, reward, done, info = zip(*results)
        obs_=[]
        for scene in range(len(obs)):
            obs_.append(obs[scene]['rgb'])
        obs = torch.from_numpy(np.array(obs_)).float().to(self.device)
        reward = torch.from_numpy(reward).float()
        return obs, reward, done, info

    def step(self, actions):
        actions = actions.cpu().numpy()
        results = self.venv.step(actions)
        obs, reward, done, info = zip(*results)
        obs_=[]
        for scene in range(len(obs)):
            rgb = np.array(obs[scene]['rgb'])
            depth = np.array(obs[scene]['depth'])*255
            obs_.append(np.concatenate((rgb,depth), axis=2))
        obs = torch.from_numpy(np.array(obs_)).float().to(self.device)
        reward = torch.from_numpy(np.array(reward)).float()
        return obs, reward, done, info

    def get_rewards(self, inputs):
        reward = self.venv.get_rewards(inputs)
        reward = torch.from_numpy(reward).float()
        return reward

    def get_short_term_goal(self, inputs):
        stg = self.venv.get_short_term_goal(inputs)
        stg = torch.from_numpy(stg).float()
        return stg

    def close(self):
        return self.venv.close()
  
# This function will be subsequentlty used to create threaded environment
def make_env_fn(args, config_env, rank):
    # Dataset 
    dataset = PointNavDatasetV1(config_env.DATASET)
    env = exploration_env(args=args, rank=rank,
                         config_env=config_env, dataset=dataset
                         )
    env.seed(rank)
    return env

def construct_envs(args):
    
    env_configs = []
    args_list = []
    basic_config = cfg_env(config_paths=
                           "habitat-lab/configs/tasks/pointnav.yaml")                       
    basic_config.defrost()
    basic_config.DATASET.SPLIT = args.split
    basic_config.DATASET.DATA_PATH = (
    "data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz")
    basic_config.DATASET.TYPE = "PointNavDataset-v1"
    basic_config.freeze()
    
    scenes = PointNavDatasetV1.get_scenes_to_load(basic_config.DATASET)
    if len(scenes) > 0:
        assert len(scenes) >= args.num_processes, (
            "reduce the number of processes as there "
            "aren't enough number of scenes"
        )
        scene_split_size = int(np.floor(len(scenes) / args.num_processes))

    for i in range(args.num_processes):
        config_env = cfg_env(config_paths=
                           "habitat-lab/configs/tasks/pointnav.yaml")     
        config_env.defrost()
        if len(scenes) > 0:
            config_env.DATASET.CONTENT_SCENES = scenes[
                                                i * scene_split_size: (i + 1) * scene_split_size
                                                ]

        if i < args.num_processes_on_first_gpu:
            gpu_id = 0
        else:
            gpu_id = int((i - args.num_processes_on_first_gpu)
                         // args.num_processes_per_gpu) + args.sim_gpu_id
        gpu_id = min(torch.cuda.device_count() - 1, gpu_id)
        config_env.SIMULATOR.HABITAT_SIM_V0.GPU_DEVICE_ID = gpu_id


        agent_sensors = []
        agent_sensors.append("RGB_SENSOR")
        agent_sensors.append("DEPTH_SENSOR")
        config_env.DATASET.DATA_PATH = (
        "data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz"
        )
        config_env.SIMULATOR.AGENT_0.SENSORS = agent_sensors

        config_env.ENVIRONMENT.MAX_EPISODE_STEPS = args.max_episode_length
        config_env.ENVIRONMENT.ITERATOR_OPTIONS.SHUFFLE = False

        config_env.SIMULATOR.RGB_SENSOR.WIDTH = args.env_frame_width
        config_env.SIMULATOR.RGB_SENSOR.HEIGHT = args.env_frame_height
        config_env.SIMULATOR.RGB_SENSOR.HFOV = args.hfov
        config_env.SIMULATOR.RGB_SENSOR.POSITION = [0, args.camera_height, 0]

        config_env.SIMULATOR.DEPTH_SENSOR.WIDTH = args.env_frame_width
        config_env.SIMULATOR.DEPTH_SENSOR.HEIGHT = args.env_frame_height
        config_env.SIMULATOR.DEPTH_SENSOR.HFOV = args.hfov
        config_env.SIMULATOR.DEPTH_SENSOR.POSITION = [0, args.camera_height, 0]

        config_env.SIMULATOR.TURN_ANGLE = 10
        config_env.DATASET.SPLIT = args.split
        config_env.freeze()
        env_configs.append(config_env)
        args_list.append(args)
    # Create a vector environment for faster training and evaluation 
    envs = VectorEnv(
        make_env_fn=make_env_fn,
        env_fn_args=tuple(
            tuple(
                zip(args_list, env_configs,range(args.num_processes)
                    )
            )
        ),
    )
    # Convert the data generated by the environment into pytorch tensors 
    envs = VecPyTorch(envs, args.device)
    return envs

def create_habitat_env(args, rank):
        config_env = cfg_env(config_paths=
                           "habitat-lab/configs/tasks/pointnav.yaml")     
        config_env.defrost()
        agent_sensors = []
        agent_sensors.append("RGB_SENSOR")
        agent_sensors.append("DEPTH_SENSOR")
        config_env.DATASET.DATA_PATH = (
        "data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz"
        )
        config_env.SIMULATOR.AGENT_0.SENSORS = agent_sensors

        config_env.ENVIRONMENT.MAX_EPISODE_STEPS = args.max_episode_length
        config_env.ENVIRONMENT.ITERATOR_OPTIONS.SHUFFLE = False

        config_env.SIMULATOR.RGB_SENSOR.WIDTH = args.env_frame_width
        config_env.SIMULATOR.RGB_SENSOR.HEIGHT = args.env_frame_height
        config_env.SIMULATOR.RGB_SENSOR.HFOV = args.hfov
        config_env.SIMULATOR.RGB_SENSOR.POSITION = [0, args.camera_height, 0]

        config_env.SIMULATOR.DEPTH_SENSOR.WIDTH = args.env_frame_width
        config_env.SIMULATOR.DEPTH_SENSOR.HEIGHT = args.env_frame_height
        config_env.SIMULATOR.DEPTH_SENSOR.HFOV = args.hfov
        config_env.SIMULATOR.DEPTH_SENSOR.POSITION = [0, args.camera_height, 0]

        config_env.SIMULATOR.TURN_ANGLE = 10
        config_env.DATASET.SPLIT = args.split
        config_env.freeze()
        
        return make_env_fn(args, config_env, rank)

class exploration_env(habitat.RLEnv):
    def __init__(self, args, rank, config_env, dataset):
        if args.visualize:
            plt.ion()
        if args.print_images or args.visualize:
            self.figure, self.ax = plt.subplots(1,3, figsize=(6*16/9, 6),
                                                facecolor="whitesmoke",
                                                num="Thread {}".format(rank))
        self._prev_measure = {
            "distance_to_goal": 0,
        }
        self.args = args
        self.num_actions = 3
        self.rank = rank
        self.timestep = 0
        self.episode_no = 0
        self.banana_pos = np.zeros((5,3))
        self.cheezit_pos = np.zeros((5,3))
        self.chefcan_pos = np.zeros((5,3))
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

    def preprocess_state(self, rgb, depth):
        preprocess_rgb = transforms.Compose([transforms.Resize(256),transforms.CenterCrop(224),transforms.ToTensor(),
                        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),])
        preprocess_depth = transforms.Compose([transforms.Resize(256),transforms.CenterCrop(224),transforms.ToTensor(),])
        rgb = Image.fromarray(rgb.astype(np.uint8))
        rgb = preprocess_rgb(rgb)
        depth = np.squeeze(depth, axis=2)
        depth = Image.fromarray(depth.astype(np.uint8))
        depth = preprocess_depth(depth)
        return rgb, depth

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
        self.timestep = 0
        rgb = np.array(obs['rgb'])
        depth = np.array(obs['depth'])*255
        rgb, depth = self.preprocess_state(rgb, depth)
        state = np.concatenate((rgb,depth), axis=0)
        return state, self.info

    def step(self, action):
        self.timestep += 1
        obs, rew, done, info = super().step(action + 1)
        # Set info
        self.info['time'] = self.timestep
        # Data Logging 
        dump_dir = "{}/data/{}/".format(self.args.dump_location,
                                                self.args.exp_name)
       
        ep_dir = '{}/episodes/{}/{}/'.format(
                        dump_dir, self.rank+1, self.episode_no)
        # Relative goal position
        rel_goal_pos = obs['pointgoal_with_gps_compass']
        # Agents pose in x,y frame 
        agent_x, agent_y, agent_o = self.get_sim_location()
        # Agents pose in map grid image 
        x_grid,y_grid = maps.to_grid(realworld_x=agent_x,realworld_y=agent_y,grid_resolution=[self.top_down_map.shape[0],self.top_down_map.shape[1]],
                                sim=self._env.sim,pathfinder=self._env.sim.pathfinder)
        # Get the Top down map
        self.top_down_map = maps.get_topdown_map_from_sim(self._env.sim,agent_id=0,meters_per_pixel=0.05,draw_border=True)
        self.top_down_map = maps.colorize_topdown_map(top_down_map=self.top_down_map)
        # Draw agent in the map 
        self.top_down_map = maps.draw_agent(self.top_down_map,(x_grid,y_grid),agent_o)  
        # Draw objects in the map
        self.draw_objects_on_map()

        self.draw_goal_on_map(rel_goal_pos)

        if not os.path.exists(ep_dir):
            os.makedirs(ep_dir)  
        if self.args.print_images or self.args.visualize:    
            vu.visualize(self.figure, self.ax, obs['rgb'], obs['depth'],self.top_down_map,
                            dump_dir, self.rank, self.episode_no,
                            self.timestep, self.args.visualize,
                            self.args.print_images,[self.args.dcropx,self.args.dcropy],self.args.task)                    
        
        rgb = np.array(obs['rgb'])
        depth = np.array(obs['depth'])*255
        rgb, depth = self.preprocess_state(rgb, depth)
        state = np.concatenate((rgb,depth), axis=0)
        return state, rew, done, self.info

    def get_goal_location(self, rel_goal_pos):
        agent_x, agent_y, agent_o = self.get_sim_location()
        #Polar coordinate relative to the agent position 
        r = rel_goal_pos[0]
        if rel_goal_pos[1] < 0:
            theta = (2*np.pi + rel_goal_pos[1])
        else:
            theta =  rel_goal_pos[1]  
        theta = theta + agent_o      
        # Converting into cartesian format
        goal_x = r * np.cos(theta) + agent_x
        goal_y = r * np.sin(theta) + agent_y
        return goal_x, goal_y

    def draw_goal_on_map(self, rel_goal_pos):
        goal_x, goal_y = self.get_goal_location(rel_goal_pos)
         # Draw goal in the map
        agent_size = int(self.top_down_map.shape[0] / 30) * 1
        source_square = np.zeros((agent_size * 2, agent_size * 2, 3))
        #square patch 
        source_square[:, :, 1] = 255
        source_square[:, :, 2] = 128
        goal_pos_x_grid,goal_pos_y_grid = maps.to_grid(realworld_x=goal_x,realworld_y=goal_y,grid_resolution=[self.top_down_map.shape[0],self.top_down_map.shape[1]],
                                sim=self._env.sim,pathfinder=self._env.sim.pathfinder)
        maps.utils.paste_overlapping_image(self.top_down_map, source_square, [goal_pos_x_grid,goal_pos_y_grid])

    def draw_objects_on_map(self):
        # Agent size based on the scene
        agent_size = int(self.top_down_map.shape[0] / 30) * 1
        for i in range(5):
            banana_x_grid,banana_y_grid = maps.to_grid(realworld_x=self.banana_pos[i][2],realworld_y=self.banana_pos[i][0],grid_resolution=[self.top_down_map.shape[0],self.top_down_map.shape[1]],
                                sim=self._env.sim,pathfinder=self._env.sim.pathfinder)
            blue_source_square = np.zeros((agent_size * 2, agent_size * 2, 3))
            green_source_square = np.zeros((agent_size * 2, agent_size * 2, 3))
            red_source_square = np.zeros((agent_size * 2, agent_size * 2, 3))
            # Blue square patch
            blue_source_square[:, :, 2] = 255
            maps.utils.paste_overlapping_image(self.top_down_map, blue_source_square, [banana_x_grid,banana_y_grid])


            cheezit_x_grid,cheezit_y_grid = maps.to_grid(realworld_x=self.cheezit_pos[i][2],realworld_y=self.cheezit_pos[i][0],grid_resolution=[self.top_down_map.shape[0],self.top_down_map.shape[1]],
                                sim=self._env.sim,pathfinder=self._env.sim.pathfinder)
            # Green square patch 
            green_source_square[:, :, 1] = 255
            maps.utils.paste_overlapping_image(self.top_down_map, green_source_square, [cheezit_x_grid,cheezit_y_grid])


            chefcan_x_grid,chefcan_y_grid = maps.to_grid(realworld_x=self.chefcan_pos[i][2],realworld_y=self.chefcan_pos[i][0],grid_resolution=[self.top_down_map.shape[0],self.top_down_map.shape[1]],
                                sim=self._env.sim,pathfinder=self._env.sim.pathfinder)
            # Red square patch
            red_source_square[:, :, 0] = 255
            maps.utils.paste_overlapping_image(self.top_down_map, red_source_square, [chefcan_x_grid,chefcan_y_grid])

    def get_reward_range(self):
        # This function is not used, Habitat-RLEnv requires this function
        return (0., 1.0)

    def get_reward(self, observations):
        reward = 0
        rel_goal_pos = observations['pointgoal_with_gps_compass']
        episode_success_reward = self.get_episode_success_reward(rel_goal_pos)
        agent_to_goal_dist_reward = self.get_agent_to_object_dist_reward(observations)
        reward += (agent_to_goal_dist_reward + episode_success_reward)
        return reward
    
    def get_episode_success_reward(self, rel_goal_pos):
        agent_x, agent_y, agent_o = self.get_sim_location()
        goal_x, goal_y = self.get_goal_location(rel_goal_pos)
        reward = 0
        if (np.sqrt((goal_x - agent_x)**2 +(goal_y - agent_y)**2) < 0.2):
            reward = 10
        return reward
    
    def get_agent_to_object_dist_reward(self, observations):
        """
        Encourage the agent to move towards the goal.
        """
        curr_metric = self._env.get_metrics()["distance_to_goal"]
        prev_metric = self._prev_measure["distance_to_goal"]
        dist_reward = prev_metric - curr_metric
        self._prev_measure["distance_to_goal"] = curr_metric
        return np.sum(dist_reward)# this has to be changed

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
       
        for i in range(5):
            # Load banana templates from configuration files
            banana_template_id = obj_templates_mgr.load_configs(
                str("data/object_datasets/banana"))[0]
            # add an object to the scene
            banana_id = super().habitat_env.sim.add_object(banana_template_id)
            super().habitat_env.sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, banana_id)
            self.banana_pos[i] = super().habitat_env.sim.sample_navigable_point()
            super().habitat_env.sim.set_translation(self.banana_pos[i], banana_id)


            # Load cheezit templates from configuration files
            cheezit_template_id = obj_templates_mgr.load_configs(
                str("data/object_datasets/cheezit"))[0]
            # add an object to the scene
            cheezit_id = super().habitat_env.sim.add_object(cheezit_template_id)
            super().habitat_env.sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, cheezit_id)
            self.cheezit_pos[i] = super().habitat_env.sim.sample_navigable_point()
            super().habitat_env.sim.set_translation(self.cheezit_pos[i], cheezit_id)


            # Load chefcan templates from configuration files
            chefcan_template_id = obj_templates_mgr.load_configs(
                str("data/object_datasets/chefcan"))[0]
            # add an object to the scene
            chefcan_id = super().habitat_env.sim.add_object(chefcan_template_id)
            super().habitat_env.sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, chefcan_id)
            self.chefcan_pos[i] = super().habitat_env.sim.sample_navigable_point()
            super().habitat_env.sim.set_translation(self.chefcan_pos[i], chefcan_id)
        super().habitat_env.sim.recompute_navmesh(super().habitat_env.sim.pathfinder, navmesh_settings, True)




import habitat
from arguments import get_args
from env import exploration_env
from habitat.config.default import get_config as cfg_env
from habitat.core.env import Env
from habitat.datasets.pointnav.pointnav_dataset import PointNavDatasetV1
import os
import time


# This function will be subsequentlty used to create threaded environment
def make_env_fn(args, config_env, rank):
    config_env.defrost()
    #Agent Configuration 
    agent_sensors = []
    agent_sensors.append("RGB_SENSOR")
    agent_sensors.append("DEPTH_SENSOR")
    config_env.DATASET.DATA_PATH = (
        "data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz"
        )
    config_env.SIMULATOR.AGENT_0.SENSORS = agent_sensors
    config_env.SIMULATOR.RGB_SENSOR.WIDTH = args.env_frame_width
    config_env.SIMULATOR.RGB_SENSOR.HEIGHT = args.env_frame_height
    config_env.SIMULATOR.RGB_SENSOR.HFOV = args.hfov
    config_env.SIMULATOR.RGB_SENSOR.POSITION = [0, args.camera_height, 0]
    config_env.SIMULATOR.DEPTH_SENSOR.WIDTH = args.env_frame_width
    config_env.SIMULATOR.DEPTH_SENSOR.HEIGHT = args.env_frame_height
    config_env.SIMULATOR.DEPTH_SENSOR.HFOV = args.hfov
    config_env.SIMULATOR.DEPTH_SENSOR.POSITION = [0, args.camera_height, 0]
    config_env.SIMULATOR.TURN_ANGLE = 10
    #Environment Configuration
    config_env.ENVIRONMENT.MAX_EPISODE_STEPS = args.max_episode_length
    config_env.ENVIRONMENT.ITERATOR_OPTIONS.SHUFFLE = False
    
    print("fetching dataset ...")
    config_env.DATASET.SPLIT = args.split
    config_env.freeze()
    # Dataset 
    dataset = PointNavDatasetV1(config_env.DATASET)
    env = exploration_env(args=args, rank=rank,
                         config_env=config_env, dataset=dataset
                         )
    env.seed(rank)
    return env

if __name__ == "__main__":
    args = get_args()
    config_env = cfg_env("habitat-lab/configs/tasks/pointnav.yaml")
    rank = 1
    env = make_env_fn(args, config_env, rank)
    obs,info = env.reset()

    # Step through environment with random actions
    if args.task=="generate_train":
        for i in range(10000):
            os.system('clear')
            print("time step - {}".format(i))
            obs, rew, done, info= env.step(env.action_space.sample())  
        print("generated training batch")
    if args.task=="milestone1":
        for i in range(1):
            os.system('clear')
            print("time step - {}".format(i))
            obs, rew, done, info= env.step(env.action_space.sample())
        print("generated image for milestone 1")
   





    
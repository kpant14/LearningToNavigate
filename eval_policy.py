import copy
import time
from collections import deque

import os

os.environ["OMP_NUM_THREADS"] = "1"
import numpy as np
import torch
import torch.nn as nn
from torch.nn import functional as F

import gym
import logging
from arguments import get_args
from envs import make_vec_envs
from utils.storage import FIFOMemory
from utils.optimization import get_optimizer
from model import Local_IL_Policy, Neural_SLAM_Module
from PIL import Image
from torchvision import transforms

import sys
import matplotlib

from ppo_utils import PPO, Policy

if sys.platform == 'darwin':
    matplotlib.use("tkagg")
import matplotlib.pyplot as plt

args = get_args()

np.random.seed(args.seed)
torch.manual_seed(args.seed)

if args.cuda:
    torch.cuda.manual_seed(args.seed)


def get_local_map_boundaries(agent_loc, local_sizes, full_sizes):
    loc_r, loc_c = agent_loc
    local_w, local_h = local_sizes
    full_w, full_h = full_sizes

    if args.global_downscaling > 1:
        gx1, gy1 = loc_r - local_w // 2, loc_c - local_h // 2
        gx2, gy2 = gx1 + local_w, gy1 + local_h
        if gx1 < 0:
            gx1, gx2 = 0, local_w
        if gx2 > full_w:
            gx1, gx2 = full_w - local_w, full_w

        if gy1 < 0:
            gy1, gy2 = 0, local_h
        if gy2 > full_h:
            gy1, gy2 = full_h - local_h, full_h
    else:
        gx1, gx2, gy1, gy2 = 0, full_w, 0, full_h

    return [gx1, gx2, gy1, gy2]


def main():
    # Setup Logging
    log_dir = "{}/models/{}/".format(args.dump_location, args.exp_name)
    dump_dir = "{}/dump/{}/".format(args.dump_location, args.exp_name)

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    if not os.path.exists("{}/images/".format(dump_dir)):
        os.makedirs("{}/images/".format(dump_dir))

    logging.basicConfig(
        filename=log_dir + 'train.log',
        level=logging.INFO)
    print("Dumping at {}".format(log_dir))
    print(args)
    logging.info(args)

    # Logging and loss variables
    num_scenes = args.num_processes
    num_episodes = int(args.num_episodes)
    device = args.device = torch.device("cuda:0" if args.cuda else "cpu")
    policy_loss = 0



    l_masks = torch.zeros(num_scenes).float().to(device)


    # Starting environments
    torch.set_num_threads(1)
    envs = make_vec_envs(args)
    obs, infos = envs.reset()
    # Initialize map variables
    ### Full map consists of 4 channels containing the following:
    ### 1. Obstacle Map
    ### 2. Exploread Area
    ### 3. Current Agent Location
    ### 4. Past Agent Locations

    torch.set_grad_enabled(False)

    # Calculating full and local map sizes
    map_size = args.map_size_cm // args.map_resolution
    full_w, full_h = map_size, map_size
    local_w, local_h = int(full_w / args.global_downscaling), \
                       int(full_h / args.global_downscaling)

    # Initializing full and local map
    full_map = torch.zeros(num_scenes, 4, full_w, full_h).float().to(device)
    local_map = torch.zeros(num_scenes, 4, local_w, local_h).float().to(device)

    # Initial full and local pose
    full_pose = torch.zeros(num_scenes, 3).float().to(device)
    local_pose = torch.zeros(num_scenes, 3).float().to(device)

    rgb_obs = torch.zeros(num_scenes,3,args.frame_width, args.frame_height).to(device)
    test_recurrent_hidden_states = torch.zeros(
        num_scenes, 512, device=device
    )
    not_done_masks = torch.zeros(num_scenes, 1, device=device)
    episode_spls = torch.zeros(num_scenes, 1, device=device)
    episode_success = torch.zeros(num_scenes, 1, device=device)
    episode_counts = torch.zeros(num_scenes, 1, device=device)
    step_counts = torch.zeros(num_scenes, 1, device=device)
    # Origin of local map
    origins = np.zeros((num_scenes, 3))

    # Local Map Boundaries
    lmb = np.zeros((num_scenes, 4)).astype(int)

    ### Planner pose inputs has 7 dimensions
    ### 1-3 store continuous global agent location
    ### 4-7 store local map boundaries
    planner_pose_inputs = np.zeros((num_scenes, 7))

    def init_map_and_pose():
        full_map.fill_(0.)
        full_pose.fill_(0.)
        full_pose[:, :2] = args.map_size_cm / 100.0 / 2.0

        locs = full_pose.cpu().numpy()
        planner_pose_inputs[:, :3] = locs
        for e in range(num_scenes):
            r, c = locs[e, 1], locs[e, 0]
            loc_r, loc_c = [int(r * 100.0 / args.map_resolution),
                            int(c * 100.0 / args.map_resolution)]

            full_map[e, 2:, loc_r - 1:loc_r + 2, loc_c - 1:loc_c + 2] = 1.0

            lmb[e] = get_local_map_boundaries((loc_r, loc_c),
                                              (local_w, local_h),
                                              (full_w, full_h))

            planner_pose_inputs[e, 3:] = lmb[e]
            origins[e] = [lmb[e][2] * args.map_resolution / 100.0,
                          lmb[e][0] * args.map_resolution / 100.0, 0.]

        for e in range(num_scenes):
            local_map[e] = full_map[e, :, lmb[e, 0]:lmb[e, 1], lmb[e, 2]:lmb[e, 3]]
            local_pose[e] = full_pose[e] - \
                            torch.from_numpy(origins[e]).to(device).float()

    def init_map_and_pose_at(env_idx):
        full_map[env_idx].fill_(0.)
        full_pose[env_idx].fill_(0.)
        full_pose[env_idx, :2] = args.map_size_cm / 100.0 / 2.0

        locs = full_pose.cpu().numpy()
        planner_pose_inputs[env_idx, :3] = locs[env_idx]
        
        r, c = locs[env_idx, 1], locs[env_idx, 0]
        loc_r, loc_c = [int(r * 100.0 / args.map_resolution),
                        int(c * 100.0 / args.map_resolution)]

        full_map[env_idx, 2:, loc_r - 1:loc_r + 2, loc_c - 1:loc_c + 2] = 1.0

        lmb[env_idx] = get_local_map_boundaries((loc_r, loc_c),
                                            (local_w, local_h),
                                            (full_w, full_h))

        planner_pose_inputs[env_idx, 3:] = lmb[env_idx]
        origins[env_idx] = [lmb[env_idx][2] * args.map_resolution / 100.0,
                        lmb[env_idx][0] * args.map_resolution / 100.0, 0.]

       
        local_map[env_idx] = full_map[env_idx, :, lmb[env_idx, 0]:lmb[env_idx, 1], lmb[env_idx, 2]:lmb[env_idx, 3]]
        local_pose[env_idx] = full_pose[env_idx] - \
                        torch.from_numpy(origins[env_idx]).to(device).float()                        

    init_map_and_pose()

    # Local policy observation space
    l_observation_space = gym.spaces.Box(0, 255,
                                         (3,
                                          args.frame_width,
                                          args.frame_width), dtype='uint8')

    # Local and Global policy recurrent layer sizes
    l_hidden_size = args.local_hidden_size


    # slam
    nslam_module = Neural_SLAM_Module(args).to(device)



    # Local policy
    l_policy = Local_IL_Policy(l_observation_space.shape, 3,
                               recurrent=args.use_recurrent_local,
                               hidden_size=l_hidden_size,
                               deterministic=args.use_deterministic_local).to(device)


    actor_critic = Policy(
        observation_space=envs.observation_space,
        action_space=envs.action_space,
        hidden_size=512,
        goal_sensor_uuid='pointgoal_with_gps_compass',
    )
    actor_critic.to(device)

    ppo = PPO(
        actor_critic=actor_critic,
        clip_param=0.1,
        ppo_epoch=4,
        num_mini_batch=32,
        value_loss_coef=0.5,
        entropy_coef=0.01,
        lr=2.5e-4,
        eps=1e-5,
        max_grad_norm=0.5,
    )                   

    # Loading model
    if args.load_slam != "0":
        print("Loading slam {}".format(args.load_slam))
        state_dict = torch.load(args.load_slam,
                                map_location=lambda storage, loc: storage)
        nslam_module.load_state_dict(state_dict)

    if not args.train_slam:
        nslam_module.eval()


    if args.load_local != "0":
        print("Loading local {}".format(args.load_local))
        state_dict = torch.load(args.load_local,
                                map_location=lambda storage, loc: storage)
        l_policy.load_state_dict(state_dict)

        ckpt = torch.load('habitat_baselines/rgbd.pth', map_location=device)
        state = ckpt["state_dict"]

        reordering = torch.tensor([3, 0, 1, 2], dtype=torch.long)
        for k in [
            "actor_critic.action_distribution.linear.weight",
            "actor_critic.action_distribution.linear.bias",
        ]:
            state[k] = state[k][reordering]


        ckpt["state_dict"] = state
        ppo.load_state_dict(ckpt["state_dict"])

    if not args.train_local:
        l_policy.eval()

    # Predict map from frame 1:
    poses = torch.from_numpy(np.asarray(
        [infos[env_idx]['sensor_pose'] for env_idx
         in range(num_scenes)])
    ).float().to(device)

    resnet = transforms.Compose([transforms.ToPILImage(),
                    transforms.Resize((args.frame_height, args.frame_width),
                                      interpolation = Image.NEAREST)])

    for e in range(num_scenes):
        rgb_obs[e] = torch.from_numpy(np.asarray(resnet(obs["rgb"][e].cpu().numpy().astype(np.uint8))).transpose(2, 0, 1)).to(device) 

    _, _, local_map[:, 0, :, :], local_map[:, 1, :, :], _, local_pose = \
        nslam_module(rgb_obs, rgb_obs, poses, local_map[:, 0, :, :],
                     local_map[:, 1, :, :], local_pose)

    # Compute PPO policy input
    locs = local_pose.cpu().numpy()
    orientation = torch.zeros(num_scenes, 1).long()
    goals = np.zeros((num_scenes, 2))
    #ppo_input = torch.zeros(num_scenes, 4, args.frame_width, args.frame_height)

    for e in range(num_scenes):
        r, c = locs[e, 1], locs[e, 0]
        loc_r, loc_c = [int(r * 100.0 / args.map_resolution),
                        int(c * 100.0 / args.map_resolution)]

        local_map[e, 2:, loc_r - 1:loc_r + 2, loc_c - 1:loc_c + 2] = 1.
        orientation[e] = int((locs[e, 2] + 180.0) / 5.)
       
        # Polar coordinate relative to the agent position 
        r_goal = infos[e]['goal_location'][0]
        # Adding agents angle with the relative angle to get the absolute angle. 
        theta_goal = infos[e]['goal_location'][1] + np.radians(locs[e,2])
        # Converting into cartesian format
        z_coordinate = r_goal * np.cos(theta_goal)
        x_coordinate = r_goal * np.sin(theta_goal)
        goals[e] = np.array([x_coordinate * 100.0 / args.map_resolution + loc_r, z_coordinate * 100.0 / args.map_resolution + loc_c ])

    # Compute planner inputs
    planner_inputs = [{} for e in range(num_scenes)]
    for e, p_input in enumerate(planner_inputs):
        p_input['goal'] = goals[e].astype(int)
        p_input['map_pred'] = local_map[e, 0, :, :].cpu().numpy()
        p_input['exp_pred'] = local_map[e, 1, :, :].cpu().numpy()
        p_input['pose_pred'] = planner_pose_inputs[e]

    # Output stores local goals as well as the the ground-truth action
    output = envs.get_short_term_goal(planner_inputs)

    last_obs = rgb_obs.detach()
    local_rec_states = torch.zeros(num_scenes, l_hidden_size).to(device)
    start = time.time()

    total_num_steps = -1
    torch.set_grad_enabled(False)

    splfile = open("{}/spl.txt".format(dump_dir), "w+")
    # for ep_num in range(num_episodes):
    #     for step in range(args.max_episode_length):
    while episode_counts.sum() < 994:
        total_num_steps += 1
        #l_step = step % args.num_local_steps
        # ------------------------------------------------------------------
        # Local Policy
        del last_obs
        last_obs = rgb_obs.detach()
        local_masks = l_masks
        local_goals = output[:, :3].to(device).long()

        if args.train_local:
            torch.set_grad_enabled(True)

        action, action_prob, local_rec_states = l_policy(
            rgb_obs,
            local_rec_states,
            local_masks,
            extras=local_goals,
        )

        if args.train_local:
            action_target = output[:, :3].long().to(device)
            policy_loss += nn.CrossEntropyLoss()(action_prob, action_target)
            torch.set_grad_enabled(False)
        l_action = action.cpu().numpy()
        # ------------------------------------------------------------------
        with torch.no_grad():
            _, ppo_actions, _, test_recurrent_hidden_states = actor_critic.act(
                obs,
                test_recurrent_hidden_states,
                not_done_masks,
                deterministic=False,
            )
        if args.agent == "ppo" or args.agent == "ppo_st":     
            obs, rew, done, infos = envs.step([a[0].item() for a in ppo_actions])
        # ------------------------------------------------------------------
        #Env step
        for e in range(num_scenes):
            # Action remapping
            if l_action[e] == 2: # Forward
                l_action[e] = 1
            elif l_action[e] == 1: # Right
                l_action[e] = 3
            elif l_action[e] == 0: # Left
                l_action[e] = 2
        if (args.agent == "ans"):
            obs, rew, done, infos = envs.step(l_action)
        for e in range(num_scenes):
            rgb_obs[e] = torch.from_numpy(np.asarray(resnet(obs["rgb"][e].cpu().numpy().astype(np.uint8))).transpose(2, 0, 1)).to(device)
            if args.agent == "ppo_st":   
                if (obs["pointgoal_with_gps_compass"][e][0] > 5):
                    obs["pointgoal_with_gps_compass"][e] = output[e][3:5]
                    obs["pointgoal_with_gps_compass"][e][0]+=0.5
        if args.eval:
            for e in range(num_scenes):
                step_counts[e] +=1
                if (infos[e]['success'] or done[e]):
                    step_counts[e] = 0
                    splfile.write(str(infos[e]['spl']) + '\t' + str(infos[e]['success']) + "\n")
                    splfile.flush() 
                    episode_spls[e] += infos[e]["spl"]
                    if infos[e]["spl"] > 0:
                        episode_success[e] += 1
                    # ------------------------------------------------------------------
                    # Reinitialize variables when episode ends
                    init_map_and_pose_at(e)
                    last_obs[e] = rgb_obs[e].detach()              
        l_masks = torch.FloatTensor([0 if x else 1
                                        for x in done]).to(device)
        not_done_masks = torch.tensor([[0.0] if done_ else [1.0] for done_ in done],dtype=torch.float,device=device,)
        episode_counts += 1 - not_done_masks

        poses = torch.from_numpy(np.asarray(
            [infos[env_idx]['sensor_pose'] for env_idx
                in range(num_scenes)])
        ).float().to(device)

        _, _, local_map[:, 0, :, :], local_map[:, 1, :, :], _, local_pose = \
            nslam_module(last_obs[:,0:3,:,:], rgb_obs, poses, local_map[:, 0, :, :],
                            local_map[:, 1, :, :], local_pose, build_maps=True)

        locs = local_pose.cpu().numpy()
        planner_pose_inputs[:, :3] = locs + origins
        local_map[:, 2, :, :].fill_(0.)  # Resetting current location channel
        for e in range(num_scenes):
            r, c = locs[e, 1], locs[e, 0]
            loc_r, loc_c = [int(r * 100.0 / args.map_resolution),
                            int(c * 100.0 / args.map_resolution)]

            local_map[e, 2:, loc_r - 2:loc_r + 3, loc_c - 2:loc_c + 3] = 1.
            # Polar coordinate relative to the agent position 
            r_goal = infos[e]['goal_location'][0]
            # Adding agents angle with the relative angle to get the absolute angle. 
            theta_goal = infos[e]['goal_location'][1] + np.radians(locs[e,2])
            # Converting into cartesian format
            z_coordinate = r_goal * np.cos(theta_goal)
            x_coordinate = r_goal * np.sin(theta_goal)
            goals[e] = np.array([x_coordinate * 100.0 / args.map_resolution + loc_r, z_coordinate * 100.0 / args.map_resolution + loc_c ])  
        # ------------------------------------------------------------------
        # ------------------------------------------------------------------
        # Change the maps to make agent in the center
       
        # For every global step, update the full and local maps
        for e in range(num_scenes):
            if step_counts[e]%args.num_local_steps == args.num_local_steps - 1:
                full_map[e, :, lmb[e, 0]:lmb[e, 1], lmb[e, 2]:lmb[e, 3]] = \
                    local_map[e]
                full_pose[e] = local_pose[e] + \
                                torch.from_numpy(origins[e]).to(device).float()

                locs = full_pose[e].cpu().numpy()
                r, c = locs[1], locs[0]
                loc_r, loc_c = [int(r * 100.0 / args.map_resolution),
                                int(c * 100.0 / args.map_resolution)]

                lmb[e] = get_local_map_boundaries((loc_r, loc_c),
                                                    (local_w, local_h),
                                                    (full_w, full_h))

                planner_pose_inputs[e, 3:] = lmb[e]
                origins[e] = [lmb[e][2] * args.map_resolution / 100.0,
                                lmb[e][0] * args.map_resolution / 100.0, 0.]

                local_map[e] = full_map[e, :,
                                lmb[e, 0]:lmb[e, 1], lmb[e, 2]:lmb[e, 3]]
                local_pose[e] = full_pose[e] - \
                                torch.from_numpy(origins[e]).to(device).float()

        locs = local_pose.cpu().numpy()
        for e in range(num_scenes):
            if step_counts[e]%args.num_local_steps == args.num_local_steps - 1:
                r, c = locs[e, 1], locs[e, 0]
                loc_r, loc_c = [int(r * 100.0 / args.map_resolution),
                                int(c * 100.0 / args.map_resolution)]

                local_map[e, 2:, loc_r - 1:loc_r + 2, loc_c - 1:loc_c + 2] = 1.
                orientation[e] = int((locs[e, 2] + 180.0) / 5.)
            
                # Polar coordinate relative to the agent position 
                r_goal = infos[e]['goal_location'][0]
                # Adding agents angle with the relative angle to get the absolute angle. 
                theta_goal = infos[e]['goal_location'][1] + np.radians(locs[e,2])
                # Converting into cartesian format
                z_coordinate = r_goal * np.cos(theta_goal)
                x_coordinate = r_goal * np.sin(theta_goal)
                goals[e] = np.array([x_coordinate * 100.0 / args.map_resolution + loc_r, z_coordinate * 100.0 / args.map_resolution + loc_c ])    

        # ------------------------------------------------------------------
        # Get short term goal
        planner_inputs = [{} for e in range(num_scenes)]
        for e, p_input in enumerate(planner_inputs):
            p_input['map_pred'] = local_map[e, 0, :, :].cpu().numpy()
            p_input['exp_pred'] = local_map[e, 1, :, :].cpu().numpy()
            p_input['pose_pred'] = planner_pose_inputs[e]
            p_input['goal'] = goals[e].astype(int)
        output = envs.get_short_term_goal(planner_inputs)
        # ------------------------------------------------------------------
        

        # ------------------------------------------------------------------
        # Logging
        if total_num_steps % args.log_interval == 0:
            end = time.time()
            time_elapsed = time.gmtime(end - start)
            log = " ".join([
                "Time: {0:0=2d}d".format(time_elapsed.tm_mday - 1),
                "{},".format(time.strftime("%Hh %Mm %Ss", time_elapsed)),
                "num timesteps {},".format(total_num_steps *
                                            num_scenes),
                "FPS {},".format(int(total_num_steps * num_scenes \
                                        / (end - start)))
            ])
            print(log)
            logging.info(log)
            # ------------------------------------------------------------------

    episode_spl_mean = (episode_spls / episode_counts).mean().item()
    episode_success_mean = (episode_success / episode_counts).mean().item()
    print("Average episode success: {:.6f}".format(episode_success_mean))
    print("Average episode spl: {:.6f}".format(episode_spl_mean))
       
    splfile.close()


if __name__ == "__main__":
    main()

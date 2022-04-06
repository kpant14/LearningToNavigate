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
from utils.storage import GlobalRolloutStorage, FIFOMemory
from utils.optimization import get_optimizer
from model import RL_Policy, Local_IL_Policy, Neural_SLAM_Module

import algo

import sys
import matplotlib

if sys.platform == 'darwin':
    matplotlib.use("tkagg")
import matplotlib.pyplot as plt

# plt.ion()
# fig, ax = plt.subplots(1,4, figsize=(10, 2.5), facecolor="whitesmoke")


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

    best_cost = 100000
    costs = deque(maxlen=1000)
    exp_costs = deque(maxlen=1000)
    pose_costs = deque(maxlen=1000)

    l_masks = torch.zeros(num_scenes).float().to(device)
    ppo_masks = torch.zeros(num_scenes).float().to(device)

    best_local_loss = np.inf
    best_ppo_reward = -np.inf

    if args.eval:
        traj_lengths = args.max_episode_length // args.num_local_steps
        #explored_area_log = np.zeros((num_scenes, num_episodes, traj_lengths))
        #explored_ratio_log = np.zeros((num_scenes, num_episodes, traj_lengths))

    ppo_episode_rewards = deque(maxlen=1000)
    ppo_value_losses = deque(maxlen=1000)
    ppo_action_losses = deque(maxlen=1000)
    ppo_dist_entropies = deque(maxlen=1000)

    per_step_ppo_rewards = deque(maxlen=1000)

    ppo_process_rewards = np.zeros((num_scenes))

    l_action_losses = deque(maxlen=1000)


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
    ppo_hidden_size = args.global_hidden_size

     # PPO policy observation space
    ppo_observation_space = gym.spaces.Box(0, 255,
                                         (4,args.frame_width,
                                          args.frame_height), dtype='uint8')

    # Global policy action space
    ppo_action_space = gym.spaces.Box(low=0.0, high=1.0,
                                    shape=(3,), dtype=np.float32)


    # Local policy observation space
    l_observation_space = gym.spaces.Box(0, 255,
                                         (3,
                                          args.frame_width,
                                          args.frame_width), dtype='uint8')


    # slam
    nslam_module = Neural_SLAM_Module(args).to(device)
    slam_optimizer = get_optimizer(nslam_module.parameters(),
                                   args.slam_optimizer)

    

    # Local policy
    l_policy = Local_IL_Policy(l_observation_space.shape, envs.action_space.n,
                               recurrent=args.use_recurrent_local,
                               hidden_size=l_hidden_size,
                               deterministic=args.use_deterministic_local).to(device)
    local_optimizer = get_optimizer(l_policy.parameters(),
                                    args.local_optimizer)

    # Global policy
    ppo_policy = RL_Policy(ppo_observation_space.shape, ppo_action_space,
                         base_kwargs={'recurrent': args.use_recurrent_global,
                                      'hidden_size': ppo_hidden_size,
                                      'downscaling': args.global_downscaling
                                      }).to(device)
    ppo_agent = algo.PPO(ppo_policy, args.clip_param, args.ppo_epoch,
                       args.num_mini_batch, args.value_loss_coef,
                       args.entropy_coef, lr=args.global_lr, eps=args.eps,
                       max_grad_norm=args.max_grad_norm)

    slam_memory = FIFOMemory(args.slam_memory_size)
    # Storage
    ppo_rollouts = GlobalRolloutStorage(args.num_global_steps,
                                      num_scenes, ppo_observation_space.shape,
                                      ppo_action_space, ppo_policy.rec_state_size,
                                      1).to(device)

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
    if args.load_global != "0":
        print("Loading ppo {}".format(args.load_global))
        state_dict = torch.load(args.load_global,
                                map_location=lambda storage, loc: storage)
        ppo_policy.load_state_dict(state_dict)    

    if not args.train_local:
        l_policy.eval()

    # Predict map from frame 1:
    poses = torch.from_numpy(np.asarray(
        [infos[env_idx]['sensor_pose'] for env_idx
         in range(num_scenes)])
    ).float().to(device)

    _, _, local_map[:, 0, :, :], local_map[:, 1, :, :], _, local_pose = \
        nslam_module(obs[:,0:3,:,:], obs[:,0:3,:,:], poses, local_map[:, 0, :, :],
                     local_map[:, 1, :, :], local_pose)

    # Compute PPO policy input
    locs = local_pose.cpu().numpy()
    orientation = torch.zeros(num_scenes, 1).long()
    goals = np.zeros((num_scenes, 2))
    ppo_input = torch.zeros(num_scenes, 4, args.frame_width, args.frame_height)

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
    ppo_input = obs.float().to(device)
    ppo_rollouts.obs[0].copy_(ppo_input)

    # Run PPO Policy (ppo_goals = navigation actions)
    ppo_value, ppo_action, ppo_action_log_prob, ppo_rec_states = \
        ppo_policy.act(
            ppo_rollouts.obs[0],
            ppo_rollouts.rec_states[0],
            ppo_rollouts.masks[0],
            extras=ppo_rollouts.extras[0],
            deterministic=False
        )

    # Compute planner inputs
    planner_inputs = [{} for e in range(num_scenes)]
    for e, p_input in enumerate(planner_inputs):
        p_input['goal'] = goals[e].astype(int)
        p_input['map_pred'] = local_map[e, 0, :, :].cpu().numpy()
        p_input['exp_pred'] = local_map[e, 1, :, :].cpu().numpy()
        p_input['pose_pred'] = planner_pose_inputs[e]

    # Output stores local goals as well as the the ground-truth action
    output = envs.get_short_term_goal(planner_inputs)

    last_obs = obs.detach()
    local_rec_states = torch.zeros(num_scenes, l_hidden_size).to(device)
    start = time.time()

    total_num_steps = -1
    g_reward = 0
    ppo_reward = torch.zeros(num_scenes, 1).to(device)
    torch.set_grad_enabled(False)


    splfile = open("{}/spl.txt".format(dump_dir), "w+")
    for ep_num in range(num_episodes):
        for step in range(args.max_episode_length):
            total_num_steps += 1
            ppo_step = step % args.num_global_steps
            l_step = step % args.num_local_steps

             # Sample actions from PPO policy
            ppo_value, ppo_action, ppo_action_log_prob, ppo_rec_states = \
                ppo_policy.act(
                    ppo_rollouts.obs[ppo_step + 1],
                    ppo_rollouts.rec_states[ppo_step + 1],
                    ppo_rollouts.masks[ppo_step + 1],
                    extras=ppo_rollouts.extras[ppo_step + 1],
                    deterministic=False
                )
            ppo_actions = nn.Sigmoid()(ppo_action)
            ppo_actions = torch.argmax(ppo_actions, dim = 1)   


            # ------------------------------------------------------------------
            # Local Policy
            del last_obs
            last_obs = obs.detach()
            local_masks = l_masks
            local_goals = output[:, :-1].to(device).long()

            if args.train_local:
                torch.set_grad_enabled(True)

            action, action_prob, local_rec_states = l_policy(
                obs[:,0:3,:,:],
                local_rec_states,
                local_masks,
                extras=local_goals,
            )

            if args.train_local:
                action_target = output[:, -1].long().to(device)
                policy_loss += nn.CrossEntropyLoss()(action_prob, action_target)
                torch.set_grad_enabled(False)
            l_action = action.cpu()
            # ------------------------------------------------------------------

            # ------------------------------------------------------------------
            # Env step
            if args.eval:
                for e in range(num_scenes):
                    if (infos[e]['distance_to_goal'] < 0.36):
                        l_action[e] = 3    
                        ppo_actions[e] = 3 
            obs, rew, done, infos = envs.step(l_action)
            #obs, rew, done, infos = envs.step(ppo_actions)
            if args.eval:
                for e in range(num_scenes):
                    if (infos[e]['success'] or done[e]):
                        splfile.write(str(infos[e]['spl']) + '\t' + str(infos[e]['success']) + "\n")
                        splfile.flush()

                        obs_, info = envs.reset_at(e)
                        obs[e] = obs_
                        infos[e] = copy.deepcopy(info[0])  
                        init_map_and_pose_at(e)
                        last_obs[e] = obs[e].detach()  
            l_masks = torch.FloatTensor([0 if x else 1
                                         for x in done]).to(device)
            # ------------------------------------------------------------------
            # PPO Rewards
            # Get exploration reward and metrics
            ppo_reward = torch.from_numpy(np.asarray(
                [rew[env_idx] for env_idx
                    in range(num_scenes)])
            ).float().to(device)

            if args.eval:
                ppo_reward = ppo_reward

            ppo_process_rewards += ppo_reward.cpu().numpy()
            ppo_total_rewards = ppo_process_rewards * \
                                (1 - ppo_masks.cpu().numpy())
            ppo_process_rewards *= ppo_masks.cpu().numpy()
            per_step_ppo_rewards.append(np.mean(ppo_reward.cpu().numpy()))

            if np.sum(ppo_total_rewards) != 0:
                for tr in ppo_total_rewards:
                    ppo_episode_rewards.append(tr) if tr != 0 else None
            
            # Add samples to PPO policy storage
            ppo_rollouts.insert(
                ppo_input, ppo_rec_states,
                ppo_action, ppo_action_log_prob, ppo_value,
                ppo_reward, ppo_masks, orientation
            )
            
            # ------------------------------------------------------------------
            # Reinitialize variables when episode ends
            if not args.eval:
                if step == args.max_episode_length - 1:  # Last episode step
                    init_map_and_pose()
                    del last_obs
                    last_obs = obs.detach()
            # ------------------------------------------------------------------

            # ------------------------------------------------------------------
            # Neural SLAM Module
            if args.train_slam:
                # Add frames to memory
                for env_idx in range(num_scenes):
                    env_obs = obs[env_idx].to("cpu")
                    env_poses = torch.from_numpy(np.asarray(
                        infos[env_idx]['sensor_pose']
                    )).float().to("cpu")
                    env_gt_fp_projs = torch.from_numpy(np.asarray(
                        infos[env_idx]['fp_proj']
                    )).unsqueeze(0).float().to("cpu")
                    env_gt_fp_explored = torch.from_numpy(np.asarray(
                        infos[env_idx]['fp_explored']
                    )).unsqueeze(0).float().to("cpu")
                    env_gt_pose_err = torch.from_numpy(np.asarray(
                        infos[env_idx]['pose_err']
                    )).float().to("cpu")
                    slam_memory.push(
                        (last_obs[env_idx].cpu(), env_obs, env_poses),
                        (env_gt_fp_projs, env_gt_fp_explored, env_gt_pose_err))

            poses = torch.from_numpy(np.asarray(
                [infos[env_idx]['sensor_pose'] for env_idx
                 in range(num_scenes)])
            ).float().to(device)

            _, _, local_map[:, 0, :, :], local_map[:, 1, :, :], _, local_pose = \
                nslam_module(last_obs[:,0:3,:,:], obs[:,0:3,:,:], poses, local_map[:, 0, :, :],
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
            if l_step == args.num_local_steps - 1:
                # For every global step, update the full and local maps
                for e in range(num_scenes):
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

            ### TRAINING
            torch.set_grad_enabled(True)
            # ------------------------------------------------------------------
            # Train Neural SLAM Module
            if args.train_slam and len(slam_memory) > args.slam_batch_size:
                for _ in range(args.slam_iterations):
                    inputs, outputs = slam_memory.sample(args.slam_batch_size)
                    b_obs_last, b_obs, b_poses = inputs
                    gt_fp_projs, gt_fp_explored, gt_pose_err = outputs

                    b_obs = b_obs.to(device)
                    b_obs_last = b_obs_last.to(device)
                    b_poses = b_poses.to(device)

                    gt_fp_projs = gt_fp_projs.to(device)
                    gt_fp_explored = gt_fp_explored.to(device)
                    gt_pose_err = gt_pose_err.to(device)

                    b_proj_pred, b_fp_exp_pred, _, _, b_pose_err_pred, _ = \
                        nslam_module(b_obs_last, b_obs, b_poses,
                                     None, None, None,
                                     build_maps=False)
                    loss = 0
                    if args.proj_loss_coeff > 0:
                        proj_loss = F.binary_cross_entropy(b_proj_pred,
                                                           gt_fp_projs)
                        costs.append(proj_loss.item())
                        loss += args.proj_loss_coeff * proj_loss

                    if args.exp_loss_coeff > 0:
                        exp_loss = F.binary_cross_entropy(b_fp_exp_pred,
                                                          gt_fp_explored)
                        exp_costs.append(exp_loss.item())
                        loss += args.exp_loss_coeff * exp_loss

                    if args.pose_loss_coeff > 0:
                        pose_loss = torch.nn.MSELoss()(b_pose_err_pred,
                                                       gt_pose_err)
                        pose_costs.append(args.pose_loss_coeff *
                                          pose_loss.item())
                        loss += args.pose_loss_coeff * pose_loss

                    if args.train_slam:
                        slam_optimizer.zero_grad()
                        loss.backward()
                        slam_optimizer.step()

                    del b_obs_last, b_obs, b_poses
                    del gt_fp_projs, gt_fp_explored, gt_pose_err
                    del b_proj_pred, b_fp_exp_pred, b_pose_err_pred

            # ------------------------------------------------------------------
            # Train PPO Policy
            if ppo_step % args.num_global_steps == args.num_global_steps - 1: 
                if args.train_global:
                    ppo_next_value = ppo_policy.get_value(
                        ppo_rollouts.obs[-1],
                        ppo_rollouts.rec_states[-1],
                        ppo_rollouts.masks[-1],
                        extras=ppo_rollouts.extras[-1]
                    ).detach()

                    ppo_rollouts.compute_returns(ppo_next_value, args.use_gae,
                                               args.gamma, args.tau)
                    g_value_loss, g_action_loss, g_dist_entropy = \
                        ppo_agent.update(ppo_rollouts)
                    ppo_value_losses.append(g_value_loss)
                    ppo_action_losses.append(g_action_loss)
                    ppo_dist_entropies.append(g_dist_entropy)
                ppo_rollouts.after_update()
            # ------------------------------------------------------------------
            # Train Local Policy
            if (l_step + 1) % args.local_policy_update_freq == 0 \
                    and args.train_local:
                local_optimizer.zero_grad()
                policy_loss.backward()
                local_optimizer.step()
                l_action_losses.append(policy_loss.item())
                policy_loss = 0
                local_rec_states = local_rec_states.detach_()
            # ------------------------------------------------------------------

            # Finish Training
            torch.set_grad_enabled(False)
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

                log += "\n\tRewards:"
                if len(ppo_episode_rewards) > 0:
                    log += " ".join([
                        " Global step mean/med rew:",
                        "{:.4f}/{:.4f},".format(
                            np.mean(per_step_ppo_rewards),
                            np.median(per_step_ppo_rewards)),
                        " Global eps mean/med/min/max eps rew:",
                        "{:.3f}/{:.3f}/{:.3f}/{:.3f},".format(
                            np.mean(ppo_episode_rewards),
                            np.median(ppo_episode_rewards),
                            np.min(ppo_episode_rewards),
                            np.max(ppo_episode_rewards))
                    ])

                log += "\n\tLosses:"

                if args.train_local and len(l_action_losses) > 0:
                    log += " ".join([
                        " Local Loss:",
                        "{:.3f},".format(
                            np.mean(l_action_losses))
                    ])

                if args.train_global and len(ppo_value_losses) > 0:
                    log += " ".join([
                        " Global Loss value/action/dist:",
                        "{:.3f}/{:.3f}/{:.3f},".format(
                            np.mean(ppo_value_losses),
                            np.mean(ppo_action_losses),
                            np.mean(ppo_dist_entropies))
                    ])
                if args.train_slam and len(costs) > 0:
                    log += " ".join([
                        " SLAM Loss proj/exp/pose:"
                        "{:.4f}/{:.4f}/{:.4f}".format(
                            np.mean(costs),
                            np.mean(exp_costs),
                            np.mean(pose_costs))
                    ])

                print(log)
                logging.info(log)
            # ------------------------------------------------------------------

            # ------------------------------------------------------------------
            # Save best models
            if (total_num_steps * num_scenes) % args.save_interval < \
                    num_scenes:

                # Save Neural SLAM Model
                if len(costs) >= 1000 and np.mean(costs) < best_cost \
                        and not args.eval:
                    best_cost = np.mean(costs)
                    torch.save(nslam_module.state_dict(),
                               os.path.join(log_dir, "model_best.slam"))

                # Save Local Policy Model
                if len(l_action_losses) >= 100 and \
                        (np.mean(l_action_losses) <= best_local_loss) \
                        and not args.eval:
                    torch.save(l_policy.state_dict(),
                               os.path.join(log_dir, "model_best.local"))

                    best_local_loss = np.mean(l_action_losses)
                 # Save Global Policy Model
                if len(ppo_episode_rewards) >= 100 and \
                        (np.mean(ppo_episode_rewards) >= best_ppo_reward) \
                        and not args.eval:
                    torch.save(ppo_policy.state_dict(),
                               os.path.join(log_dir, "model_best.ppo"))
                    best_ppo_reward = np.mean(ppo_episode_rewards)

            # Save periodic models
            if (total_num_steps * num_scenes) % args.save_periodic < \
                    num_scenes:
                step = total_num_steps * num_scenes
                if args.train_slam:
                    torch.save(nslam_module.state_dict(),
                               os.path.join(dump_dir,
                                            "periodic_{}.slam".format(step)))
                if args.train_local:
                    torch.save(l_policy.state_dict(),
                               os.path.join(dump_dir,
                                            "periodic_{}.local".format(step)))
            # ------------------------------------------------------------------
    splfile.close()
    # Print and save model performance numbers during evaluation
    if args.eval:
        # logfile = open("{}/explored_area.txt".format(dump_dir), "w+")
        # for e in range(num_scenes):
        #     for i in range(explored_area_log[e].shape[0]):
        #         logfile.write(str(explored_area_log[e, i]) + "\n")
        #         logfile.flush()

        # logfile.close()

        # logfile = open("{}/explored_ratio.txt".format(dump_dir), "w+")
        # for e in range(num_scenes):
        #     for i in range(explored_ratio_log[e].shape[0]):
        #         logfile.write(str(explored_ratio_log[e, i]) + "\n")
        #         logfile.flush()

        # logfile.close()

        # log = "Final Exp Area: \n"
        # for i in range(explored_area_log.shape[2]):
        #     log += "{:.5f}, ".format(
        #         np.mean(explored_area_log[:, :, i]))

        # log += "\nFinal Exp Ratio: \n"
        # for i in range(explored_ratio_log.shape[2]):
        #     log += "{:.5f}, ".format(
        #         np.mean(explored_ratio_log[:, :, i]))

        print(log)
        logging.info(log)


if __name__ == "__main__":
    main()

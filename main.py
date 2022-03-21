from __future__ import print_function

import argparse
import os

import torch
import torch.multiprocessing as mp
import time
import shared_optim
from env import create_habitat_env
from model import ActorCritic
from test import test
from train import train
from arguments import get_args
import torch.optim as optim
import torch.nn as  nn 
import matplotlib.pyplot as plt
import numpy as np
import torch.nn.functional as F

if __name__ == '__main__':
    mp.set_start_method('spawn')
    os.environ['OMP_NUM_THREADS'] = '16'
    os.environ['CUDA_VISIBLE_DEVICES'] = "0"
    args = get_args()
    
    device = 'cpu'
    # if torch.cuda.is_available():
    #     print("Using GPU....")
    #     device = torch.device("cuda:0")
    torch.manual_seed(args.seed)
    env = create_habitat_env(args, 0)
    shared_model = ActorCritic(
        env.observation_space.shape[0], env.action_space, device)
    shared_model.share_memory()
    shared_model.to(device)
    if args.no_shared:
        optimizer = None
    else:
        #optimizer = shared_optim.SharedAdam(shared_model.parameters(), lr=args.lr)
        optimizer = shared_optim.SharedRMSprop(shared_model.parameters(), lr=args.lr)
        optimizer.share_memory()
    processes = []
    
    

    counter = mp.Value('i', 0)
    lock = mp.Lock()
    p = mp.Process(target=test, args=(args.num_processes, args, shared_model, counter, device))
    p.start()
    processes.append(p)
    time.sleep(0.1)
    print(args.num_processes)
    for rank in range(0, args.num_processes):
        p = mp.Process(target=train, args=(rank, args, shared_model, counter, lock, device, optimizer ))
        p.start()
        processes.append(p)
        time.sleep(0.1)
    for p in processes:
        p.join()
        time.sleep(0.1)

# if __name__ == "__main__":
#     args = get_args()
#     rank = 0
#     env = create_habitat_env(args, rank)

#     obs, info  = env.reset()

#     obs, reward, done, info  = env.step(1)
    # Step through environment with random actions

    # if args.task=="generate_train":
    #     for i in range(10000):
    #         os.system('clear')
    #         print("time step - {}".format(i))
    #         if i<50:
    #             obs, rew, done, info= env.step(2)
    #         else:
    #             obs, rew, done, info= env.step(env.action_space.sample())
    #         if i==0:
    #             img= plt.imread(args.dump_location +"data/exp1/episodes/1/1/0-1-Vis-depth-1.png")
    #             plt.imsave(args.dump_location +"data/exp1/episodes/1/1/0-1-Vis-depth-1.png",img[int((img.shape[0]-4)/2):int((img.shape[0]-2+4)/2)+1,:])
    #             img= plt.imread(args.dump_location +"data/exp1/episodes/1/1/0-1-Vis-rgb-1.png")
    #             plt.imsave(args.dump_location +"data/exp1/episodes/1/1/0-1-Vis-rgb-1.png",img[int((img.shape[0]-84)/2):int((img.shape[0]-2+84)/2)+1,:])

    #     print("generated training batch")
    # if args.task=="milestone1":
    #     for i in range(200):
    #         os.system('clear')
    #         print("time step - {}".format(i))
    #         if i<50:
    #             obs, rew, done, info= env.step(2)    
    #         obs, rew, done, info= env.step(env.action_space.sample())
    #         if i==100:
    #             obs,info = env.reset()
    #     print("generated image for milestone 1")
   





    
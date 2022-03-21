import time
from collections import deque
import os
import torch
import torch.nn.functional as F

from env import create_habitat_env
from model import ActorCritic
from torch.utils.tensorboard import SummaryWriter


def test(rank, args, shared_model, counter, device):
    torch.manual_seed(args.seed + rank)
    env = create_habitat_env(args, rank)

    model = ActorCritic(env.observation_space.shape[0], env.action_space, device).to(device)

    model.eval()

    state, info = env.reset()
    state = torch.from_numpy(state).float().to(device)
    reward_sum = 0
    done = True

    start_time = time.time()

    # a quick hack to prevent the agent from stucking
    actions = deque(maxlen=100)
    episode_length = 0
    episode_count = 0
    trainDataFolder  = os.path.join('logs/', args.exp_name)
    writer = SummaryWriter(log_dir=trainDataFolder)
    while True:
        episode_length += 1
        # Sync with the shared model
        if done:
            model.load_state_dict(shared_model.state_dict())
            cx = torch.zeros(1, 256).float().to(device)
            hx = torch.zeros(1, 256).float().to(device)
        else:
            cx = cx.detach().float().to(device)
            hx = hx.detach().float().to(device)

        with torch.no_grad():
            value, logit, (hx, cx) = model((state.unsqueeze(0), (hx, cx)))
        prob = F.softmax(logit, dim=-1)
        action = prob.max(1, keepdim=True)[1].cpu().numpy()
        state, reward, done, _ = env.step(action[0, 0])
        done = done or episode_length >= args.max_episode_length
        reward_sum += reward
        # a quick hack to prevent the agent from stucking
        actions.append(action[0, 0])
        if actions.count(actions[0]) == actions.maxlen:
            done = True
        if (episode_length % 20==0):
            print("Time {}, num steps {}, FPS {:.0f}, episode reward {}, episode length {}".format(
                time.strftime("%Hh %Mm %Ss",
                              time.gmtime(time.time() - start_time)),
                counter.value, counter.value / (time.time() - start_time),
                reward_sum, episode_length))
            print(prob, action)    
        if done:
            reward_sum = 0
            episode_length = 0
            actions.clear()
            state, info  = env.reset()
            done = False
            writer.add_scalar('Reward', reward_sum, episode_count)
            writer.add_scalar('Time', time.time() - start_time , episode_count)
            episode_count +=1
            time.sleep(60)

        state = torch.from_numpy(state).float().to(device)
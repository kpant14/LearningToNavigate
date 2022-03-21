import torch
import torch.nn.functional as F
import torch.optim as optim

from env import create_habitat_env
from model import ActorCritic
import numpy as np

def ensure_shared_grads(model, shared_model):
    for param, shared_param in zip(model.parameters(),
                                   shared_model.parameters()):
        if shared_param.grad is not None:
            return
        shared_param._grad = param.grad


def train(rank, args, shared_model, counter, lock, device, optimizer=None):
    torch.manual_seed(args.seed + rank)

    #env = create_atari_env(args.env_name)
    env = create_habitat_env(args, rank)
    #env.seed(args.seed + rank)

    model = ActorCritic(env.observation_space.shape[0], env.action_space, device).to(device)

    if optimizer is None:
        optimizer = optim.Adam(shared_model.parameters(), lr=args.lr)

    model.train()

    state, info = env.reset()
    state = torch.from_numpy(state).float().to(device)
    done = True

    episode_length = 0
    while True:
        # Sync with the shared model
        model.load_state_dict(shared_model.state_dict())
        if done:
            cx = torch.zeros(1, 256).float().to(device)
            hx = torch.zeros(1, 256).float().to(device)
        else:
            cx = cx.detach().float().to(device)
            hx = hx.detach().float().to(device)

        values = []
        log_probs = []
        rewards = []
        entropies = []

        for step in range(args.num_steps):
            episode_length += 1
            value, logit, (hx, cx) = model((state.unsqueeze(0),
                                            (hx, cx)))
            prob = F.softmax(logit, dim=-1)
            log_prob = F.log_softmax(logit, dim=-1)
            entropy = -(log_prob * prob).sum(1, keepdim=True)
            entropies.append(entropy)

            action = prob.multinomial(num_samples=1).detach()
            log_prob = log_prob.gather(1, action)
            action = action.cpu().numpy()
            state, reward, done, _ = env.step(action[0 , 0])
            done = done or episode_length >= args.max_episode_length
            #reward = max(min(reward, 1), -1)
            
            with lock:
                counter.value += 1

            if done:
                episode_length = 0
                state, info = env.reset()

            state = torch.from_numpy(state).float().to(device)
            values.append(value)
            log_probs.append(log_prob)
            rewards.append(reward)

            if done:
                break

        R = torch.zeros(1, 1).to(device)
        if not done:
            value, _, _ = model((state.unsqueeze(0), (hx, cx)))
            R = value.data   

        values.append(R)
        policy_loss = 0
        value_loss = 0
        gae = torch.zeros(1, 1).to(device)
        for i in reversed(range(len(rewards))):
            R = args.gamma * R + rewards[i]
            advantage = R - values[i]
            value_loss = value_loss + 0.5 * advantage.pow(2)

            # Generalized Advantage Estimation
            delta_t = rewards[i] + args.gamma * \
                values[i + 1] - values[i]
            gae = gae * args.gamma * args.gae_lambda + delta_t

            policy_loss = policy_loss - \
                log_probs[i] * gae.detach() - args.entropy_coef * entropies[i]

        optimizer.zero_grad()

        (policy_loss + args.value_loss_coef * value_loss).backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), args.max_grad_norm)
        print('rank:{} Loss: {}'.format( rank, (policy_loss + args.value_loss_coef * value_loss).detach().cpu().numpy()))
        ensure_shared_grads(model, shared_model)
        optimizer.step()
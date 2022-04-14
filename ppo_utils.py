#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
from collections import defaultdict

import numpy as np
import torch
import torch.nn as nn


class Flatten(nn.Module):
    def forward(self, x):
        return x.view(x.size(0), -1)


class CustomFixedCategorical(torch.distributions.Categorical):
    def sample(self, sample_shape=torch.Size()):
        return super().sample().unsqueeze(-1)
        #return super().sample(sample_shape).unsqueeze(-1)

    def log_probs(self, actions):
        return (
            super()
            .log_prob(actions.squeeze(-1))
            .view(actions.size(0), -1)
            .sum(-1)
            .unsqueeze(-1)
        )

    def mode(self):
        return self.probs.argmax(dim=-1, keepdim=True)


class CategoricalNet(nn.Module):
    def __init__(self, num_inputs, num_outputs):
        super().__init__()

        self.linear = nn.Linear(num_inputs, num_outputs)

        nn.init.orthogonal_(self.linear.weight, gain=0.01)
        nn.init.constant_(self.linear.bias, 0)

    def forward(self, x):
        x = self.linear(x)
        return CustomFixedCategorical(logits=x)


def _flatten_helper(t, n, tensor):
    return tensor.view(t * n, *tensor.size()[2:])


def update_linear_schedule(optimizer, epoch, total_num_epochs, initial_lr):
    r"""Decreases the learning rate linearly
    """
    lr = initial_lr - (initial_lr * (epoch / float(total_num_epochs)))
    for param_group in optimizer.param_groups:
        param_group["lr"] = lr


class RolloutStorage:
    def __init__(
        self,
        num_steps,
        num_envs,
        observation_space,
        action_space,
        recurrent_hidden_state_size,
    ):
        self.observations = {}

        for sensor in observation_space.spaces:
            self.observations[sensor] = torch.zeros(
                num_steps + 1,
                num_envs,
                *observation_space.spaces[sensor].shape
            )

        self.recurrent_hidden_states = torch.zeros(
            num_steps + 1, num_envs, recurrent_hidden_state_size
        )

        self.rewards = torch.zeros(num_steps, num_envs, 1)
        self.value_preds = torch.zeros(num_steps + 1, num_envs, 1)
        self.returns = torch.zeros(num_steps + 1, num_envs, 1)

        self.action_log_probs = torch.zeros(num_steps, num_envs, 1)
        if action_space.__class__.__name__ == "Discrete":
            action_shape = 1
        else:
            action_shape = 1 

        self.actions = torch.zeros(num_steps, num_envs, action_shape)
        if action_space.__class__.__name__ == "Discrete":
            self.actions = self.actions.long()

        self.masks = torch.ones(num_steps + 1, num_envs, 1)

        self.num_steps = num_steps
        self.step = 0

    def to(self, device):
        for sensor in self.observations:
            self.observations[sensor] = self.observations[sensor].to(device)

        self.recurrent_hidden_states = self.recurrent_hidden_states.to(device)
        self.rewards = self.rewards.to(device)
        self.value_preds = self.value_preds.to(device)
        self.returns = self.returns.to(device)
        self.action_log_probs = self.action_log_probs.to(device)
        self.actions = self.actions.to(device)
        self.masks = self.masks.to(device)

    def insert(
        self,
        observations,
        recurrent_hidden_states,
        actions,
        action_log_probs,
        value_preds,
        rewards,
        masks,
    ):
        for sensor in observations:
            self.observations[sensor][self.step + 1].copy_(
                observations[sensor]
            )
        self.recurrent_hidden_states[self.step + 1].copy_(
            recurrent_hidden_states
        )
        self.actions[self.step].copy_(actions)
        self.action_log_probs[self.step].copy_(action_log_probs)
        self.value_preds[self.step].copy_(value_preds)
        self.rewards[self.step].copy_(rewards)
        self.masks[self.step + 1].copy_(masks)

        self.step = (self.step + 1) % self.num_steps

    def after_update(self):
        for sensor in self.observations:
            self.observations[sensor][0].copy_(self.observations[sensor][-1])

        self.recurrent_hidden_states[0].copy_(self.recurrent_hidden_states[-1])
        self.masks[0].copy_(self.masks[-1])

    def compute_returns(self, next_value, use_gae, gamma, tau):
        if use_gae:
            self.value_preds[-1] = next_value
            gae = 0
            for step in reversed(range(self.rewards.size(0))):
                delta = (
                    self.rewards[step]
                    + gamma * self.value_preds[step + 1] * self.masks[step + 1]
                    - self.value_preds[step]
                )
                gae = delta + gamma * tau * self.masks[step + 1] * gae
                self.returns[step] = gae + self.value_preds[step]
        else:
            self.returns[-1] = next_value
            for step in reversed(range(self.rewards.size(0))):
                self.returns[step] = (
                    self.returns[step + 1] * gamma * self.masks[step + 1]
                    + self.rewards[step]
                )

    def recurrent_generator(self, advantages, num_mini_batch):
        num_processes = self.rewards.size(1)
        assert num_processes >= num_mini_batch, (
            "PPO requires the number of processes ({}) "
            "to be greater than or equal to the number of "
            "PPO mini batches ({}).".format(num_processes, num_mini_batch)
        )
        num_envs_per_batch = num_processes // num_mini_batch
        perm = torch.randperm(num_processes)
        for start_ind in range(0, num_processes, num_envs_per_batch):
            observations_batch = defaultdict(list)

            recurrent_hidden_states_batch = []
            actions_batch = []
            value_preds_batch = []
            return_batch = []
            masks_batch = []
            old_action_log_probs_batch = []
            adv_targ = []

            for offset in range(num_envs_per_batch):
                ind = perm[start_ind + offset]

                for sensor in self.observations:
                    observations_batch[sensor].append(
                        self.observations[sensor][:-1, ind]
                    )

                recurrent_hidden_states_batch.append(
                    self.recurrent_hidden_states[0:1, ind]
                )

                actions_batch.append(self.actions[:, ind])
                value_preds_batch.append(self.value_preds[:-1, ind])
                return_batch.append(self.returns[:-1, ind])
                masks_batch.append(self.masks[:-1, ind])
                old_action_log_probs_batch.append(
                    self.action_log_probs[:, ind]
                )

                adv_targ.append(advantages[:, ind])

            T, N = self.num_steps, num_envs_per_batch

            # These are all tensors of size (T, N, -1)
            for sensor in observations_batch:
                observations_batch[sensor] = torch.stack(
                    observations_batch[sensor], 1
                )

            actions_batch = torch.stack(actions_batch, 1)
            value_preds_batch = torch.stack(value_preds_batch, 1)
            return_batch = torch.stack(return_batch, 1)
            masks_batch = torch.stack(masks_batch, 1)
            old_action_log_probs_batch = torch.stack(
                old_action_log_probs_batch, 1
            )
            adv_targ = torch.stack(adv_targ, 1)

            # States is just a (N, -1) tensor
            recurrent_hidden_states_batch = torch.stack(
                recurrent_hidden_states_batch, 1
            ).view(N, -1)

            # Flatten the (T, N, ...) tensors to (T * N, ...)
            for sensor in observations_batch:
                observations_batch[sensor] = _flatten_helper(
                    T, N, observations_batch[sensor]
                )

            actions_batch = _flatten_helper(T, N, actions_batch)
            value_preds_batch = _flatten_helper(T, N, value_preds_batch)
            return_batch = _flatten_helper(T, N, return_batch)
            masks_batch = _flatten_helper(T, N, masks_batch)
            old_action_log_probs_batch = _flatten_helper(
                T, N, old_action_log_probs_batch
            )
            adv_targ = _flatten_helper(T, N, adv_targ)

            yield (
                observations_batch,
                recurrent_hidden_states_batch,
                actions_batch,
                value_preds_batch,
                return_batch,
                masks_batch,
                old_action_log_probs_batch,
                adv_targ,
            )


def batch_obs(observations):
    batch = defaultdict(list)

    for obs in observations:
        for sensor in obs:
            batch[sensor].append(obs[sensor])

    for sensor in batch:
        batch[sensor] = torch.tensor(
            np.array(batch[sensor]), dtype=torch.float
        )
    return batch


def ppo_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--clip-param",
        type=float,
        default=0.2,
        help="ppo clip parameter (default: 0.2)",
    )
    parser.add_argument(
        "--ppo-epoch",
        type=int,
        default=4,
        help="number of ppo epochs (default: 4)",
    )
    parser.add_argument(
        "--num-mini-batch",
        type=int,
        default=5,
        help="number of batches for ppo (default: 32)",
    )
    parser.add_argument(
        "--value-loss-coef",
        type=float,
        default=0.5,
        help="value loss coefficient (default: 0.5)",
    )
    parser.add_argument(
        "--entropy-coef",
        type=float,
        default=0.01,
        help="entropy term coefficient (default: 0.01)",
    )
    parser.add_argument(
        "--lr", type=float, default=7e-4, help="learning rate (default: 7e-4)"
    )
    parser.add_argument(
        "--eps",
        type=float,
        default=1e-5,
        help="RMSprop optimizer epsilon (default: 1e-5)",
    )
    parser.add_argument(
        "--max-grad-norm",
        type=float,
        default=0.5,
        help="max norm of gradients (default: 0.5)",
    )
    parser.add_argument(
        "--num-steps",
        type=int,
        default=5,
        help="number of forward steps in A2C (default: 5)",
    )
    parser.add_argument("--hidden-size", type=int, default=512)
    parser.add_argument(
        "--num-processes",
        type=int,
        default=5,
        help="number of training processes " "to use (default: 16)",
    )
    parser.add_argument(
        "--use-gae",
        action="store_true",
        default=False,
        help="use generalized advantage estimation",
    )
    parser.add_argument(
        "--use-linear-lr-decay",
        action="store_true",
        default=False,
        help="use a linear schedule on the learning rate",
    )
    parser.add_argument(
        "--use-linear-clip-decay",
        action="store_true",
        default=False,
        help="use a linear schedule on the " "ppo clipping parameter",
    )
    parser.add_argument(
        "--gamma",
        type=float,
        default=0.99,
        help="discount factor for rewards (default: 0.99)",
    )
    parser.add_argument(
        "--tau", type=float, default=0.95, help="gae parameter (default: 0.95)"
    )
    parser.add_argument(
        "--log-file", type=str, required=True, help="path for log file"
    )
    parser.add_argument(
        "--reward-window-size",
        type=int,
        default=50,
        help="logging window for rewards",
    )
    parser.add_argument(
        "--log-interval",
        type=int,
        default=10,
        help="number of updates after which metrics are logged",
    )
    parser.add_argument(
        "--checkpoint-interval",
        type=int,
        default=500,
        help="number of updates after which models are checkpointed",
    )
    parser.add_argument(
        "--checkpoint-folder",
        type=str,
        required=True,
        help="folder for storing checkpoints",
    )
    parser.add_argument(
        "--sim-gpu-id",
        type=int,
        required=True,
        help="gpu id on which scenes are loaded",
    )
    parser.add_argument(
        "--pth-gpu-id",
        type=int,
        required=True,
        help="gpu id on which pytorch runs",
    )
    parser.add_argument(
        "--num-updates",
        type=int,
        default=10000,
        help="number of PPO updates to run",
    )
    parser.add_argument(
        "--sensors",
        type=str,
        default="RGB_SENSOR,DEPTH_SENSOR",
        help="comma separated string containing different sensors to use,"
        "currently 'RGB_SENSOR' and 'DEPTH_SENSOR' are supported",
    )
    parser.add_argument(
        "--task-config",
        type=str,
        default="habitat-lab/configs/tasks/pointnav.yaml",
        help="path to config yaml containing information about task",
    )
    parser.add_argument("--seed", type=int, default=100)
    parser.add_argument(
        "opts",
        default=None,
        nargs=argparse.REMAINDER,
        help="Modify config options from command line",
    )
    return parser

#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import torch
import torch.nn as nn

#from habitat_baselines.rl.ppo.ppo_utils import CategoricalNet, Flatten


class Policy(nn.Module):
    def __init__(
        self,
        observation_space,
        action_space,
        goal_sensor_uuid,
        hidden_size=512,
    ):
        super().__init__()
        self.dim_actions = action_space.n
        self.goal_sensor_uuid = goal_sensor_uuid
        self.net = Net(
            observation_space=observation_space,
            hidden_size=hidden_size,
            goal_sensor_uuid=goal_sensor_uuid,
        )
        self.action_distribution = CategoricalNet(
            self.net.output_size, self.dim_actions
        )

    def forward(self, *x):
        raise NotImplementedError

    def act(self, observations, rnn_hidden_states, masks, deterministic=False):
        value, actor_features, rnn_hidden_states = self.net(
            observations, rnn_hidden_states, masks
        )
        distribution = self.action_distribution(actor_features)

        if deterministic:
            action = distribution.mode()
        else:
            action = distribution.sample()

        action_log_probs = distribution.log_probs(action)

        return value, action, action_log_probs, rnn_hidden_states

    def get_value(self, observations, rnn_hidden_states, masks):
        value, _, _ = self.net(observations, rnn_hidden_states, masks)
        return value

    def evaluate_actions(self, observations, rnn_hidden_states, masks, action):
        value, actor_features, rnn_hidden_states = self.net(
            observations, rnn_hidden_states, masks
        )
        distribution = self.action_distribution(actor_features)

        action_log_probs = distribution.log_probs(action)
        distribution_entropy = distribution.entropy().mean()

        return value, action_log_probs, distribution_entropy, rnn_hidden_states


class Net(nn.Module):
    r"""Network which passes the input image through CNN and concatenates
    goal vector with CNN's output and passes that through RNN.
    """

    def __init__(self, observation_space, hidden_size, goal_sensor_uuid):
        super().__init__()
        self.goal_sensor_uuid = goal_sensor_uuid
        self._n_input_goal = observation_space.spaces[
            self.goal_sensor_uuid
        ].shape[0]
        self._hidden_size = hidden_size

        self.cnn = self._init_perception_model(observation_space)

        if self.is_blind:
            self.rnn = nn.GRU(self._n_input_goal, self._hidden_size)
        else:
            self.rnn = nn.GRU(
                self.output_size + self._n_input_goal, self._hidden_size
            )

        self.critic_linear = nn.Linear(self._hidden_size, 1)

        self.layer_init()
        self.train()

    def _init_perception_model(self, observation_space):
        if "rgb" in observation_space.spaces:
            self._n_input_rgb = observation_space.spaces["rgb"].shape[2]
        else:
            self._n_input_rgb = 0

        if "depth" in observation_space.spaces:
            self._n_input_depth = observation_space.spaces["depth"].shape[2]
        else:
            self._n_input_depth = 0

        # kernel size for different CNN layers
        self._cnn_layers_kernel_size = [(8, 8), (4, 4), (3, 3)]

        # strides for different CNN layers
        self._cnn_layers_stride = [(4, 4), (2, 2), (1, 1)]

        if self._n_input_rgb > 0:
            cnn_dims = np.array(
                observation_space.spaces["rgb"].shape[:2], dtype=np.float32
            )
        elif self._n_input_depth > 0:
            cnn_dims = np.array(
                observation_space.spaces["depth"].shape[:2], dtype=np.float32
            )

        if self.is_blind:
            return nn.Sequential()
        else:
            for kernel_size, stride in zip(
                self._cnn_layers_kernel_size, self._cnn_layers_stride
            ):
                cnn_dims = self._conv_output_dim(
                    dimension=cnn_dims,
                    padding=np.array([0, 0], dtype=np.float32),
                    dilation=np.array([1, 1], dtype=np.float32),
                    kernel_size=np.array(kernel_size, dtype=np.float32),
                    stride=np.array(stride, dtype=np.float32),
                )

            return nn.Sequential(
                nn.Conv2d(
                    in_channels=self._n_input_rgb + self._n_input_depth,
                    out_channels=32,
                    kernel_size=self._cnn_layers_kernel_size[0],
                    stride=self._cnn_layers_stride[0],
                ),
                nn.ReLU(),
                nn.Conv2d(
                    in_channels=32,
                    out_channels=64,
                    kernel_size=self._cnn_layers_kernel_size[1],
                    stride=self._cnn_layers_stride[1],
                ),
                nn.ReLU(),
                nn.Conv2d(
                    in_channels=64,
                    out_channels=32,
                    kernel_size=self._cnn_layers_kernel_size[2],
                    stride=self._cnn_layers_stride[2],
                ),
                nn.Flatten(),
                nn.Linear(32 * cnn_dims[0] * cnn_dims[1], self._hidden_size),
                nn.ReLU(),
            )

    def _conv_output_dim(
        self, dimension, padding, dilation, kernel_size, stride
    ):
        r"""Calculates the output height and width based on the input
        height and width to the convolution layer.
        ref: https://pytorch.org/docs/master/nn.html#torch.nn.Conv2d
        """
        assert len(dimension) == 2
        out_dimension = []
        for i in range(len(dimension)):
            out_dimension.append(
                int(
                    np.floor(
                        (
                            (
                                dimension[i]
                                + 2 * padding[i]
                                - dilation[i] * (kernel_size[i] - 1)
                                - 1
                            )
                            / stride[i]
                        )
                        + 1
                    )
                )
            )
        return tuple(out_dimension)

    @property
    def output_size(self):
        return self._hidden_size

    def layer_init(self):
        for layer in self.cnn:
            if isinstance(layer, (nn.Conv2d, nn.Linear)):
                nn.init.orthogonal_(
                    layer.weight, nn.init.calculate_gain("relu")
                )
                nn.init.constant_(layer.bias, val=0)

        for name, param in self.rnn.named_parameters():
            if "weight" in name:
                nn.init.orthogonal_(param)
            elif "bias" in name:
                nn.init.constant_(param, 0)

        nn.init.orthogonal_(self.critic_linear.weight, gain=1)
        nn.init.constant_(self.critic_linear.bias, val=0)

    def forward_rnn(self, x, hidden_states, masks):
        if x.size(0) == hidden_states.size(0):
            x, hidden_states = self.rnn(
                x.unsqueeze(0), (hidden_states * masks).unsqueeze(0)
            )
            x = x.squeeze(0)
            hidden_states = hidden_states.squeeze(0)
        else:
            # x is a (T, N, -1) tensor flattened to (T * N, -1)
            n = hidden_states.size(0)
            t = int(x.size(0) / n)

            # unflatten
            x = x.view(t, n, x.size(1))
            masks = masks.view(t, n)

            # steps in sequence which have zero for any agent. Assume t=0 has
            # a zero in it.
            has_zeros = (
                (masks[1:] == 0.0).any(dim=-1).nonzero().squeeze().cpu()
            )

            # +1 to correct the masks[1:]
            if has_zeros.dim() == 0:
                has_zeros = [has_zeros.item() + 1]  # handle scalar
            else:
                has_zeros = (has_zeros + 1).numpy().tolist()

            # add t=0 and t=T to the list
            has_zeros = [0] + has_zeros + [t]

            hidden_states = hidden_states.unsqueeze(0)
            outputs = []
            for i in range(len(has_zeros) - 1):
                # process steps that don't have any zeros in masks together
                start_idx = has_zeros[i]
                end_idx = has_zeros[i + 1]

                rnn_scores, hidden_states = self.rnn(
                    x[start_idx:end_idx],
                    hidden_states * masks[start_idx].view(1, -1, 1),
                )

                outputs.append(rnn_scores)

            # x is a (T, N, -1) tensor
            x = torch.cat(outputs, dim=0)
            x = x.view(t * n, -1)  # flatten
            hidden_states = hidden_states.squeeze(0)

        return x, hidden_states

    @property
    def is_blind(self):
        return self._n_input_rgb + self._n_input_depth == 0

    def forward_perception_model(self, observations):
        cnn_input = []
        if self._n_input_rgb > 0:
            rgb_observations = observations["rgb"]
            # permute tensor to dimension [BATCH x CHANNEL x HEIGHT X WIDTH]
            rgb_observations = rgb_observations.permute(0, 3, 1, 2)
            rgb_observations = rgb_observations / 255.0  # normalize RGB
            cnn_input.append(rgb_observations)

        if self._n_input_depth > 0:
            depth_observations = observations["depth"]
            # permute tensor to dimension [BATCH x CHANNEL x HEIGHT X WIDTH]
            depth_observations = depth_observations.permute(0, 3, 1, 2)
            cnn_input.append(depth_observations)

        cnn_input = torch.cat(cnn_input, dim=1)

        return self.cnn(cnn_input)

    def forward(self, observations, rnn_hidden_states, masks):
        x = observations[self.goal_sensor_uuid]

        if not self.is_blind:
            perception_embed = self.forward_perception_model(observations)
            x = torch.cat([perception_embed, x], dim=1)

        x, rnn_hidden_states = self.forward_rnn(x, rnn_hidden_states, masks)

        return self.critic_linear(x), x, rnn_hidden_states

#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import torch
import torch.nn as nn
import torch.optim as optim

EPS_PPO = 1e-5


class PPO(nn.Module):
    def __init__(
        self,
        actor_critic,
        clip_param,
        ppo_epoch,
        num_mini_batch,
        value_loss_coef,
        entropy_coef,
        lr=None,
        eps=None,
        max_grad_norm=None,
        use_clipped_value_loss=True,
    ):

        super().__init__()

        self.actor_critic = actor_critic

        self.clip_param = clip_param
        self.ppo_epoch = ppo_epoch
        self.num_mini_batch = num_mini_batch

        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef

        self.max_grad_norm = max_grad_norm
        self.use_clipped_value_loss = use_clipped_value_loss

        self.optimizer = optim.Adam(actor_critic.parameters(), lr=lr, eps=eps)

    def forward(self, *x):
        raise NotImplementedError

    def update(self, rollouts):
        advantages = rollouts.returns[:-1] - rollouts.value_preds[:-1]
        advantages = (advantages - advantages.mean()) / (
            advantages.std() + EPS_PPO
        )

        value_loss_epoch = 0
        action_loss_epoch = 0
        dist_entropy_epoch = 0

        for e in range(self.ppo_epoch):
            data_generator = rollouts.recurrent_generator(
                advantages, self.num_mini_batch
            )

            for sample in data_generator:
                (
                    obs_batch,
                    recurrent_hidden_states_batch,
                    actions_batch,
                    value_preds_batch,
                    return_batch,
                    masks_batch,
                    old_action_log_probs_batch,
                    adv_targ,
                ) = sample

                # Reshape to do in a single forward pass for all steps
                (
                    values,
                    action_log_probs,
                    dist_entropy,
                    _,
                ) = self.actor_critic.evaluate_actions(
                    obs_batch,
                    recurrent_hidden_states_batch,
                    masks_batch,
                    actions_batch,
                )

                ratio = torch.exp(
                    action_log_probs - old_action_log_probs_batch
                )
                surr1 = ratio * adv_targ
                surr2 = (
                    torch.clamp(
                        ratio, 1.0 - self.clip_param, 1.0 + self.clip_param
                    )
                    * adv_targ
                )
                action_loss = -torch.min(surr1, surr2).mean()

                if self.use_clipped_value_loss:
                    value_pred_clipped = value_preds_batch + (
                        values - value_preds_batch
                    ).clamp(-self.clip_param, self.clip_param)
                    value_losses = (values - return_batch).pow(2)
                    value_losses_clipped = (
                        value_pred_clipped - return_batch
                    ).pow(2)
                    value_loss = (
                        0.5
                        * torch.max(value_losses, value_losses_clipped).mean()
                    )
                else:
                    value_loss = 0.5 * (return_batch - values).pow(2).mean()

                self.optimizer.zero_grad()
                (
                    value_loss * self.value_loss_coef
                    + action_loss
                    - dist_entropy * self.entropy_coef
                ).backward()
                nn.utils.clip_grad_norm_(
                    self.actor_critic.parameters(), self.max_grad_norm
                )
                self.optimizer.step()

                value_loss_epoch += value_loss.item()
                action_loss_epoch += action_loss.item()
                dist_entropy_epoch += dist_entropy.item()

        num_updates = self.ppo_epoch * self.num_mini_batch

        value_loss_epoch /= num_updates
        action_loss_epoch /= num_updates
        dist_entropy_epoch /= num_updates

        return value_loss_epoch, action_loss_epoch, dist_entropy_epoch
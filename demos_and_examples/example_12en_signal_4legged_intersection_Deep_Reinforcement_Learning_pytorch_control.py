"""
FOR JUPYTER NOTEBOOK
Execute this code by
%run example_12en_signal_4legged_intersection_Deep_Reinforcement_Learning_pytorch_control.py
on Jupyter Notebook for visualization.

DRL part of this code is adapted from https://pytorch.org/tutorials/intermediate/reinforcement_q_learning.html
"""
import os
os.environ['KMP_DUPLICATE_LIB_OK']='True'

import gymnasium as gym
import math
import random
import matplotlib
import matplotlib.pyplot as plt
from collections import namedtuple, deque
from itertools import count
from scipy.optimize import minimize

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

from uxsim import *
import random

################################################
# Environment of gymnasium

class TrafficSim(gym.Env):
    def __init__(self):
        """
        traffic scenario: 4-legged intersection with 2 phase signal.
        action 1: greenlight for direction 1
        action 2: greenlight for direction 2
        state: number of waiting vehicles for each incoming link
        reward: negative of difference of total waiting vehicles
        """

        #action
        self.n_action = 2
        self.action_space = gym.spaces.Discrete(self.n_action)

        #state
        self.n_state = 4
        low = np.array([0 for i in range(self.n_state)])
        high = np.array([100 for i in range(self.n_state)])
        self.observation_space = gym.spaces.Box(low=low, high=high)

        self.reset()

    def reset(self):
        """
        reset the env
        """
        seed = None #demand is always random
        self.W = World(
            name="",
            deltan=5,
            tmax=3600,
            print_mode=0, save_mode=0, show_mode=1,
            random_seed=seed,
            duo_update_time=99999
        )
        random.seed(seed)

        #network definition
        self.II = self.W.addNode("Intersection", 0, 0, signal=[60,60])
        self.EE = self.W.addNode("E", 1, 0)
        self.WW = self.W.addNode("W", -1, 0)
        self.SS = self.W.addNode("S", 0, 1)
        self.NN = self.W.addNode("N", 0, -1)
        self.W.addLink("EI", self.EE, self.II, length=500, free_flow_speed=10, jam_density=0.2, signal_group=0)
        self.W.addLink("WI", self.WW, self.II, length=500, free_flow_speed=10, jam_density=0.2, signal_group=0)
        self.W.addLink("SI", self.SS, self.II, length=500, free_flow_speed=10, jam_density=0.2, signal_group=1)
        self.W.addLink("NI", self.NN, self.II, length=500, free_flow_speed=10, jam_density=0.2, signal_group=1)
        self.W.addLink("IE", self.II, self.EE, length=500, free_flow_speed=10, jam_density=0.2)
        self.W.addLink("IW", self.II, self.WW, length=500, free_flow_speed=10, jam_density=0.2)
        self.W.addLink("IS", self.II, self.SS, length=500, free_flow_speed=10, jam_density=0.2)
        self.W.addLink("IN", self.II, self.NN, length=500, free_flow_speed=10, jam_density=0.2)

        #random demand definition
        dt = 30
        for t in range(0, 3600, dt):
            self.W.adddemand(self.EE, self.WW, t, t+dt, random.uniform(0, 0.6))
            self.W.adddemand(self.WW, self.EE, t, t+dt, random.uniform(0, 0.6))
            self.W.adddemand(self.SS, self.NN, t, t+dt, random.uniform(0, 0.6))
            self.W.adddemand(self.NN, self.SS, t, t+dt, random.uniform(0, 0.6))

        #initial observation
        observation = np.array([0 for i in range(self.n_state)])

        #log
        self.log_state = []
        self.log_reward = []

        return observation, None

    def comp_state(self):
        """
        compute the current state
        """
        vehicles_per_links = {}
        for l in self.II.inlinks.values():
            vehicles_per_links[l] = l.num_vehicles_queue #l.num_vehicles_queue: the number of vehicles in queue in link l
        return list(vehicles_per_links.values())

    def comp_n_veh_queue(self):
        return sum(self.comp_state())

    def step(self, action_index):
        """
        proceed env by 1 step = 30 seconds
        """

        n_queue_veh_old = self.comp_n_veh_queue()

        #change signal by action
        if action_index == 0:
            self.II.signal_phase = 0
            self.II.signal_t = 0
        elif action_index == 1:
            self.II.signal_phase = 1
            self.II.signal_t = 0

        #traffic dynamics. execute simulation for 30 seconds
        if self.W.check_simulation_ongoing():
            self.W.exec_simulation(duration_t=30)

        #observe state
        observation = np.array(self.comp_state())

        #compute reward
        n_queue_veh = self.comp_n_veh_queue()
        reward = -(n_queue_veh-n_queue_veh_old)

        #check termination
        done = False
        if self.W.check_simulation_ongoing() == False:
            done = True

        #log
        self.log_state.append(observation)
        self.log_reward.append(reward)

        return observation, reward, done, {}, None

################################################
# DQN
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

env = TrafficSim()

Transition = namedtuple('Transition',
                        ('state', 'action', 'next_state', 'reward'))

class ReplayMemory:
    def __init__(self, capacity):
        self.memory = deque([], maxlen=capacity)

    def push(self, *args):
        """Save a transition"""
        self.memory.append(Transition(*args))

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)

class DQN(nn.Module):
    def __init__(self, n_observations, n_actions):
        super().__init__()
        n_neurals = 64
        n_layers = 2
        self.layers = nn.ModuleList()
        self.layers.append(nn.Linear(n_observations, n_neurals))
        for i in range(n_layers):
            self.layers.append(nn.Linear(n_neurals, n_neurals))
        self.layer_last = nn.Linear(n_neurals, n_actions)

    # Called with either one element to determine next action, or a batch during optimization. Returns tensor([[left0exp,right0exp]...]).
    def forward(self, x):
        for layer in self.layers:
            x = F.relu(layer(x))
        return self.layer_last(x)

def select_action(state):
    global steps_done
    sample = random.random()
    eps_threshold = EPS_END + (EPS_START - EPS_END) * math.exp(-1. * steps_done / EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
        with torch.no_grad():
            # t.max(1) will return the largest column value of each row. second column on max result is index of where max element was found, so we pick action with the larger expected reward.
            return policy_net(state).max(1)[1].view(1, 1)
    else:
        return torch.tensor([[env.action_space.sample()]], device=device, dtype=torch.long)

def optimize_model():
    if len(memory) < BATCH_SIZE:
        return
    transitions = memory.sample(BATCH_SIZE)
    # Transpose the batch (see https://stackoverflow.com/a/19343/3343043 for detailed explanation). This converts batch-array of Transitions to Transition of batch-arrays.
    batch = Transition(*zip(*transitions))

    # Compute a mask of non-final states and concatenate the batch elements (a final state would've been the one after which simulation ended)
    non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                          batch.next_state)), device=device, dtype=torch.bool)
    non_final_next_states = torch.cat([s for s in batch.next_state
                                                if s is not None])
    state_batch = torch.cat(batch.state)
    action_batch = torch.cat(batch.action)
    reward_batch = torch.cat(batch.reward)

    # Compute Q(s_t, a) - the model computes Q(s_t), then we select the columns of actions taken. These are the actions which would've been taken for each batch state according to policy_net
    state_action_values = policy_net(state_batch).gather(1, action_batch)

    # Compute V(s_{t+1}) for all next states.
    # Expected values of actions for non_final_next_states are computed based on the "older" target_net; selecting their best reward with max(1)[0].
    # This is merged based on the mask, such that we'll have either the expected state value or 0 in case the state was final.
    next_state_values = torch.zeros(BATCH_SIZE, device=device)
    with torch.no_grad():
        next_state_values[non_final_mask] = target_net(non_final_next_states).max(1)[0]
    # Compute the expected Q values
    expected_state_action_values = (next_state_values * GAMMA) + reward_batch

    # Compute Huber loss
    criterion = nn.SmoothL1Loss()
    loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))

    # Optimize the model
    optimizer.zero_grad()
    loss.backward()
    # In-place gradient clipping
    torch.nn.utils.clip_grad_value_(policy_net.parameters(), 100)
    optimizer.step()

################################################
# (hyper)parameters

# the number of transitions sampled from the replay buffer
BATCH_SIZE = 128
# the discount factor as mentioned in the previous section
GAMMA = 0.99
# the starting value of epsilon
EPS_START = 0.9
# the final value of epsilon
EPS_END = 0.05
# the rate of exponential decay of epsilon, higher means a slower decay
EPS_DECAY = 1000
# the update rate of the target network
TAU = 0.005
# the learning rate of the ``AdamW`` optimizer
LR = 1e-4

# Get number of actions from gym action space
n_actions = env.action_space.n
# Get the number of state observations
state, info = env.reset()
n_observations = len(state)

policy_net = DQN(n_observations, n_actions).to(device)
target_net = DQN(n_observations, n_actions).to(device)
target_net.load_state_dict(policy_net.state_dict())

optimizer = optim.AdamW(policy_net.parameters(), lr=LR, amsgrad=True)
memory = ReplayMemory(10000)

steps_done = 0


################################################
# Execute DRL
num_episodes = 40

log_states = []
log_epi_average_delay = []
for i_episode in range(num_episodes):
    # Initialize the environment and get it's state
    state, info = env.reset()
    state = torch.tensor(state, dtype=torch.float32, device=device).unsqueeze(0)
    log_states.append([])
    for t in count():
        action = select_action(state)
        observation, reward, terminated, truncated, _ = env.step(action.item())
        reward = torch.tensor([reward], device=device)
        done = terminated or truncated

        if terminated:
            next_state = None
        else:
            next_state = torch.tensor(observation, dtype=torch.float32, device=device).unsqueeze(0)

        log_states[-1].append(state)

        # Store the transition in memory
        memory.push(state, action, next_state, reward)

        # Move to the next state
        state = next_state

        # Perform one step of the optimization (on the policy network)
        optimize_model()

        # Soft update of the target network's weights
        # θ′ ← τ θ + (1 −τ )θ′
        target_net_state_dict = target_net.state_dict()
        policy_net_state_dict = policy_net.state_dict()
        for key in policy_net_state_dict:
            target_net_state_dict[key] = policy_net_state_dict[key]*TAU + target_net_state_dict[key]*(1-TAU)
        target_net.load_state_dict(target_net_state_dict)

        if done:
            log_epi_average_delay.append(env.W.analyzer.average_delay)
            print(f"{i_episode}:[{log_epi_average_delay[-1] : .3f}]", end=" ")
            break

    if i_episode%10 == 0 or i_episode == num_episodes-1:
        env.W.analyzer.print_simple_stats(force_print=True)
        env.W.analyzer.time_space_diagram_traj(["EI", "WI", "SI", "NI"], figsize=(12,2))
        for t in list(range(0,env.W.TMAX,int(env.W.TMAX/6))):
            env.W.analyzer.network(t, detailed=1, network_font_size=0, figsize=(2,2))

        plt.figure(figsize=(4,3))
        plt.plot(log_epi_average_delay, "r.")
        plt.xlabel("episode")
        plt.ylabel("average delay (s)")
        plt.grid()
        plt.show()

---
sidebar_position: 1
title: "Reinforcement Learning Fundamentals"
description: "Core concepts of reinforcement learning including agents, environments, rewards, and policies"
---

# Reinforcement Learning Fundamentals

## Introduction

Reinforcement Learning (RL) is a type of machine learning where an agent learns to make decisions by interacting with an environment.

## Key Concepts

### Agent and Environment

- **Agent**: The learner and decision maker
- **Environment**: Everything the agent interacts with
- **State**: Current situation of the agent
- **Action**: Choices available to the agent
- **Reward**: Feedback signal from the environment

### The RL Loop

```
Agent → Action → Environment → State, Reward → Agent
```

## Markov Decision Process (MDP)

### Components of MDP

1. **State Space (S)**: Set of all possible states
2. **Action Space (A)**: Set of all possible actions
3. **Transition Function (P)**: Probability of moving to next state
4. **Reward Function (R)**: Immediate reward for state-action pairs
5. **Discount Factor (γ)**: Importance of future rewards

## Value Functions

### State Value Function V(s)

The expected return starting from state s.

### Action Value Function Q(s,a)

The expected return starting from state s, taking action a.

## Key Algorithms

### Value-Based Methods
- Q-Learning
- Deep Q-Networks (DQN)
- Double DQN

### Policy-Based Methods
- REINFORCE
- Policy Gradient

### Actor-Critic Methods
- A2C (Advantage Actor-Critic)
- A3C (Asynchronous Advantage Actor-Critic)
- PPO (Proximal Policy Optimization)
- SAC (Soft Actor-Critic)

## Summary

Understanding RL fundamentals is essential for applying learning-based approaches to robotics problems.

---

*Next: [Robot Learning](./robot-learning.md)*

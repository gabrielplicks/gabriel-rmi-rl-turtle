#!/usr/bin/env python

import rospy
import std_msgs

import random
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from q_value import Q_Value


class Agent:


    MAX_TRAINING_EPISODES = 500
    MAX_STEPS_PER_EPISODE = 50


    def __init__(self):
        # Stats attributes
        self.episode_rewards = dict()
        self.rewards_list = list()

        # Agent attributes
        self.state = None
        self.reward = None
        self.action = None
        self.p_state = None
        self.p_reward = None
        self.p_action = None

        # Q-learning attributes
        self.alpha = 0.9
        self.epsilon = 0.9
        self.gamma = 0.8

        self.r_plus = 5000
        self.exploration = 1

        self.q_values = dict()
        self.frequency = dict()



    def f(self, qv):
        if qv in self.q_values and self.frequency[qv] >= self.exploration:
            return self.q_values[qv]
        else:
            return self.r_plus

    

    def argmax_a(self, state):
        a = None
        max_value = float('-inf')
        
        if state == None:
            return a

        for action in self.env.get_available_actions(state):
            qv = Q_Value(state, action)
            f_value = self.f(qv)
            if f_value > max_value:
                max_value = f_value
                a = action

        return a
    


    def max_a(self, state):
        max_value = float('-inf')

        for action in self.env.get_available_actions(state):
            qv = Q_Value(state, action)
            if qv in self.q_values:
                q_sa = self.q_values[qv]
                if q_sa > max_value:
                    max_value = q_sa

        if max_value == float('-inf'): 
            max_value = 0.0
        
        return max_value



    def get_random_action(self, state):
        actions = list()
        for action in self.env.get_available_actions(state):
            qv = Q_Value(state, action)
            if qv not in self.q_values:
                actions.append(action)

        # VERIFICAR ISSO DE FICAR SEM ACOES!!!!
        if not actions:
            return 'FORWARD'
        else:
            return random.choice(actions)



    def get_action_epsilon_greedy(self, state):
        # Epsilon-greedily choose action
        rand = random.random()
        if rand > self.epsilon: # EXPLOIT
            print("Random %.2f > %.2f Epsilon (Get argmax action)" % (rand, self.epsilon))
            action = self.argmax_a(state)
        else: # EXPLORE
            print("Random %.2f < %.2f Epsilon (Get random action)" % (rand, self.epsilon))
            action = self.get_random_action(state)
        return action



    def train(self, env):
        # Reset environment
        self.env = env
        self.state = self.env.reset()

        # Episodes loop
        for episode in range(self.MAX_TRAINING_EPISODES):

            # Update statistics
            self.episode_rewards[episode] = 0

            # Steps in each episode
            for step in range(self.MAX_STEPS_PER_EPISODE):

                print("\n\nEpisode {}/{} @ Step {}".format(episode, self.MAX_TRAINING_EPISODES, step))
                print("Previous state: ", self.p_state)
                print("Previous action: ", self.p_action)
                print("Previous reward: ", self.p_reward)

                # Update frequencies and Q-Values
                if self.p_state != None:
                    # Update frequency of value
                    if Q_Value(self.p_state, self.p_action) in self.frequency and self.frequency[Q_Value(self.p_state, self.p_action)] > 0:
                        self.frequency[Q_Value(self.p_state, self.p_action)] += 1
                    else:
                        self.frequency[Q_Value(self.p_state, self.p_action)] = 1
                        self.q_values[Q_Value(self.p_state, self.p_action)] = 0
                    # Update Q-Value (Bellman equation)
                    q_sa = self.q_values[Q_Value(self.p_state, self.p_action)]
                    self.q_values[Q_Value(self.p_state, self.p_action)] = q_sa + self.alpha * (self.reward + self.gamma * self.max_a(self.state) - q_sa)
                
                # Get action
                self.action = self.get_action_epsilon_greedy(self.state)
                print("Chosen action: ", self.action)

                # Save previous state and reward
                self.p_state = self.state
                self.p_reward = self.reward

                # Execute action in the environment
                self.state, self.reward, done = self.env.step(self.action)
                print("Resulting state: ", self.state)
                print("Resulting reward: ", self.reward)

                # Save previous action
                self.p_action = self.action

                # Update statistics
                self.episode_rewards[episode] += self.reward

                # If episode's last execution
                if step+1 == self.MAX_STEPS_PER_EPISODE or done:

                    print("Total reward in episode {}: {}".format(episode, self.episode_rewards[episode]))

                    # Decrease epsilon value by half
                    self.epsilon -= self.epsilon * 0.01

                    # Reset environment and attributes
                    self.state = self.env.reset()
                    self.action = None
                    self.reward = None
                    self.p_state = None
                    self.p_action = None
                    self.p_reward = None

                    self.rewards_list.append(self.episode_rewards[episode])

                    # if episode+1 % 20 == 0:
                    plt.plot(self.rewards_list)
                    plt.ylabel('Reward')
                    plt.xlabel('Episode')
                    plt.savefig("rewards.png", bbox_inches="tight")
                    plt.draw()
                    plt.pause(0.001)
                    

                    # Go to next episode
                    break



if __name__ == '__main__':
    pass
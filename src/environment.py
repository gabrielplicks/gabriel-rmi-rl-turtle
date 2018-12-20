#!/usr/bin/env python

import rospy
import std_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from agent import Agent

import time, math


class Environment:


    def __init__(self):

        # Start node
        # rospy.init_node('environment', anonymous=True)

        self.start_pos = (1.5, 1.5)
        self.dest_pos = (9.0, 1.5)

        # self.prev_state = self.start_pos

        # Robot variables
        # self.odom = None
        self.base_pose = None
        self.laser_distance = None

        self.laser_threshold = 0.7
        self.rate = rospy.Rate(10)

        # Pubs
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Subs
        # rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.base_pose_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)



    def step(self, action):
        # Execute the action
        done = self.execute_action(action)
        # print("Action:", action)

        # Get new state (X,Y)
        state = self.get_state()
        # print("State:", state)

        # Get state reward
        reward = self.get_reward(state, action)
        if done == True:
            reward -= 100
        # print("Reward:", reward)

        # Check if is state arrived is goal state
        if state[0] >= self.dest_pos[0]:
            print("============= ARRIVED AT GOAL STATE =============")
            done = True
            reward += 50

        # self.prev_state = state

        return state, reward, done



    def get_available_actions(self, state):
        available_actions = list()
        available_actions.append('FORWARD')
        available_actions.append('LEFT')
        available_actions.append('RIGHT')
        return available_actions



    def get_reward(self, state, action):
        # Check chosen action
        if action == 'FORWARD':
            reward = 20
        elif action == 'LEFT':
            reward = -20
        elif action == 'RIGHT':
            reward = -20
        
        # Check Euclidean distance
        # distance = math.sqrt((self.dest_pos[0] - state[0]) ** 2 + (self.dest_pos[1] - state[1]) ** 2)
        # print("!!!!!!!!!!!!!Distance = ", distance)

        # reward += reward * distance

        # reward -= reward * self.laser_distance
        
        return reward



    def get_state(self):
        # Get current X, Y coordinates
        rospy.sleep(0.5)
        # state = (self.odom.position.x, self.odom.position.y)
        state = (self.base_pose.position.x, self.base_pose.position.y)
        return state



    def reset(self):
        # Move the robot back to initial position
        # rospy.ServiceProxy('reset_positions', ResetPositions)
        from subprocess import call
        call(["rosservice", "call", "reset_positions"])
        
        # Return the initial position
        return self.start_pos

    

    def execute_action(self, action):
        twist = Twist()

        twist.linear.x = 0.5

        if action == 'FORWARD':
            twist.angular.z = 0.0
        elif action == 'LEFT':
            twist.angular.z = 0.5
        elif action == 'RIGHT':
            twist.angular.z = -0.5

        # print(twist)

        # Send command
        done = False
        now = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() < now + 1):
            
            # Verify if robot is too close to the wall
            if (self.laser_distance < self.laser_threshold):
                # print('Laser:', self.laser_distance)
                # twist.linear.x = 0
                # self.cmd_vel_pub.publish(twist)
                done = True
                break
                
            # Publish
            self.cmd_vel_pub.publish(twist)

            # self.rate.sleep()

        # Stop the robot
        twist.linear.x = 0
        twist.angular.z = 0
        # print("Stopping")
        self.cmd_vel_pub.publish(twist)
        self.rate.sleep()

        return done



    ### ROS CALLBACKS

    # def odom_callback(self, data):
    #     self.odom = data.pose.pose
    
    def base_pose_callback(self, data):
        # rospy.sleep(0.1)
        self.base_pose = data.pose.pose
    
    def laser_callback(self, data):
        # print("!!!!!!!!!!!!!!!!!!!!Range:", data)
        # print("Min range:", min(data.ranges))
        # rospy.sleep(0.1)
        self.laser_distance = min(data.ranges)
        # rospy.sleep(1)



if __name__ == '__main__':
    rospy.init_node('rl_turtle', anonymous=True)

    # env = Environment()
    # rospy.sleep(1)
    # # print(env.odom.position)
    # print(env.get_state())
    # env.execute_action('FORWARD')
    # rospy.sleep(1)
    # # print(env.odom.position)
    # print(env.get_state())
    # env.execute_action('LEFT')
    # rospy.sleep(1)
    # # print(env.odom.position)
    # print(env.get_state())
    # env.execute_action('RIGHT')
    # rospy.sleep(1)
    # # print(env.odom.position)
    # print(env.get_state())


    agent = Agent()
    env = Environment()

    start_time = time.time()
    agent.train(env)
    elapsed_time = time.time() - start_time

    print("It took %.2f seconds to train." % elapsed_time)
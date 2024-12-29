import numpy as np
from RRTTree import RRTTree
import time
import math

class RRTPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        # step_size can be either 5, 10 or 15
        self.step_size = 5

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 3

        self.tree.add_vertex(self.planning_env.start)
        numOfStates = (self.planning_env.xlimits[1]+1)*(self.planning_env.ylimits[1]+1)
        '''not sure what is n in the algorithm!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'''
        for i in range(numOfStates):
            '''not sure what is n in the algorithm!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! '''
            x_rand = np.random.uniform(self.planning_env.xlimits[0], self.planning_env.xlimits[1])
            if self.planning_env.state_validity_checker(x_rand) == False:
                continue
            x_near_idx, x_near = self.tree.get_nearest_state(x_rand)
            x_new = self.extend(x_near,x_rand)
            self.tree.add_vertex(x_new)
            x_new_idx = self.tree.get_idx_for_state(x_new)
            if self.planning_env.edge_validity_checker(x_near,x_new) == False:
                continue
            edge_cost = self.planning_env.compute_distance(x_near, x_new)
            self.tree.add_edge(x_near_idx,x_new_idx,edge_cost)
            plan.append(x_new)
            '''maybe need to check both indexes'''
            if self.planning_env.goal == x_new:
                break
        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 3
        i = 0
        cost = 0
        while i < len(plan) - 1:
            cost += self.planning_env.compute_distance(plan[i], plan[i+1])
        return cost

    def extend(self, near_state, rand_state):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
        # TODO: Task 3
        x_new = [0,0]
        if (self.ext_mode == 'E1'):
            x_new = rand_state
        else:
            # Need to fix this!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            distance_vec = rand_state - near_state
            distance_vec_norm = math.sqrt(distance_vec[0]*distance_vec[0] + distance_vec[1]*distance_vec[1])
            distance_vec_normalized = distance_vec * (1/distance_vec_norm)
            x_new = near_state + distance_vec_normalized*self.step_size
        return x_new
import numpy as np

class RCSPlanner(object):    
    def __init__(self, planning_env):
        self.planning_env = planning_env

        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = [] 

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''

        # initialize an empty plan.
        plan = []

        # TODO: Task 4
        "Initialize coarse and fine sets with the desired actions"
        coarse_set = []
        fine_set = []
        for num1 in [-2, 0, 2]:
            for num2 in [-2, 0, 2]:
                coarse_set.append((num1, num2))
        coarse_set.remove((0, 0))
        for num1 in [-1, 0, 1]:
            for num2 in [-1, 0, 1]:
                fine_set.append((num1, num2))
        fine_set.remove((0, 0))

        "node = (state, rank, resolution, parent_node)"
        root_node = (self.planning_env.start, 0, "coarse", None)
        open_list = [root_node]
        close_list = []
        while True:
            "take the node with the minimal rank"
            v = min(open_list, key=lambda node: node[1])
            open_list.remove(v)
            if self.planning_env.state_validity_checker(v[0]):
                if v not in close_list:
                    if (v[0] == self.planning_env.goal).all():
                        self.reconstruct_path(v)
                    for action in coarse_set:
                        new_node = (v[0] + action, v[1] + 1, "coarse", v)
                        open_list.append(new_node)
                    close_list.append(v)
            if (v != root_node) and (v[2] == "coarse"):
                for action in fine_set:
                    new_node = (v[3][0] + action, v[3][1] + 1, "fine", v[3])
                    open_list.append(new_node)
            if len(open_list) == 0:
                break

        return np.array(plan)

    def reconstruct_path(self, node):
        '''
        Reconstruct the path from the goal to the start using parent pointers.
        # YOU DON'T HAVE TO USE THIS FUNCTION!!!
        '''
        path = []
        while node:
            path.append(node.state)  # Append the state
            node = node.parent  # Move to the parent
        path.reverse()
        print(path)
        return np.array(path)
    
    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        DO NOT MODIFY THIS FUNCTION!!!
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes

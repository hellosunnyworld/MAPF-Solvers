import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, all_eq
from copy import deepcopy

def detect_first_collision_for_path_pair(path1, path2):
    ##############################
    # Task 2.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    for i in range(max(len(path1), len(path2))):
        if all_eq(get_location(path1, i), get_location(path2, i)):
            return {'loc': [get_location(path1, i)], 'timestep': i}
        if i > 0:
            if all_eq(get_location(path1, i-1), get_location(path2, i)) and all_eq(get_location(path1, i), get_location(path2, i - 1)):
                return {'loc': [get_location(path2, i), get_location(path1, i)], 'timestep': i}
            
    # end1 = [get_location(path1,-1), get_location(path2, -1)][len(path1) > len(path2)]
    # long_p = [path1, path2][len(path1) < len(path2)]
    # for i in range(min(len(path1), len(path2)), max(len(path1), len(path2))):
    #     if all_eq(get_location(long_p, i), end1):
    #         return {'loc': [end1], 'timestep': i}
    return None


def detect_collisions_among_all_paths(paths):
    ##############################
    # Task 2.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            c = detect_first_collision_for_path_pair(paths[i], paths[j])
            if c != None:
                c['a1'] = i
                c['a2'] = j
                collisions.append(c)
    return collisions



def standard_splitting(collision):
    ##############################
    # Task 2.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []
    if len(collision['loc']) == 1: # vertex collision
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']})
    else: # edge collision
        constraints.append({'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints.append({'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]], 'timestep': collision['timestep']})
    return constraints


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations

        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        new_node = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        root = deepcopy(new_node)
        a_stars = 0
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            a_stars += 1
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions_among_all_paths(root['paths'])
        self.push_node(root)

        # Task 2.1: Testing
        #print(root['collisions'])

        # Task 2.2: Testing
        # for collision in root['collisions']:
        #     print(standard_splitting(collision))

        ##############################
        # Task 2.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        # These are just to print debug output - can be modified once you implement the high-level search
        while len(self.open_list) > 0:
            curr = self.pop_node()

            print('*********************')
            print('newly popped costs', curr['cost'])
            if curr['collisions'] == []:
                self.print_results(curr)
                return curr['paths']
            print('HAVE COLLISIONS', curr['collisions'])
            new_constr = standard_splitting(curr['collisions'][0])

            for c in new_constr:
                print('---------------------')
                child = deepcopy(new_node)
                child['constraints'] = deepcopy(curr['constraints'])
                child['paths'] = deepcopy(curr['paths'])
                
                if (c not in child['constraints']):
                    #print('new nc', c)
                    child['constraints'].append(c)

                print('CHILD constraints', child['constraints'], '\n\n\n')


                n = c['agent']
                low_level_constr = []
                for nc in child['constraints']:
                    #print('CHILD\'s agent', n, 'path planning constraints', nc)
                    if nc['agent'] == n:
                        low_level_constr.append(nc)
                        #print('append!')

                path = a_star(self.my_map, self.starts[n], self.goals[n], self.heuristics[n],
                            n, low_level_constr, cbs=self.num_of_agents)
                a_stars += 1
                if path is None:
                    print('CHILD FAILS')
                    continue
                child['paths'][n] = path
               # print('PATH',n, ':', child['paths'])


                child['cost'] = get_sum_of_cost(child['paths'])
                child['collisions'] = detect_collisions_among_all_paths(child['paths'])
                self.push_node(child)
                
        raise BaseException('No solutions')


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

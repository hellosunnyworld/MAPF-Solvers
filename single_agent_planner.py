import heapq
from itertools import permutations

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def move_joint_state(locs, dir):

    new_locs = [move(locs[i], dir[i]) for i in range(len(locs))]
    
    return new_locs

def generate_motions_recursive(num_agents,cur_agent):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    # Use combinations from itertools to choose two items without considering order
    combos = list(permutations(list(range(len(directions))), num_agents))
    print(combos)
    joint_state_motions = [list(combo) for combo in combos]

    return joint_state_motions + [[0]*num_agents,[1]*num_agents,[2]*num_agents]


def is_valid_motion(old_loc, new_loc):
    ##############################
    # Task 1.3/1.4: Check if a move from old_loc to new_loc is valid
    # Check if two agents are in the same location (vertex collision)
    # TODO
    if new_loc[0][0] == new_loc[1][0] and new_loc[0][1] == new_loc[1][1]:
        return False

    # Check edge collision
    # TODO
    if new_loc[0][0] == old_loc[1][0] and new_loc[0][1] == old_loc[1][1] and \
        old_loc[0][0] == new_loc[1][0] and old_loc[0][1] == new_loc[1][1]:
        return False
    return True

def get_sum_of_cost(paths):
    rst = 0
    if paths is None:
        return -1
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, path):
    ##############################
    # Task 1.2/1.3/1.4: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    # constraints:[agent][time]=constraints
    constraints.append([])
    #print(constraints)
    for i in range(len(path)):
        #print('-----------------')
        constraints[-1].append([])
        constraints[-1][i].append(path[i])
        if i > 0:
            constraints[-1][i].append([path[i-1], path[i]])
        #print(constraints)



def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path

def all_eq(loc1, loc2):
    return (loc1[0] == loc2[0] and loc1[1] == loc2[1])

def is_constrained(curr_loc, next_loc, next_time, constraint_table, cbs=False):
    ##############################
    # Task 1.2/1.3/1.4: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if not cbs:
        for cs in constraint_table:
            if next_loc is None and next_time < len(cs): # this agent has reached the goal:
                for c in cs[next_time:]:
                    for ct in c:
                        if type(ct[0]) == int and all_eq(curr_loc, ct): # vertex contraint
                            return True
            elif next_time < len(cs):
                for ct in cs[next_time]:
                    if type(ct[0]) == list or type(ct[0]) == tuple: # edge constraint
                        if (all_eq(curr_loc, ct[1]) and all_eq(next_loc, ct[0])) or (all_eq(curr_loc, ct[0]) and all_eq(next_loc, ct[1])):
                            return True
                    elif type(ct[0]) == int and all_eq(next_loc, ct): # vertex contraint
                        return True
            elif next_loc is not None:
                for ct in cs[-1]:
                    if type(ct[0]) == int and all_eq(next_loc, ct): # vertex contraint
                        return True            
    else:
        if next_loc == None:
            #print('\n\n\nNONE!! A* end')
            #print(constraint_table)
            for constr in constraint_table:
                #print(constr, '\n')
                if next_time <= constr['timestep'] and len(constr['loc']) == 1 and all_eq(curr_loc, constr['loc'][0]):   
                    print('ending fails')
                    return True

        else:
            end_of_constr_reached = True
            final_constr_time = -1
            for constr in constraint_table:
                if next_time < constr['timestep']:
                    end_of_constr_reached = False
                else:
                    if next_time == constr['timestep']:
                        if len(constr['loc']) == 2: # edge constraint
                            if (all_eq(curr_loc, constr['loc'][1]) and all_eq(next_loc, constr['loc'][0])) or (all_eq(curr_loc, constr['loc'][0]) and all_eq(next_loc, constr['loc'][1])):
                                return True
                        elif all_eq(next_loc, constr['loc'][0]): # vertex contraint
                            return True 

                    # if constr['timestep'] > final_constr_time:
                    #     final_constr_time = constr['timestep']

    return False  


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return (n1['g_val'] + n1['h_val']) < (n2['g_val'] + n2['h_val'])

def in_map(map, loc):
    if loc[0] >= len(map) or loc[1] >= len(map[0]) or min(loc) < 0:
        return False
    else:
        return True

def all_in_map(map, locs):
    for loc in locs:
        if not in_map(map, loc):
            return False
    return True

def mahanttan_dist(loc1, loc2):
    return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1])

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, cbs = False):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.2/1.3/1.4: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
    push_node(open_list, root)
    closed_list[(tuple(root['loc']), root['time_step'])] = root
    # print('A*A*A*A*A*A*A*A*A*A*')
    # print(constraints)

    if cbs:
        num_agents = cbs
    else:
        num_agents = len(constraints) + 1

    max_time = num_agents * (len(my_map) * len(my_map[0]) - sum([sum(grid) for grid in my_map]))
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 2.2: Adjust the goal test condition to handle goal constraints
        if curr['time_step'] > max_time:
            print("no solutions", agent)
            return None
        if curr['loc'] == goal_loc:
            if not is_constrained(curr['loc'], None, curr['time_step'] + 1, constraints, cbs): # ensure the other agent won't run into it later
                return get_path(curr)
        for dir in range(5): # allow staying still
            child_loc = move(curr['loc'], dir)
            if not in_map(my_map, child_loc) or my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'], child_loc, curr['time_step'] + 1, constraints, cbs):
                continue

            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr, 
                    'time_step': curr['time_step'] + 1}

            if (tuple(child['loc']), child['time_step']) in closed_list:
                existing_node = closed_list[(tuple(child['loc']), child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(tuple(child['loc']), child['time_step'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(tuple(child['loc']), child['time_step'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions

def joint_state_a_star(my_map, starts, goals, h_values, num_agents):
    """ my_map      - binary obstacle map
        start_loc   - start positions
        goal_loc    - goal positions
        num_agent   - total number of agents in fleet
    """

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = 0
     ##############################
    # Task 1.1: Iterate through starts and use list of h_values to calculate total h_value for root node
    #
    # TODO
    for i in range(num_agents):
        h_value += h_values[i][starts[i]]

    
    root = {'loc': starts, 'g_val': 0, 'h_val': h_value, 'parent': None }
    push_node(open_list, root)
    closed_list[tuple(root['loc'])] = root

     ##############################
    # Task 1.1:  Generate set of all possible motions in joint state space
    #
    # TODO
    directions = generate_motions_recursive(num_agents,0)
    while len(open_list) > 0:
        curr = pop_node(open_list)
        # print('---------------------------')
        # print('POPPED', curr['loc'], 'g=', curr['g_val'], 'h=', curr['h_val'], 'parent', curr['parent'])
        
        if curr['loc'] == goals:
            return get_path(curr)

        for dir in directions:
            
            ##############################
            # Task 1.1:  Update position of each agent
            #
            # TODO
            child_loc = move_joint_state(curr['loc'], dir) # curr['loc']: locations of all children. dir: dirs of all children
            
            if not all_in_map(my_map, child_loc):
                continue
             ##############################
            # Task 1.1:  Check if any agent is in an obstacle
            #
            valid_move = True
            # TODO
            valid_move = not (my_map[child_loc[0][0]][child_loc[0][1]] or my_map[child_loc[1][0]][child_loc[1][1]])
            if not valid_move:
                continue
             ##############################
            # Task 1.1:   check for collisions
            #
            # TODO
            if not is_valid_motion(curr['loc'],child_loc):
                continue
             ##############################
            # Task 1.1:  Calculate heuristic value
            #
            # TODO
            h_value = 0
            for i in range(num_agents):
                h_value += h_values[i][child_loc[i]]

            # Create child node
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + num_agents,
                    'h_val': h_value,
                    'parent': curr}
            if tuple(child['loc']) in closed_list:
                existing_node = closed_list[tuple(child['loc'])]
                if compare_nodes(child, existing_node):
                    closed_list[tuple(child['loc'])] = child
                    push_node(open_list, child)
            else:
                closed_list[tuple(child['loc'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
# def joint_state_a_star(my_map, starts, goals, h_values, num_agents):
#     open_list = []
#     closed_list = dict()

#     # Calculate total heuristic value for the root node
#     h_value = sum(h_values[i][starts[i]] for i in range(num_agents))

#     # Initialize root with a time step
#     root = {'loc': starts, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
#     push_node(open_list, root)
#     closed_list[(tuple(root['loc']), root['time_step'])] = root

#     # Generate set of all possible motions, including waiting in place
#     directions = generate_motions_recursive(num_agents, 0)  

#     while len(open_list) > 0:
#         curr = pop_node(open_list)

#         if curr['loc'] == goals:
#             return get_path(curr)

#         for dir in directions:
#             child_loc = move_joint_state(curr['loc'], dir)

#             if not all_in_map(my_map, child_loc):
#                 continue

#             valid_move = not any(my_map[loc[0]][loc[1]] for loc in child_loc)
#             if not valid_move:
#                 continue

#             if not is_valid_motion(curr['loc'], child_loc):
#                 continue

#             h_value = sum(h_values[i][child_loc[i]] for i in range(num_agents))

#             # Create child node with incremented time step
#             child = {'loc': child_loc, 
#                     'g_val': curr['g_val'] + 1, 
#                     'h_val': h_value, 
#                     'parent': curr, 
#                     'time_step': curr['time_step'] + 1}

#             # Use (cell, time_step) tuple as key for closed_list
#             if (tuple(child['loc']), child['time_step']) in closed_list:
#                 existing_node = closed_list[(tuple(child['loc']), child['time_step'])]
#                 if compare_nodes(child, existing_node):
#                     closed_list[(tuple(child['loc']), child['time_step'])] = child
#                     push_node(open_list, child)
#             else:
#                 closed_list[(tuple(child['loc']), child['time_step'])] = child
#                 push_node(open_list, child)

#     return None

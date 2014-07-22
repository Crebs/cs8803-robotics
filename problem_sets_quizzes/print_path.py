# ----------
# User Instructions:
# 
# Implement the function optimum_policy2D() below.
#
# You are given a car in a grid with initial state
# init = [x-position, y-position, orientation]
# where x/y-position is its position in a given
# grid and orientation is 0-3 corresponding to 'up',
# 'left', 'down' or 'right'.
#
# Your task is to compute and return the car's optimal
# path to the position specified in `goal'; where
# the costs for each motion are as defined in `cost'.

# EXAMPLE INPUT:

# grid format:
#     0 = navigable space
#     1 = occupied space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]
goal = [2, 0] # final position
init = [4, 3, 0] # first 2 elements are coordinates, third is direction
cost = [2, 1, 20] # the cost field has 3 values: right turn, no turn, left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D() should return the array
# 
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
#
# ----------


# there are four motion directions: up/left/down/right
# increasing the index in this array corresponds to
# a left turn. Decreasing is is a right turn.

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # do right
forward_name = ['up', 'left', 'down', 'right']

# the cost field has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']


# ----------------------------------------
# modify code below
# ----------------------------------------

def optimum_policy2D():
    value = [[[999 for _ in range(4)] for x in grid[0]]  for y in grid]
    policy = [[[' ' for _ in range(4)] for x in grid[0]]  for y in grid]
    policy2D = [[' ' for l in grid[0]] for w in grid]    
    queue = []

    for i in range(4):
        value[goal[0]][goal[1]][i] = 0
   
    for motion in forward:
        for orientation in range(4):
            x = goal[0] + motion[0]
            y = goal[1] + motion[1]

            if x < 0 or y <0 or x >= len(grid) or y >= len(grid[0]) or grid[x][y] == 1:
                continue
            queue.append([x,y,orientation])


    while len(queue) != 0:
        current_x, current_y, current_orientation = queue.pop(0)
        
        for motion_idx, [motion_x, motion_y] in enumerate(forward):
            next_x = current_x + motion_x
            next_y = current_y + motion_y
            
            #illegal grid
            if next_x < 0 or next_y <0 or next_x >= len(grid) or next_y >= len(grid[0]) or grid[next_x][next_y] == 1:
                continue
            
            next_orientation = motion_idx
            next_value = value[next_x] [next_y] [next_orientation]
            #specify the move_cost to be really if unreachable
            move_cost = 999
            
            #calculates the cost to get from current orientation to the next orientation
            for action_idx, act in enumerate(action):
                if (act + current_orientation)%4 == next_orientation:
                    move_cost = cost[action_idx]
                    movement = action_idx

            new_value = next_value + move_cost

            if new_value < value[current_x][current_y][current_orientation]:
                value[current_x][current_y][current_orientation] = new_value
                policy[current_x][current_y][current_orientation] = action_name[movement]
                
                #because value changed, we add back surrounding nodes minus the 'next node' to queue of nodes to be expanded
                for motion_idx2, [motion_x2, motion_y2] in enumerate(forward):
                    if motion_idx2 != motion_idx:
                        for o in range(4):
                            next_x = current_x + motion_x2
                            next_y = current_y + motion_y2
                            
                            #illegal grid
                            if next_x < 0 or next_y <0 or next_x >= len(grid) or next_y >= len(grid[0]) or grid[next_x][next_y] == 1:
                                continue
                            else:
                                queue.append([next_x, next_y, o])


    policy2D[goal[0]][goal[1]] = '*'
    #computes policy given a starting position
    start_x, start_y, start_orientation = init
    while start_x != goal[0] or start_y != goal[1]:
        move = policy[start_x][start_y][start_orientation]
        policy2D[start_x][start_y] = move

        start_x = start_x + forward[start_orientation + action[action_name.index(move)]][0]
        start_y = start_y + forward[start_orientation + action[action_name.index(move)]][1]
        start_orientation = (start_orientation + action[action_name.index(move)]) %  4
        
    return policy2D

for l in optimum_policy2D():
    print l


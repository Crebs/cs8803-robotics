# --------------
# USER INSTRUCTIONS
#
# Write a function called stochastic_value that 
# takes no input and RETURNS two grids. The
# first grid, value, should contain the computed
# value of each cell as shown in the video. The
# second grid, policy, should contain the optimum
# policy for each cell.
#
# Stay tuned for a homework help video! This should
# be available by Thursday and will be visible
# in the course content tab.
#
# Good luck! Keep learning!
#
# --------------
# GRADING NOTES
#
# We will be calling your stochastic_value function
# with several different grids and different values
# of success_prob, collision_cost, and cost_step.
# In order to be marked correct, your function must
# RETURN (it does not have to print) two grids,
# value and policy.
#
# When grading your value grid, we will compare the
# value of each cell with the true value according
# to this model. If your answer for each cell
# is sufficiently close to the correct answer
# (within 0.001), you will be marked as correct.
#
# NOTE: Please do not modify the values of grid,
# success_prob, collision_cost, or cost_step inside
# your function. Doing so could result in your
# submission being inappropriately marked as incorrect.

# -------------
# GLOBAL VARIABLES
#
# You may modify these variables for testing
# purposes, but you should only modify them here.
# Do NOT modify them inside your stochastic_value
# function.

grid = [[0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 1, 1, 0]]
       
goal = [0, len(grid[0])-1] # Goal is in top right corner


delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>'] # Use these when creating your policy grid.

success_prob = 0.5                      
failure_prob = (1.0 - success_prob)/2.0 # Probability(stepping left) = prob(stepping right) = failure_prob
collision_cost = 100                    
cost_step = 1        
                     

############## INSERT/MODIFY YOUR CODE BELOW ##################
#
# You may modify the code below if you want, but remember that
# your function must...
#
# 1) ...be called stochastic_value().
# 2) ...NOT take any arguments.
# 3) ...return two grids: FIRST value and THEN policy.

def valid(x,y):
    return x >= 0 and y >= 0  and x < len(grid) and y < len(grid[0]) and grid[x][y] == 0

def stochastic_value():
    value = [[1000 for row in range(len(grid[0]))] for col in range(len(grid))]
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    policy[goal[0]][goal[1]] = '*'
    queue = []
    value[goal[0]][goal[1]] = 0
    for idx, [action_x, action_y] in enumerate(delta):
        current_x = goal[0] + action_x
        current_y = goal[1] + action_y 
        if valid(current_x, current_y):
            queue.append([current_x, current_y])        
           
    while len(queue) != 0:
        current = queue.pop(0)
        current_x = current[0]
        current_y = current[1]
        
        for idx, [action_x, action_y] in enumerate(delta):
            x = current_x + action_x
            y = current_y + action_y
            if valid(x, y):
                new_cost = value[x][y] * success_prob
                #calculate the left cost
                l = delta[ (idx+1)%len(delta)]
                lx = current_x + l[0]
                ly = current_y + l[1]
                if valid(lx, ly):
                    new_cost += failure_prob * value[lx][ly]
                else:
                    new_cost += failure_prob * collision_cost
                
                new_cost += 1
                r = delta[ (idx-1)%len(delta)]
                rx = current_x + r[0]
                ry = current_y + r[1]
                if valid(rx, ry):
                    new_cost += failure_prob * value[rx][ry]
                else:
                    new_cost += failure_prob * collision_cost

                if new_cost < value[current_x][current_y]:
                    value[current_x][current_y] = new_cost
                    policy[current_x][current_y] = delta_name[idx]
                    
                    
                    for idx2, [action_x2, action_y2] in enumerate(delta): 
                        recal_x = current_x + action_x2
                        recal_y = current_y + action_y2
                        if valid(recal_x, recal_y):
                            queue.append([recal_x, recal_y])
                            
    
    return value, policy


for l in stochastic_value()[0]:
    print l
    
for l in stochastic_value()[1]:
    print l
    

# ----------
# User Instructions:
# 
# Define a function, search() that takes no input
# and returns a list
# in the form of [optimal path length, x, y]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1] # Make sure that the goal definition stays in the function.

delta = [[-1, 0 ], # go up
        [ 0, -1], # go left
        [ 1, 0 ], # go down
        [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

cost = 1

#a greedy first search algorithm
def search():
    # ----------------------------------------
    # insert code here and make sure it returns the appropriate result
    # ----------------------------------------
    next_node = init
    visited_nodes = set([(next_node[0], next_node[1])])
    new_cost = 0
        
    candidates = []
    i = 0
    while next_node != goal:
        y,x = next_node
        candidates += [(y+delta_y,x+delta_x,new_cost + cost) for delta_y, delta_x in delta 
                if y+delta_y >=0 and y+delta_y<len(grid) and x+delta_x >= 0 and x+delta_x < len(grid[0]) and grid[y+delta_y][x+delta_x] != 1 
                and (y+delta_y,x+delta_x) not in visited_nodes]
        #expand the cheapest node first
        lowest_cost = 1000000
        next_index = -1
        print 'len: %s' %len(candidates)
        for index, candidate in enumerate(candidates):
            y,x,path_cost = candidate
            if lowest_cost > path_cost and (y,x) not in visited_nodes:
                next_index = index
                lowest_cost = path_cost
        if len(candidates) == 0 :
            return 'fail'
        
        print 'index: %s candidates: %s visited: %s' %(next_index, candidates, visited_nodes)
        n = candidates.pop(next_index)
        next_node = [n[0], n[1]]
        visited_nodes.add(tuple(next_node))
        new_cost = n[2]

    path = [new_cost,next_node[0], next_node[1]]
    return path # you should RETURN your result

print search()

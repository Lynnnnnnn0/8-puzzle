
# coding: utf-8

# In[ ]:

import sys
import math
import queue
from collections import deque
import heapq
import time
import resource


# In[ ]:

def exploringNeighbors_bfs(l, n):
    neighbors = []
    ind = l.index(0)
    if ind == 0:
        move = 'Down'
        swape(l, 0, n, move, neighbors)
        move = 'Right'
        swape(l, 0, 1, move, neighbors)
    elif ind == n-1:
        move = 'Down'
        swape(l, n-1, 2*n-1, move, neighbors)
        move = 'Left'
        swape(l, n-1, n-2, move, neighbors)
    elif ind == n*n-n:
        move = 'Up'
        swape(l, n*n-n, n*n-2*n, move, neighbors)
        move = 'Right'
        swape(l, n*n-n, n*n-n+1, move, neighbors)
    elif ind == n*n-1:
        move = 'Up'
        swape(l, n*n-1, n*n-n-1, move, neighbors)
        move = 'Left'
        swape(l, n*n-1, n*n-2, move, neighbors)
    elif ind>0 and ind<n-1:
        move = 'Down'
        swape(l, ind, ind+n, move, neighbors)
        move = 'Left'
        swape(l, ind, ind-1, move, neighbors)
        move = 'Right'
        swape(l, ind, ind+1, move, neighbors)
    elif (ind/n).is_integer() and (ind/n)>0 and (ind/n)<n-1:
        move = 'Up'
        swape(l, ind, ind-n, move, neighbors)
        move = 'Down'
        swape(l, ind, ind+n, move, neighbors)
        move = 'Right'
        swape(l, ind, ind+1, move, neighbors)
    elif ((ind+1)/n).is_integer() and (ind+1)/n>1 and (ind+1)/n<n:
        move = 'Up'
        swape(l, ind, ind-n, move, neighbors)
        move = 'Down'
        swape(l, ind, ind+n, move, neighbors)
        move = 'Left'
        swape(l, ind, ind-1, move, neighbors)
    elif ind>n*n-n and ind<n*n-1:
        move = 'Up'
        swape(l, ind, ind-n, move, neighbors)
        move = 'Left'
        swape(l, ind, ind-1, move, neighbors)
        move = 'Right'
        swape(l, ind, ind+1, move, neighbors)
    else:
        move = 'Up'
        swape(l, ind, ind-n, move, neighbors)
        move = 'Down'
        swape(l, ind, ind+n, move, neighbors)
        move = 'Left'
        swape(l, ind, ind-1, move, neighbors)
        move = 'Right'
        swape(l, ind, ind+1, move, neighbors)
    return neighbors


# In[ ]:

def swape(l, i, j, move, neighbors):
    l[i], l[j] = l[j], l[i]
    #[child list, move, parent node]
    neighbors.append([list(l), move, l])
    l[i], l[j] = l[j], l[i]


# In[ ]:

def bfs(l1,l2, n):
    start_time = time.time()
    initialState = l1
    goalState = l2
    explored_and_frontier = set()
    explored_and_frontier.add(tuple(initialState))
    d = dict()
    num_nodes_expanded = 0
    max_fringe_size = 0
    max_depth = 0
    frontier = queue.Queue(maxsize=0)
    frontier.put(initialState)
    depth = queue.Queue(maxsize=0)
    depth.put(0)
    while not frontier.empty():
        state = frontier.get()
        dep = depth.get()
        if state == goalState:
            path_to_goal = findingPath(d, tuple(initialState), tuple(goalState))
            file = open('output.txt','w') 
            file.write("path_to_goal: " + str(path_to_goal) +'\n')
            file.write("cost_of_path: " + str(len(path_to_goal)) +'\n')
            file.write("nodes_expanded: " + str(num_nodes_expanded) +'\n')
            file.write("fringe_size: " + str(frontier.qsize()) +'\n')
            file.write("max_fringe_size: " + str(max_fringe_size) +'\n')
            file.write("search_depth: " + str(len(path_to_goal)) +'\n')
            file.write("max_search_depth: " + str(max_depth) +'\n')
            file.write("running_time: " + str(round(time.time() - start_time,8)) +'\n')
            file.write("max_ram_usage: " + str(round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*(10**(-6)),8)) +'\n')
            file.close()
            return
        neighbors = exploringNeighbors_bfs(state, n)
        num_nodes_expanded += 1
        for neighbor in neighbors:
            node = tuple(neighbor[0])
            if node not in explored_and_frontier:# and node not in frontier.queue:
                frontier.put(neighbor[0])
                depth.put(dep+1)
                explored_and_frontier.add(node)
                d[node] = (neighbor[1],tuple(neighbor[2]))
        fringe_size = frontier.qsize()
        if fringe_size > max_fringe_size:
            max_fringe_size = fringe_size
        if dep+1 > max_depth:
            max_depth = dep+1
    return False
        


# In[ ]:

def exploringNeighbors_dfs(l, n):
    neighbors = []
    ind = l.index(0)
    if ind == 0:
        move = 'Right'
        swape(l, 0, 1, move, neighbors)
        move = 'Down'
        swape(l, 0, n, move, neighbors)
    elif ind == n-1:
        move = 'Left'
        swape(l, n-1, n-2, move, neighbors)
        move = 'Down'
        swape(l, n-1, 2*n-1, move, neighbors)
    elif ind == n*n-n:
        move = 'Right'
        swape(l, n*n-n, n*n-n+1, move, neighbors)
        move = 'Up'
        swape(l, n*n-n, n*n-2*n, move, neighbors)
    elif ind == n*n-1:
        move = 'Left'
        swape(l, n*n-1, n*n-2, move, neighbors)
        move = 'Up'
        swape(l, n*n-1, n*n-n-1, move, neighbors)
    elif ind>0 and ind<n-1:
        move = 'Right'
        swape(l, ind, ind+1, move, neighbors)
        move = 'Left'
        swape(l, ind, ind-1, move, neighbors)
        move = 'Down'
        swape(l, ind, ind+n, move, neighbors)
    elif (ind/n).is_integer() and (ind/n)>0 and (ind/n)<n-1:
        move = 'Right'
        swape(l, ind, ind+1, move, neighbors)
        move = 'Down'
        swape(l, ind, ind+n, move, neighbors)
        move = 'Up'
        swape(l, ind, ind-n, move, neighbors)
    elif ((ind+1)/n).is_integer() and (ind+1)/n>1 and (ind+1)/n<n:
        move = 'Left'
        swape(l, ind, ind-1, move, neighbors)
        move = 'Down'
        swape(l, ind, ind+n, move, neighbors)
        move = 'Up'
        swape(l, ind, ind-n, move, neighbors)
    elif ind>n*n-n and ind<n*n-1:
        move = 'Right'
        swape(l, ind, ind+1, move, neighbors)
        move = 'Left'
        swape(l, ind, ind-1, move, neighbors)
        move = 'Up'
        swape(l, ind, ind-n, move, neighbors)
    else:
        move = 'Right'
        swape(l, ind, ind+1, move, neighbors)
        move = 'Left'
        swape(l, ind, ind-1, move, neighbors)
        move = 'Down'
        swape(l, ind, ind+n, move, neighbors)
        move = 'Up'
        swape(l, ind, ind-n, move, neighbors)
    return neighbors


# In[ ]:

def findingPath(d, initial, goal):
    path = []
    while goal != initial:
        value = d[goal]
        path.insert(0, value[0])
        goal = value[1] 
    return path


# In[ ]:

def dfs(l1,l2, n):
    start_time = time.time()
    initialState = tuple(l1)
    goalState = tuple(l2)
    explored_and_frontier = set()
    explored_and_frontier.add(tuple(initialState))
    d = dict()
    num_nodes_expanded = 0
    max_fringe_size = 0
    max_depth = 0
    frontier = deque()  # use as stack
    frontier.append(initialState)
    depth = deque()
    depth.append(0)
    while frontier:
        state = frontier.pop()
        dep = depth.pop()
        if state == goalState:
            file = open('output.txt','w') 
            path_to_goal = findingPath(d, initialState, goalState)
            file.write("path_to_goal: " + str(path_to_goal) +'\n')
            file.write("cost_of_path: " + str(len(path_to_goal)) +'\n')
            file.write("nodes_expanded: " + str(num_nodes_expanded) +'\n')
            file.write("fringe_size: " + str(len(frontier)) +'\n')
            file.write("max_fringe_size: " + str(max_fringe_size) +'\n')
            file.write("search_depth: " + str(len(path_to_goal)) +'\n')
            file.write("max_search_depth: " + str(max_depth) +'\n')
            file.write("running_time: " + str(round(time.time() - start_time,8)) +'\n')
            file.write("max_ram_usage: " + str(round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*(10**(-6)),8)) +'\n')
            file.close()
            return
        neighbors = exploringNeighbors_dfs(list(state), n)
        num_nodes_expanded += 1
        count = 0
        for neighbor in neighbors:
            node = tuple(neighbor[0])
            if node not in explored_and_frontier: #list(zip(*path))[0]:# and node not in frontier:
                frontier.append(node)
                depth.append(dep+1)
                explored_and_frontier.add(node)
                d[node] = (neighbor[1],tuple(neighbor[2]))
                count += 1
        fringe_size = len(frontier)
        if fringe_size > max_fringe_size:
            max_fringe_size = fringe_size
        if count > 0:
            if dep+1 > max_depth:
                max_depth = dep+1
    return False          


# In[ ]:

def hcost(state):
    cost = 0
    ind_1 = state.index(1)
    ind_2 = state.index(2)
    ind_3 = state.index(3)
    ind_4 = state.index(4)
    ind_5 = state.index(5)
    ind_6 = state.index(6)
    ind_7 = state.index(7)
    ind_8 = state.index(8)
    #cost for 1
    if ind_1 == 0 or ind_1 == 2 or ind_1 == 4: 
        cost += 1
    elif ind_1 == 3 or ind_1 == 5 or ind_1 ==7: 
        cost += 2
    elif ind_1 == 6 or ind_1 == 8: 
        cost += 3
    #cost for 2
    if ind_2 == 1 or ind_2 == 5:
        cost += 1
    elif ind_2 == 0 or ind_2 == 4 or ind_2 == 8: 
        cost += 2
    elif ind_2 == 3 or ind_2 == 7: 
        cost += 3
    elif ind_2 == 6:
        cost += 4
    #cost for 3
    if ind_3 == 0 or ind_3 == 4 or ind_3 == 6:
        cost += 1
    elif ind_3 == 1 or ind_3 == 5 or ind_3 == 7:
        cost += 2
    elif ind_3 == 2 or ind_3 == 8:
        cost += 3
    #cost for 4:
    if ind_4 == 1 or ind_4 == 3 or ind_4 == 5 or ind_4 == 7:
        cost += 1
    elif ind_4 == 0 or ind_4 == 2 or ind_4 == 6 or ind_4 == 8:
        cost += 2
    #cost for 5
    if ind_5 == 2 or ind_5 == 4 or ind_5 == 8:
        cost += 1
    elif ind_5 == 1 or ind_5 == 3 or ind_5 == 7:
        cost += 2
    elif ind_5 == 0 or ind_5 == 6:
        cost += 3
    #cost for 6:
    if ind_6 == 3 or ind_6 == 7:
        cost += 1
    elif ind_6 == 0 or ind_6 == 4 or ind_6 == 8:
        cost += 2
    elif ind_6 == 1 or ind_6 == 5:
        cost += 3
    elif ind_6 == 2:
        cost += 4
    #cost for 7
    if ind_7 == 4 or ind_7 == 6 or ind_7 == 8:
        cost += 1
    elif ind_7 == 1 or ind_7 == 3 or ind_7 == 5:
        cost += 2
    elif ind_7 == 0 or ind_7 == 2:
        cost += 3
    #cost for 8
    if ind_8 == 5 or ind_8 == 7:
        cost += 1
    elif ind_8 == 2 or ind_8 == 4 or ind_8 == 6:
        cost += 2
    elif ind_8 == 1 or ind_8 == 3:
        cost += 3
    elif ind_8 == 0:
        cost += 4
    return cost


# In[ ]:

def ast(l1,l2, n):
    start_time = time.time()
    initialState = l1
    goalState = l2
    num_nodes_expanded = 0
    max_fringe_size = 0
    frontier = []
    explored_and_frontier = set()
    explored_and_frontier.add(tuple(initialState))
    d = dict()
    child = []
    move = []
    parent = []
    depth = []
    max_depth = 0
    heapq.heappush(frontier, (hcost(initialState), 0, initialState))
    while frontier:
        state = heapq.heappop(frontier)
        if state[2] == goalState:
            file = open('output.txt','w') 
            path_to_goal = findingPath(d, tuple(initialState), tuple(goalState))
            file.write("path_to_goal: " + str(path_to_goal) +'\n')
            file.write("cost_of_path: " + str(len(path_to_goal)) +'\n')
            file.write("nodes_expanded: " + str(num_nodes_expanded) +'\n')
            file.write("fringe_size: " + str(len(frontier)) +'\n')
            file.write("max_fringe_size: " + str(max_fringe_size) +'\n')
            file.write("search_depth: " + str(len(path_to_goal)) +'\n')
            file.write("max_search_depth: " + str(max_depth) +'\n')
            file.write("running_time: " + str(round(time.time() - start_time,8)) +'\n')
            file.write("max_ram_usage: " + str(round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*(10**(-6)),8)) +'\n')
            file.close() 
            return
        neighbors = exploringNeighbors_bfs(state[2], n)
        num_nodes_expanded += 1
        for neighbor in neighbors:
            node = [hcost(neighbor[0])+ state[1], state[1]+1, neighbor[0]]
            if tuple(node[2]) not in explored_and_frontier:
                heapq.heappush(frontier, node)
                explored_and_frontier.add(tuple(neighbor[0]))
                d[tuple(neighbor[0])] = (neighbor[1],tuple(neighbor[2]))
            elif node[2] in list(zip(*frontier))[2]:
                #decreasekey
                for entry in frontier:
                    if entry[2] == node[2]:
                        if entry[0] > node[0]:
                            frontier.remove(entry)
                            heapq.heapify(frontier)
                            heapq.heappush(frontier,node)
                            d[tuple(neighbor[0])] = (neighbor[1],tuple(neighbor[2]))
        fringe_size = len(frontier)
        if fringe_size > max_fringe_size:
            max_fringe_size = fringe_size
        if state[1]+1 > max_depth:
            max_depth = state[1]+1
    return False


# In[ ]:

def ida(l1, l2, n):
    start_time = time.time()
    initialState = tuple(l1)
    goalState = tuple(l2)
    num_nodes_expanded = 0
    max_fringe_size = 0
    max_depth = 0
    cost_limit = hcost(initialState)
    cost_increment = 1
    while True:
        explored_and_frontier = set()
        explored_and_frontier.add(tuple(initialState))
        d = dict()
        cost = dict()
        cost[initialState] = hcost(initialState)
        frontier = deque()
        frontier.append(initialState)
        depth = deque()
        depth.append(0)
        while frontier:
            state = frontier.pop()
            dep = depth.pop()
            if state == goalState:
                file = open('output.txt','w') 
                path_to_goal = findingPath(d, initialState, goalState)
                file.write("path_to_goal: " + str(path_to_goal) +'\n')
                file.write("cost_of_path: " + str(len(path_to_goal)) +'\n')
                file.write("nodes_expanded: " + str(num_nodes_expanded) +'\n')
                file.write("fringe_size: " + str(len(frontier)) +'\n')
                file.write("max_fringe_size: " + str(max_fringe_size) +'\n')
                file.write("search_depth: " + str(len(path_to_goal)) +'\n')
                file.write("max_search_depth: " + str(max_depth) +'\n')
                file.write("running_time: " + str(round(time.time() - start_time,8)) +'\n')
                file.write("max_ram_usage: " + str(round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*(10**(-6)),8)) +'\n')
                file.close() 
                return
            neighbors = exploringNeighbors_dfs(list(state), n)
            num_nodes_expanded += 1
            for neighbor in neighbors:
                node = tuple(neighbor[0])
                if hcost(node)+dep+1 <= cost_limit:
                    if node not in explored_and_frontier: #list(zip(*path))[0]:# and node not in frontier:
                        frontier.append(node)
                        explored_and_frontier.add(node)
                        depth.append(dep+1)
                        d[node] = (neighbor[1], tuple(neighbor[2]))
                        cost[node] = hcost(node)+dep+1
                    elif cost[node] >= hcost(node)+dep+1:##?
                        cost[node] = hcost(node)+dep+1
                        d[node] = (neighbor[1], tuple(neighbor[2]))
                        frontier.append(node)
                        depth.append(dep+1)
            fringe_size = len(frontier)
            if fringe_size > max_fringe_size:
                max_fringe_size = fringe_size
            if dep+1 > max_depth:
                max_depth = dep+1
        cost_limit += cost_increment
    return False     


# In[ ]:

def main():
    alg = sys.argv[1]
    initial = list(map(int,sys.argv[2].split(',')))
    goal = list(initial)
    goal.sort()
    n = int(math.sqrt(len(initial)))
    if alg == 'bfs':
        bfs(initial,goal, n)
    elif alg == 'dfs':
        dfs(initial,goal, n)
    elif alg == 'ast':
        ast(initial,goal, n)
    elif alg == 'ida':
        ida(initial,goal, n)


# In[ ]:

main()


# In[ ]:




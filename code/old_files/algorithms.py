# - *- coding: utf- 8 - *-
# implementation of each search algorithm in a size X size coordinate space

import random
import math
import time
import sys

graph= [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
        [4, 0, 8, 0, 0, 0, 0, 11, 0], 
        [0, 8, 0, 7, 0, 4, 0, 0, 2], 
        [0, 0, 7, 0, 9, 14, 0, 0, 0], 
        [0, 0, 0, 9, 0, 10, 0, 0, 0], 
        [0, 0, 4, 14, 10, 0, 2, 0, 0], 
        [0, 0, 0, 0, 0, 2, 0, 1, 6], 
        [8, 11, 0, 0, 0, 0, 1, 0, 7], 
        [0, 0, 2, 0, 0, 0, 6, 7, 0]]

size = 10
dimension = 2

start_list = list()
goal_list = list()

for x in range(dimension):
    start = random.randint(0, size)
    goal = random.randint(0, size)
    while goal == start:
        goal = random.randint(0, size)
    start_list.append(start)
    goal_list.append(goal)

start_tup = tuple(start_list)
goal_tup = tuple(goal_list)


def expand_fringe(vertex, fringe):
    new_nodes = list()
    vertex_list = list()
    for c in range(len(vertex)):
        vertex_list.append(vertex[c])
    for x in range(dimension):
        if vertex_list[x] + 1 <= size:
            vertex_prototype = vertex_list.copy()
            vertex_prototype[x] += 1
            new_v = tuple(vertex_prototype)
            fringe.append(new_v)
            new_nodes.append(new_v)
        if vertex_list[x] - 1 >= 0:
            vertex_prototype = vertex_list.copy()
            vertex_prototype[x] -= 1
            new_v = tuple(vertex_prototype)
            fringe.append(new_v)
            new_nodes.append(new_v)
    return new_nodes

def get_neighbors(vertex):
    x_max = vertex[0] + 1
    x_min = vertex[0] - 1
    y_max = vertex[1] + 1
    y_min = vertex[1] - 1

    successors = [(x_max, y_max), (x_max, y_min), (x_min, y_max), (x_min, y_min)]
    neighbors = []
    for n in neighbors :
        if n[0] > 0 and n[0] < 99 and n[1] > 0 and n[1] < 99 :
            neighbors.append(n)
    return neighbors


# uninformed searches:
def uninformed_search(starting, goal, dfs=None,):
    fringe = [starting]
    already_visited = set()
    while len(fringe) > 0:
        if dfs:
            vertex = fringe.pop()
        else:
            vertex = fringe.pop(0)
        if vertex not in already_visited:
            already_visited.add(vertex)
            if vertex == goal:
                return vertex
            else:
                expand_fringe(vertex, fringe)


print('Starting Node: ' + str(start_tup))
print('Goal Node: ' + str(goal_tup))

start_t = time.time()
end = uninformed_search(start_tup, goal_tup)
end_t = time.time()
if end != goal_tup:
    print('BFS found the wrong goal')
else:
    print('Uninformed BFS Search took: ' + str(end_t - start_t) + ' seconds')

start_t = time.time()
end = uninformed_search(start_tup, goal_tup, dfs=True)
end_t = time.time()
if end != goal_tup:
    print('DFS found the wrong goal')
else:
    print('Uninformed DFS Search took: ' + str(end_t - start_t) + ' seconds')


# for R2
# finds euclidean distance from vertex to goal
def euclidean_distance(vertex):
    if len(vertex) != len(goal_tup):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(vertex)):
            distance += (goal_tup[x] - vertex[x]) ** 2
    return math.sqrt(distance)

# for R2
# finds euclidean distance from vertex to vertex2
def euclidean_distance_2(vertex, vertex2):
    if len(vertex) != len(vertex2):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(vertex)):
            distance += (vertex2[x] - vertex[x]) ** 2
    return math.sqrt(distance)

# finds manhattan distance from vertex to goal
# sum of absolute values of differences in the goal’s x and y coordinates
# and the current cell’s x and y coordinates respectively
def manhattan_distance(vertex):
    if len(vertex) != len(goal_tup):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(vertex)):
            distance += abs(goal_tup[x] - vertex[x])
    return distance

# finds manhattan distance from vertex to vertex2
# sum of absolute values of differences in the goal’s x and y coordinates
# and the current cell’s x and y coordinates respectively
def manhattan_distance_2(vertex, vertex2):
    if len(vertex) != len(vertex2):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(vertex)):
            distance += abs(vertex2[x] - vertex[x])
    return distance

# informed Searches

def best_first_search(starting, goal):
    # time complexity is O(n)?

    # create empty lists for open and closed nodes
    open = []
    closed = []

    # add start node
    open.append(starting)

    #loop until open list is empty
    while len(open) > 0:

        # sort open list to get the node with the lowest cost first
        open.sort(key=euclidean_distance)

        # lst = len(open)
        # for i in range(0,lst):
        #     for j in range(0,lst-i-1):
        #         if(euclidean_distance(open[j])>euclidean_distance(open[j+1])):
        #             temp=open[j]
        #             open[j]=open[j+1]
        #             open[j+1]=temp


        # get node with lowest cost
        current_node = open.pop(0)

        # add current_node to closed list
        closed.append(current_node)

        # check if current_node is goal, then return current_node
        if current_node == goal:
            return current_node

        # get current node x and y from tuple
        x = current_node[0]
        y = current_node[1]

        # get neighbors
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

        # loop neighbors
        for next in neighbors:
            # checking for a wall or obstacle would go here

            # check if next is in closed list
            if(next in closed):
                continue

            # check if next is in open list and if it has a lower dist
            if(add_to_open(open,next) == True):
                open.append(next)

    # Return None, no path is found
    return None

def add_to_open(open, next):
    for vector in open:
        if(next == vector and euclidean_distance(next)>=euclidean_distance(vector)):
            return False
    return True

start_t = time.time()
end = best_first_search(start_tup, goal_tup)
end_t = time.time()
if end != goal_tup:
    print('Best-First Search found the wrong goal')
else:
    print('Informed Best-First Search Search took: ' + str(end_t - start_t) + ' seconds')

def a_star(starting, goal):
    fringe = [starting]

    already_visited = set()
    while len(fringe) > 0:
        vertex = fringe.pop(0)
        if vertex not in already_visited:
            already_visited.add(vertex)
            if vertex == goal:
                return vertex, len(already_visited)
            else:
                new = expand_fringe(vertex, fringe)
                fringe = fringe + new
                fringe.sort(key=euclidean_distance)


start_t = time.time()
end = a_star(start_tup, goal_tup)
end_t = time.time()
if end[0] != goal_tup:
    print('A* found the wrong goal')
else:
    print('Informed A* Search took: ' + str(end_t - start_t) + ' seconds')
    print('Path Length:', end[1])


#
def dkistras(source,graph):
    vertices=len(graph[0])
    def minimum_distance(distance,vert_set):
        min_val= sys.maxsize
        min_index=None
        for v in range(vertices):
            if distance[v] < min_val and vert_set[v]==False:
                min_val= distance[v]
                min_index=v
        return min_index
    distance= [sys.maxsize]* vertices
    distance[source]=0
    vert_set= [False]*vertices
    for i in range(vertices):
        min_dist= minimum_distance(distance,vert_set)
        vert_set[min_dist]=True
        for v in range(vertices):
            if graph[min_dist][v] > 0 and vert_set[v]==False and distance[v]>distance[min_dist] + graph[min_dist][v]:
                distance[v]= distance[min_dist] +graph[min_dist][v]
    
    return distance



def print_dk(distance):
    print("Dkikstra's Results-")
    for node in range(len(distance)):
        print("Distance from source to node: ",node,": ",distance[node])


start_t = time.time()
dk_result = dkistras(0,graph)
end_t = time.time()

print("Djikstra's algorithm took ",str(end_t - start_t),"seconds")
print_dk(dk_result)

def kruskal(graph): 
    def find(i): 
        while parent[i] != i: 
           i = parent[i] 
        return i 
  
    # Does union of i and j. It returns
    # false if i and j are already in same
    # set.
    def union(i, j): 
        a = find(i) 
        b = find(j) 
        parent[a] = b 
    vertices= len(graph[0])
    parent = [i for i in range(vertices)]
    mincst = 0 # Cost of min MST 
  
    # Initialize sets of disjoint sets 
    for i in range(vertices): 
        parent[i] = i 
  
    # Include minimum weight edges one by one  
    edge_count = 0
    while edge_count < vertices - 1: 
        min = sys.maxsize 
        a = -1
        b = -1
        for i in range(vertices): 
            for j in range(vertices): 
                if find(i) != find(j) and graph[i][j] < min and graph[i][j] != 0: 
                    min = graph[i][j] 
                    a = i 
                    b = j 
        union(a, b) 
        print('Edge {}:({}, {}) cost:{}'.format(edge_count, a, b, min)) 
        edge_count += 1
        mincst += min
  
    print("Minimum cost= {}".format(mincst)) 

start_t = time.time()
kruskal(graph)
end_t = time.time()

print("Kruskal's algorithm took ",str(end_t - start_t),"seconds")


#Very Slow
def id_a_star(starting):
    path = []
    #Stack use append() and pop()
    path.append(starting)
    # Value that will cut off the search
    cutoff = euclidean_distance(starting)
    
    while True :
        ret = id_a_star_helper(path, 0, cutoff)
        if ret == "Found" :
            return (path, cutoff)
        if ret == math.inf :
            return None
        cutoff = ret

def id_a_star_helper(path, total_cost, cutoff):
    #Gets the last element added to path
    node = path[-1]
    #print(node)
    f = total_cost + euclidean_distance(node)
    if f > cutoff :
        return f
    if node == goal_tup :
        return "Found"
    min = math.inf
    # get successors
    s = [(node[0]-1, node[1]), (node[0]+1, node[1]), (node[0], node[1]-1), (node[0], node[1]+1)]
    successors = []
    for succ in s :
        if not (succ[0] < 0 or succ[0] > 99 or succ[1] < 0 or succ[1] > 99) :
            successors.append(succ)
        s.pop()
    for succ in successors :
        if succ not in path :
            path.append(succ) 
            ret = id_a_star_helper(path, total_cost + 1, cutoff)
            if ret == "Found" :
                return "Found"
            if ret < min :
                min = ret
            path.pop()
    
    return min

start_t = time.time()
var = id_a_star(start_tup)
end_t = time.time()
print("IDA* Completed: Input = ", start_tup, "Output =", str(var), "Time =", str(end_t - start_t))


def bidirectional_search(starting):
    fringeFromStart = [starting]
    fringeFromGoal = [goal_tup]
    

def beam_search(starting, goal, beam_width):
    discovered = []
    fringe = [starting]
    while len(fringe) > 0 :
        if fringe[0] == goal :
            discovered.append(fringe[0])
            return (fringe[0], len(discovered))
        else :
            if fringe[0] not in discovered :
                discovered.append(fringe[0])
                neighbors = expand_fringe(fringe[0], fringe)
                fringe = fringe + neighbors
                neighbors.sort(key=euclidean_distance)
                if len(neighbors) > beam_width :
                    i = -1
                    while abs(i) < (4 - beam_width):
                        fringe.remove(neighbors[i])
                        i -= 1
                fringe.pop(0)
                fringe.sort(key=euclidean_distance)
            else :
                fringe.pop(0)

    return None

#Beam should be 1 to 4
start_t = time.time()
beam_width = 4
var = beam_search(start_tup, goal_tup, beam_width)
end_t = time.time()
print("Beam Search completed, Output=", str(var), "Time =", str(end_t - start_t), "Beam Width =", str(beam_width))

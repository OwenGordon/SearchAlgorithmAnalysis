# These are the algorithms that can search a random board:
# uninformed_search
# best_first_search
# a_star
# beam_search

import random
import math
import time
import matplotlib.pyplot as plt
import noise
import sys
import heapq

size = 100
dimension = 2
weight_factor = 10
obstacles = 10

weight_matrix = [[0 for x in range(size)] for y in range(size)]

for x in range(size):
    for y in range(size):
        weight_matrix[x][y] = abs(noise.snoise2(x * 0.03, y * 0.03)) * weight_factor

plt.imshow(weight_matrix)
plt.colorbar()

start_list = list()
goal_list = list()

for i in range(dimension):
    start = random.randint(0, int(size / 2 - 1))
    goal = random.randint(int(size / 2), size - 1)
    while goal == start:
        goal = random.randint(int(size/2), size - 1)
    start_list.append(start)
    goal_list.append(goal)

start_tup = tuple(start_list)
goal_tup = tuple(goal_list)

obstacles_list = list()

for i in range(obstacles):
    obstacle = random.randint(0, size - 1), random.randint(0, size - 1)
    while obstacle in obstacles_list or obstacle == start_tup or obstacle == goal_tup:
        obstacle = random.randint(0, size - 1), random.randint(0, size - 1)
    weight_matrix[obstacle[0]][obstacle[1]] = 0
    obstacles_list.append(obstacle)

obstacles_x = list()
obstacles_y = list()
for i in range(len(obstacles_list)):
    obstacles_x.append(obstacles_list[i][0])
    obstacles_y.append(obstacles_list[i][1])

plt.axis('off')
plt.xlim(-1, size)
plt.ylim(-1, size)

plt.plot(start_tup[0], start_tup[1], 'go', label='Start')
plt.plot(goal_tup[0], goal_tup[1], 'bo', label='Goal')
plt.plot(obstacles_x, obstacles_y, 'r.', label='Obstacles')

def graph_predetermined(current):
    plt.plot(start_tup[0], start_tup[1], 'o', label='Start')
    plt.plot(goal_tup[0], goal_tup[1], 'o', label='Goal')
    plt.plot(obstacles_x, obstacles_y, 'o', label='Obstacles')
    plt.plot(current[0], current[1], 'o', label='failure point')
    plt.legend(loc='lower center')
    plt.show()


def cost(current):
    return weight_matrix[current[1]][current[0]]


class Vertex:
    def __init__(self, previous, current, p_cost):
        self.previous = previous
        self.current = current
        self.cost = p_cost + cost(current)

    def __lt__(self, other):
        return self.cost < other.cost


def expand_fringe(vertex):
    new_nodes = set()
    c_cost = vertex.cost
    if vertex.current[0] + 1 < size:
        if cost((vertex.current[0] + 1, vertex.current[1])) == 0:
            pass
        else:
            new_nodes.add(Vertex(vertex, (vertex.current[0] + 1, vertex.current[1]), c_cost))
    if vertex.current[1] + 1 < size:
        if cost((vertex.current[0], vertex.current[1] + 1)) == 0:
            pass
        else:
            new_nodes.add(Vertex(vertex, (vertex.current[0], vertex.current[1] + 1), c_cost))
    if vertex.current[0] - 1 >= 0:
        if cost((vertex.current[0] - 1, vertex.current[1])) == 0:
            pass
        else:
            new_nodes.add(Vertex(vertex, (vertex.current[0] - 1, vertex.current[1]), c_cost))
    if vertex.current[1] - 1 >= 0:
        if cost((vertex.current[0], vertex.current[1] - 1)) == 0:
            pass
        else:
            new_nodes.add(Vertex(vertex, (vertex.current[0], vertex.current[1] - 1), c_cost))

    return new_nodes

def euclidean_distance(current):
    if len(current) != len(goal_tup):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(current)):
            distance += (goal_tup[x] - current[x]) ** 2
    return math.sqrt(distance)

def euclidean_distance_between(current, other):
    if len(current) != len(other):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(current)):
            distance += (other[x] - current[x]) ** 2
    return math.sqrt(distance)

def manhattan_distance(vertex):
    if len(vertex) != len(goal_tup):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(vertex)):
            distance += abs(goal_tup[x] - vertex[x])
    return distance

def uninformed_search(starting, goal, dfs=None):
    fringe = [starting]
    already_visited = set()
    while len(fringe) > 0:
        if dfs:
            vertex = fringe.pop()
            current = vertex.current
        else:
            vertex = fringe.pop(0)
            current = vertex.current
        if current not in already_visited:
            already_visited.add(current)
            if current == goal:
                return vertex, len(already_visited)
            else:
                new = expand_fringe(vertex)
                new_nodes = list(new)
                fringe += new_nodes

def IDDFS(starting, goal):
    for depth in range(sys.maxsize):
        found, remaining = DLS(starting, goal, depth)
        if found is not None:
            return found, depth
        elif not remaining:
            return None

def DLS(vertex, goal, depth):
    if depth == 0:
        if vertex.current == goal:
            return vertex, True
        else:
            return None, True
    elif depth > 0:
        any_remaining = False
        new = expand_fringe(vertex)
        for n in new:
            found, remaining = DLS(n, goal, depth - 1)
            if found is not None:
                return found, True
            if remaining:
                any_remaining = True
        return None, any_remaining

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
            return current_node, len(closed)

        # get current node x and y from tuple
        # x = current_node[0]
        # y = current_node[1]

        # get neighbors
        open += expand_fringe(current_node)

        # loop neighbors
        # for next in neighbors:
        #     # checking for a wall or obstacle would go here
        #
        #     # check if next is in closed list
        #     if(next in closed):
        #         continue
        #
        #     # check if next is in open list and if it has a lower dist
        #     if(add_to_open(open,next) == True):
        #         open.append(next)

    # Return None, no path is found
    return None

def greedy_search(starting, goal):
    distances = list()
    heapq.heappush(distances, (greedy_heuristic(starting), starting))
    already_visited = set()
    while len(distances) > 0:
        vertex_d, vertex = heapq.heappop(distances)
        current = vertex.current
        if current not in already_visited:
            already_visited.add(current)
            if current == goal:
                return vertex, len(already_visited)
            else:
                new = expand_fringe(vertex)
                for v in new:
                    heapq.heappush(distances, (greedy_heuristic(v), v))

    return vertex

def UCS(starting, goal):
    distances = list()
    heapq.heappush(distances, (UCS_heuristic(starting), starting))
    already_visited = set()
    while len(distances) > 0:
        vertex_d, vertex = heapq.heappop(distances)
        current = vertex.current
        if current not in already_visited:
            already_visited.add(current)
            if current == goal:
                return vertex, len(already_visited)
            else:
                new = expand_fringe(vertex)
                for v in new:
                    heapq.heappush(distances, (UCS_heuristic(v), v))

    return vertex

def heuristic(vertex):
    return vertex.cost + euclidean_distance(vertex.current)

def greedy_heuristic(vertex):
    return euclidean_distance(vertex.current)

def UCS_heuristic(vertex):
    return vertex.cost

def bidirectional_heuristic(vertex, goal):
    return vertex.cost + euclidean_distance_between(vertex.current, goal)

def bidirectional_search(start_vertex, goal_vertex):
    start_heuristics = list()
    heapq.heappush(start_heuristics, (bidirectional_heuristic(start_vertex, goal_vertex.current), start_vertex))

    goal_heuristics = list()
    heapq.heappush(goal_heuristics, (bidirectional_heuristic(goal_vertex, start_vertex.current), goal_vertex))

    start_already_visited_set = set()
    start_already_visited_dict = dict()
    goal_already_visited_set = set()
    goal_already_visited_dict = dict()

    val, start_side = heapq.heappop(start_heuristics)
    val, goal_side = heapq.heappop(goal_heuristics)
    while start_already_visited_set.isdisjoint(goal_already_visited_set):
        if start_side.current in goal_already_visited_set:
            start_already_visited_set.add(start_side.current)
            start_already_visited_dict[start_side.current] = start_side
            break
        elif start_side.current not in start_already_visited_set:
            start_already_visited_set.add(start_side.current)
            start_already_visited_dict[start_side.current] = start_side
            new = expand_fringe(start_side)
            for v in new:
                heapq.heappush(start_heuristics, (bidirectional_heuristic(v, goal_vertex.current), v))

        if goal_side.current in start_already_visited_set:
            goal_already_visited_set.add(goal_side.current)
            goal_already_visited_dict[goal_side.current] = goal_side
            break
        elif goal_side.current not in goal_already_visited_set:
            goal_already_visited_set.add(goal_side.current)
            goal_already_visited_dict[goal_side.current] = goal_side
            new = expand_fringe(goal_side)
            for v in new:
                heapq.heappush(goal_heuristics, (bidirectional_heuristic(v, start_vertex.current), v))

        val, start_side = heapq.heappop(start_heuristics)
        val, goal_side = heapq.heappop(goal_heuristics)

    intersection = start_already_visited_set.intersection(goal_already_visited_set)
    i = intersection.pop()

    start_end = start_already_visited_dict[i]
    goal_end = goal_already_visited_dict[i]

    start_path = [start_end]
    while start_end.previous:
        start_path.append(start_end.previous)
        start_end = start_end.previous

    goal_path = [goal_end]
    while goal_end.previous:
        goal_path.append(goal_end.previous)
        goal_end = goal_end.previous

    start_path.reverse()
    vertex = start_path.pop(0)
    for node in start_path:
        prev = vertex
        vertex = Vertex(vertex, (node.current[0], node.current[1]), prev.cost)

    for n in range(1, len(goal_path)):
        node = goal_path[n]
        prev = vertex
        vertex = Vertex(vertex, (node.current[0], node.current[1]), prev.cost)

    return vertex, len(goal_already_visited_set.union(start_already_visited_set))


def ida_star(starting, goal):
    bound = greedy_heuristic(starting)
    found = False
    discovered = [starting]
    while not found:
        t, vertex = ida_star_search(starting, goal, discovered, bound)
        if isinstance(t, bool) and t:
            return vertex, bound
        if t == math.inf:
            return None
        bound = t


def ida_star_search(vertex, goal, discovered, bound):
    f = greedy_heuristic(vertex)
    if f > bound:
        return f, None
    if vertex.current == goal:
        return True, vertex
    min = math.inf
    new = expand_fringe(vertex)
    for n in new:
        if n not in discovered:
            discovered.append(n)
            t, solving_n = ida_star_search(n, goal, discovered, bound)
            if isinstance(t, bool) and t:
                return True, solving_n
            if t < min:
                min = t
            discovered.remove(n)
    return min, None


def a_star(starting, goal):
    heuristics = list()
    heapq.heappush(heuristics, (heuristic(starting), starting))

    already_visited = dict()
    while len(heuristics) > 0:
        vertex_h, vertex = heapq.heappop(heuristics)
        current = vertex.current
        if current not in already_visited:
            already_visited[current] = vertex.cost
            if current == goal:
                return vertex, len(already_visited)
            else:
                new = expand_fringe(vertex)
                for v in new:
                    heapq.heappush(heuristics, (heuristic(v), v))
        else:
            if vertex.cost < already_visited[current]:
                already_visited[current] = vertex.cost
                if current == goal:
                    return vertex, len(already_visited)
                else:
                    new = expand_fringe(vertex)
                    for v in new:
                        heapq.heappush(heuristics, (heuristic(v), v))
    return vertex

def beam_search(starting, goal, beam_width):
    heuristics = list()
    heapq.heappush(heuristics, (heuristic(starting), starting))

    discovered = []
    while len(heuristics) > 0:
        vertex_h, vertex = heapq.heappop(heuristics)
        current = vertex.current
        if current == goal:
            discovered.append(vertex)
            return vertex, len(discovered)
        else:
            if current not in discovered:
                discovered.append(current)
                n = expand_fringe(vertex)
                neighbors = list(n)
                neighbors.sort(key=heuristic)
                if len(neighbors) > beam_width:
                    b = -1
                    while abs(b) < (4 - beam_width):
                        neighbors.pop()
                        b -= 1
                for v in neighbors:
                    heapq.heappush(heuristics, (heuristic(v), v))

    return starting, 0

def get_neighbors(vertex):
    new_nodes = list()
    if vertex[0] + 1 < size:
        if cost((vertex[0] + 1, vertex[1])) == 0:
            pass
        else:
            new_nodes.append((vertex[0] + 1, vertex[1]))
    if vertex[1] + 1 < size:
        if cost((vertex[0], vertex[1] + 1)) == 0:
            pass
        else:
            new_nodes.append((vertex[0], vertex[1] + 1))
    if vertex[0] - 1 >= 0:
        if cost((vertex[0] - 1, vertex[1])) == 0:
            pass
        else:
            new_nodes.append((vertex[0] - 1, vertex[1]))
    if vertex[1] - 1 >= 0:
        if cost((vertex[0], vertex[1] - 1)) == 0:
            pass
        else:
            new_nodes.append((vertex[0], vertex[1] - 1))

    return new_nodes

def dkistras(source, graph):
    dist = dict()
    dist[source] = cost(source)

    vertices = list()

    heapq.heappush(vertices, (dist[source], source))
    prev = dict()
    prev[source] = None

    for x in range(len(graph)):
        for y in range(len(graph[x])):
            v = (x, y)
            if v != source:
                dist[v] = math.inf
                prev[v] = None
            heapq.heappush(vertices, (dist[v], v))

    while len(vertices) > 0:
        v_dist, v = heapq.heappop(vertices)
        if v == goal_tup:
            return v, dist, prev
        else:
            neighbors = get_neighbors(v)
            for n in neighbors:
                alt = dist[v] + cost(n)
                if alt < dist[n]:
                    dist[n] = alt
                    prev[n] = v
                    heapq.heappush(vertices, (dist[n], n))


def test_random_search(start, goal):
    print('Start:', start)
    print('End:', goal)

    starting_vertex = Vertex(None, start, 0)
    goal_vertex = Vertex(None, goal, 0)


    start_t = time.time()
    end, length = uninformed_search(starting_vertex, goal)
    end_t = time.time()
    cost = end.cost
    if end.current != goal:
        print('Uninformed BFS found the wrong goal')
    else:
        print('Uninformed BFS took: ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(cost) + ' after exploring ' + str(length) + ' nodes')

    path = [end.current]
    while (end.previous):
        path.append(end.previous.current)
        end = end.previous

    path.reverse()
    # print(path)
    path_x = list()
    path_y = list()
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    # plt.plot(path_x, path_y, '.', label='BFS, cost = ' + str(cost))

    start_t = time.time()
    end, length = uninformed_search(starting_vertex, goal, dfs=True)
    end_t = time.time()
    cost = end.cost
    if end.current != goal:
        print('Uninformed DFS found the wrong goal')
    else:
        print('Uninformed DFS took: ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(cost) + ' after exploring ' + str(length) + ' nodes')

    path = [end.current]
    while end.previous:
        path.append(end.previous.current)
        end = end.previous

    path.reverse()
    # print(path)
    path_x = list()
    path_y = list()
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    # plt.plot(path_x, path_y, '.', label='DFS, cost = ' + str(cost))

    # start_t = time.time()
    # end, depth = IDDFS(starting_vertex, goal)
    # end_t = time.time()
    # cost = end.cost
    # if end.current != goal:
    #     print('Uninformed IDDFS found the wrong goal')
    # else:
    #     print('Uninformed IDDFS took: ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(cost) + ' at depth ' + str(depth))


    #
    # start_t = time.time()
    # end = best_first_search(start, goal)
    # end_t = time.time()
    # if end[0] != goal:
    #     print('Best-First Search found the wrong goal')
    # else:
    #     print('Informed Best-First Search Search took: ' + str(end_t - start_t) + ' seconds, and explored ' + str(end[1]) + ' nodes')


    start_t = time.time()
    end, length = greedy_search(starting_vertex, goal)
    end_t = time.time()
    cost = end.cost
    if end.current != goal:
        print('Informed greedy search found the wrong goal')
    else:
        print('Informed greedy search took ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(cost) + ' after exploring ' + str(length) + ' nodes')

    path = [end.current]
    while (end.previous):
        path.append(end.previous.current)
        end = end.previous

    path.reverse()
    # print(path)
    path_x = list()
    path_y = list()
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    plt.plot(path_x, path_y, 'w:', label='Greedy, cost = ' + str(cost))

    start_t = time.time()
    end, length = UCS(starting_vertex, goal)
    end_t = time.time()
    cost = end.cost
    if end.current != goal:
        print('Informed UCS found the wrong goal')
    else:
        print('Informed UCS took ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(
            cost) + ' after exploring ' + str(length) + ' nodes')

    path = [end.current]
    while (end.previous):
        path.append(end.previous.current)
        end = end.previous

    path.reverse()
    # print(path)
    path_x = list()
    path_y = list()
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    plt.plot(path_x, path_y, 'w--', label='UCS, cost = ' + str(cost))

    start_t = time.time()
    end, length = a_star(starting_vertex, goal)
    end_t = time.time()
    cost = end.cost
    if end.current != goal:
        print('Informed A* search found the wrong goal')
    else:
        print('Informed A* Search took ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(cost) + ' after exploring ' + str(length) + ' nodes')

    path = [end.current]
    while end.previous:
        path.append(end.previous.current)
        # print(end.cost)
        end = end.previous

    path.reverse()
    # print(path)
    path_x = list()
    path_y = list()
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    plt.plot(path_x, path_y, '-', label='A*, cost = ' + str(cost))

    # start_t = time.time()
    # end, bound = ida_star(starting_vertex, goal)
    # end_t = time.time()
    # cost = end.cost
    # if end.current != goal:
    #     print('Informed IDA* search found the wrong goal')
    # else:
    #     print('Informed IDA* Search took ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(
    #         cost) + ' and finished with a bound of ' + str(bound))

    beam_length = 2
    start_t = time.time()
    end, length = beam_search(starting_vertex, goal, beam_length)
    end_t = time.time()
    cost = end.cost
    if end.current != goal:
        print('Beam Search found the wrong goal')
    else:
        print('Informed Beam Search with a beam length of ' + str(beam_length) + ' took: ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(cost) + ' after exploring ' + str(length) + ' nodes')

    path = [end.current]
    while end.previous:
        path.append(end.previous.current)
        # print(end.cost)
        end = end.previous

    path.reverse()
    path_x = list()
    path_y = list()
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    plt.plot(path_x, path_y, '-', label='Beam, cost = ' + str(cost))
    # plt.show()

    start_t = time.time()
    dk_vertex, dk_dist, dk_prev = dkistras(start_tup, weight_matrix)
    end_t = time.time()
    dk_vertex = goal_tup
    cost = dk_dist[dk_vertex]

    if dk_vertex != goal:
        print('Dijkstra found the wrong goal')
    else:
        print('Dijkstra took: ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(cost))

    path = [dk_vertex]
    vertex = dk_vertex
    while dk_prev[vertex]:
        path.append(dk_prev[vertex])
        vertex = dk_prev[vertex]

    path.reverse()
    # print(path)
    path_x = list()
    path_y = list()
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    plt.plot(path_x, path_y, 'k:', label='Dijkstra, cost = ' + str(cost))

    start_t = time.time()
    end, length = bidirectional_search(starting_vertex, goal_vertex)
    end_t = time.time()
    cost = end.cost
    if end.current != goal:
        print('Bidirectional Search found the wrong goal')
    else:
        print('Bidirectional Search took: ' + str(end_t - start_t) + ' seconds, and found a path cost of ' + str(cost) + ' after exploring ' + str(
            length) + ' nodes')

    path = [end.current]
    while (end.previous):
        path.append(end.previous.current)
        end = end.previous

    path.reverse()
    path_x = list()
    path_y = list()
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    plt.plot(path_x, path_y, 'k--',  label='Bidirectional, cost = ' + str(cost))


    # plt.legend()
    plt.show()


test_random_search(start_tup, goal_tup)
"""
Author: Owen Gordon, owgordon@iu.edu
This file contains each search algorithm
"""
import heapq
import sys
from testing_helpers import *


def uninformed_search(starting, goal, dfs=None):
    """
    Performs the uninformed search
    :param starting: x,y location of starting node
    :param goal: x,y location of goal node
    :param dfs: when true preforms DFS search, otherwise performs BFS
    :return: Tuple containing vertex object of goal node, length of nodes expanded
    """
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
    return vertex, 0


def IDDFS(starting, goal):
    """
    Performs IDDFS search
    :param starting: x,y of starting node
    :param goal: x,y of goal node
    :return: tuple containing goal node vertex, depth reached
    """
    for depth in range(sys.maxsize):
        found, remaining = DLS(starting, goal, depth)
        if found is not None:
            return found, depth
        elif not remaining:
            return starting, 0


def DLS(vertex, goal, depth):
    """
    IDDFS helper function, performs IDDFS until max depth is reached or goal is discovered
    :param vertex: current vertex object being explored
    :param goal: x,y of goal
    :param depth: the current depth
    :return: vertex when found
    """
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


def greedy_search(starting, goal, manhattan=False):
    """
    Performs greedy search
    :param starting: x,y of starting node
    :param goal: x,y of goal
    :param manhattan: when true uses manhattan distance heuristic, otherwise uses euclidean distance heuristic
    :return: Tuple containing vertex object of goal node, length of nodes expanded
    """
    distances = list()
    if manhattan:
        f = lambda x: manhattan_greedy_heuristic(x)
    else:
        f = lambda x: greedy_heuristic(x)
    heapq.heappush(distances, (f(starting), starting))
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
                    heapq.heappush(distances, (f(v), v))

    return vertex, 0


def UCS(starting, goal):
    """
    Performs UCS search
    :param starting: x,y of starting node
    :param goal: x,y of goal
    :return: Tuple containing vertex object of goal node, length of nodes expanded
    """
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

    return vertex, 0


def bidirectional_search(start_vertex, goal_vertex, manhattan=False):
    """
    Performs bidirectional search
    :param start_vertex: x,y of starting node
    :param goal_vertex: x,y of goal
    :param manhattan: when true uses manhattan distance heuristic, otherwise uses euclidean distance heuristic
    :return: Tuple containing vertex object of goal node, length of nodes expanded
    """
    if manhattan:
        f = lambda x, y: manhattan_bidirectional_heuristic(x, y)
    else:
        f = lambda x, y: bidirectional_heuristic(x, y)

    start_heuristics = list()
    heapq.heappush(start_heuristics, (f(start_vertex, goal_vertex.current), start_vertex))

    goal_heuristics = list()
    heapq.heappush(goal_heuristics, (f(goal_vertex, start_vertex.current), goal_vertex))

    start_already_visited_set = set()
    start_already_visited_dict = dict()
    goal_already_visited_set = set()
    goal_already_visited_dict = dict()

    while start_already_visited_set.isdisjoint(goal_already_visited_set):
        if len(start_heuristics) <= 0 and len(goal_heuristics) <= 0:
            return start_vertex, 0

        if len(start_heuristics) > 0:
            val, start_side = heapq.heappop(start_heuristics)
        if len(goal_heuristics) > 0:
            val, goal_side = heapq.heappop(goal_heuristics)

        if start_side.current in goal_already_visited_set:
            start_already_visited_set.add(start_side.current)
            start_already_visited_dict[start_side.current] = start_side
            break
        elif start_side.current not in start_already_visited_set:
            start_already_visited_set.add(start_side.current)
            start_already_visited_dict[start_side.current] = start_side
            new = expand_fringe(start_side)
            for v in new:
                heapq.heappush(start_heuristics, (f(v, goal_vertex.current), v))

        if goal_side.current in start_already_visited_set:
            goal_already_visited_set.add(goal_side.current)
            goal_already_visited_dict[goal_side.current] = goal_side
            break
        elif goal_side.current not in goal_already_visited_set:
            goal_already_visited_set.add(goal_side.current)
            goal_already_visited_dict[goal_side.current] = goal_side
            new = expand_fringe(goal_side)
            for v in new:
                heapq.heappush(goal_heuristics, (f(v, start_vertex.current), v))


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


def ida_star(starting, goal, manhattan=False):
    """
    Performs IDA* search
    :param starting: x,y of starting node
    :param goal: x,y of goal
    :param manhattan: when true uses manhattan distance heuristic, otherwise uses euclidean distance heuristic
    :return: Tuple containing vertex object of goal node, bound reached
    """
    if manhattan:
        f = lambda x: manhattan_heuristic(x)
    else:
        f = lambda x: heuristic(x)

    bound = f(starting)
    found = False
    discovered = [starting]
    while not found:
        t, vertex = ida_star_search(starting, goal, discovered, bound, f)
        if isinstance(t, bool) and t:
            return vertex, bound
        if t == math.inf:
            return None
        bound = t


def ida_star_search(vertex, goal, discovered, bound, heuristic_function):
    """
    helper function for IDA* search, performs IDA* search until bound is hit or goal is discovered
    :param vertex: current vertex being explored
    :param goal: x,y of goal node
    :param discovered: list of already explored nodes
    :param bound: the maximum heuristic value being looked at in this iteration
    :param heuristic_function: type of heuristic function being used, manhattan or euclidean
    :return: true hwne goal is found, and vertex object of goal
    """
    f = heuristic_function(vertex)
    if f > bound:
        return f, None
    if vertex.current == goal:
        return True, vertex
    min = math.inf
    new = expand_fringe(vertex)
    for n in new:
        if n not in discovered:
            discovered.append(n)
            t, solving_n = ida_star_search(n, goal, discovered, bound, heuristic_function)
            if isinstance(t, bool) and t:
                return True, solving_n
            if t < min:
                min = t
            discovered.remove(n)
    return min, None


def a_star(starting, goal, manhattan=False):
    """
    Performs A* search
    :param starting: x,y of starting node
    :param goal: x,y of goal
    :param manhattan: when true uses manhattan distance heuristic, otherwise uses euclidean distance heuristic
    :return: Tuple containing vertex object of goal node, length of nodes expanded
    """
    heuristics = list()
    if manhattan:
        f = lambda x: manhattan_heuristic(x)
    else:
        f = lambda x: heuristic(x)

    heapq.heappush(heuristics, (f(starting), starting))

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
                    heapq.heappush(heuristics, (f(v), v))
        else:
            if vertex.cost < already_visited[current]:
                already_visited[current] = vertex.cost
                if current == goal:
                    return vertex, len(already_visited)
                else:
                    new = expand_fringe(vertex)
                    for v in new:
                        heapq.heappush(heuristics, (f(v), v))
    return vertex, 0


def beam_search(starting, goal, beam_width, manhattan=False):
    """
    Performs beam search
    :param starting: x,y of starting node
    :param goal: x,y of goal
    :param beam_width: the length of the beam used, value from 1-4
    :param manhattan: when true uses manhattan distance heuristic, otherwise uses euclidean distance heuristic
    :return: Tuple containing vertex object of goal node, length of nodes expanded
    """
    heuristics = list()

    if manhattan:
        f = lambda x: manhattan_heuristic(x)
    else:
        f = lambda x: heuristic(x)

    heapq.heappush(heuristics, (f(starting), starting))

    discovered = dict()
    thrown_away = list()
    while len(heuristics) > 0:
        vertex_h, vertex = heapq.heappop(heuristics)
        current = vertex.current
        if current == goal:
            discovered[current] = vertex.cost
            return vertex, len(discovered)
        else:
            if current not in discovered:
                discovered[current] = vertex.cost
                n = expand_fringe(vertex)
                neighbors = list(n)
                discard = list()
                for i in range(len(neighbors)):
                    if neighbors[i].current in discovered:
                        discard.append(neighbors[i])
                for d in discard:
                    neighbors.remove(d)
                neighbors.sort(key=f)
                if len(neighbors) > beam_width:
                    b = -1
                    while abs(b) < (4 - beam_width):
                        x = neighbors.pop()
                        heapq.heappush(thrown_away, (f(x), x))
                        b -= 1
                for v in neighbors:
                    heapq.heappush(heuristics, (f(v), v))
            else:
                if vertex.cost < discovered[current]:
                    discovered[current] = vertex.cost
                    n = expand_fringe(vertex)
                    neighbors = list(n)
                    neighbors.sort(key=f)
                    if len(neighbors) > beam_width:
                        b = -1
                        while abs(b) < (4 - beam_width):
                            x = neighbors.pop()
                            heapq.heappush(thrown_away, (f(x), x))
                            b -= 1
                    for v in neighbors:
                        heapq.heappush(heuristics, (f(v), v))
        if len(heuristics) < 1 and len(thrown_away) > 0:
            h, v = heapq.heappop(thrown_away)
            heapq.heappush(heuristics, (f(v), v))

    return vertex, 0


def dijkstras(source, goal, graph, end_at_found=True):
    """
    performs Dijkstra's algorithm
    :param source: x,y for source node
    :param goal: x,y for goal node
    :param graph: graph being search
    :param end_at_found: if true the search will stop when goal is reached, otherwise entire board searched
    :return: Tuple containing vertex object of goal node, length of nodes expanded
    """
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
        if v == goal and end_at_found:
            return v, dist, prev
        else:
            neighbors = get_neighbors(v)
            for n in neighbors:
                alt = dist[v] + cost(n)
                if alt < dist[n]:
                    dist[n] = alt
                    prev[n] = v
                    heapq.heappush(vertices, (dist[n], n))

    return goal, dist, prev
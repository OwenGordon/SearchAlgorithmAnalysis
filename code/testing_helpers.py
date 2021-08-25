"""
Author: Owen Gordon, owgordon@iu.edu
Helper functions for testing algorithms
"""
import math
from testing_initialization import _initialize, get_size
import matplotlib.pyplot as plt

weight_matrix, obstacles_list, source, goal = None, None, None, None

graph_size = get_size()


def initialize(size, weight_factor, obstacles_percentage, no_weight=False, beam_size=None, num_runs=None, visualize=False, preset=0):
    """
    initializes the board by calling _initialize function inside testing_initializization
    :param size: size of board
    :param weight_factor: maximum weight value in board
    :param obstacles_percentage: percentage of board as obstacles
    :param no_weight: if true, then uniform weight of board
    :param beam_size: if true sets beam size
    :param num_runs: if true executes multiple times
    :param visualize: if true, visualizes board
    :param preset: if greater than 0, initializes board according to a predefined option
    :return: none
    """
    global weight_matrix, obstacles_list, source, goal, graph_size
    weight_matrix, obstacles_list, source, goal = _initialize(size, weight_factor, obstacles_percentage, no_weight=no_weight, beam_size_=beam_size, num_runs_=num_runs, visualize_=visualize, preset=preset)
    graph_size = get_size()


def get_weight_matrix():
    """
    gets the weight matrix
    :return: weight matrix
    """
    return weight_matrix


def get_obstacles_list():
    """
    gets obstacle list
    :return: obstacle list
    """
    return obstacles_list


def get_source():
    """
    gets source node
    :return: source node
    """
    return source


def get_goal():
    """
    gets goal node
    :return: goal node
    """
    return goal


class Vertex:
    """
    Vertex Class
    Creates a vertex that is explored
    """
    def __init__(self, previous, current, p_cost):
        """
        Initializes vertex object
        :param previous: x,y position that was expanded to create this vertex
        :param current: current x,y position of vertex
        :param p_cost: previous cost value
        """
        self.previous = previous
        self.current = current
        self.cost = p_cost + cost(current)

    def __lt__(self, other):
        """
        compares vertex objects
        :param other: other vertex to compare
        :return: true if current vertex cost is less than other, else false
        """
        return self.cost < other.cost


def cost(current):
    """
    calculates cost of specific node
    :param current: x,y for node to find cost
    :return: cost
    """
    return weight_matrix[current[1]][current[0]]


def euclidean_distance(current):
    """
    calculates eulidean distance from current to goal
    :param current: x,y of node
    :return: euclidean distance between two nodes
    """
    if len(current) != len(goal):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(current)):
            distance += (goal[x] - current[x]) ** 2
    return math.sqrt(distance)


def euclidean_distance_between(current, other):
    """
    calculates eulidean distance from current to other
    :param current: x,y of current
    :param other: x,y of other
    :return: euclidean distance between two nodes
    """
    if len(current) != len(other):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(current)):
            distance += (other[x] - current[x]) ** 2
    return math.sqrt(distance)


def manhattan_distance(current):
    """
    calculates manhattan distance from current to goal
    :param current: x,y of current
    :return: manhattan distance between two points
    """
    if len(current) != len(goal):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(current)):
            distance += abs(goal[x] - current[x])
    return distance


def manhattan_distance_between(current, other):
    """
    calculates manhattan distance from current to other
    :param current: x,y of current node
    :param other: x,y of other node
    :return: manhattan distance between two points
    """
    if len(current) != len(other):
        raise AttributeError('Dimension of vectors must be the same')
    else:
        distance = 0
        for x in range(len(current)):
            distance += abs(other[x] - current[x])
    return distance


def heuristic(vertex):
    """
    calculates heuristic value for vertex, uses euclidean distance
    :param vertex: current vertex object
    :return: heuristic value
    """
    return vertex.cost + euclidean_distance(vertex.current)


def greedy_heuristic(vertex):
    """
    heuristic function for greedy search, uses euclidean distance
    :param vertex: current vertex object
    :return: heuristic value
    """
    return euclidean_distance(vertex.current)


def UCS_heuristic(vertex):
    """
    heuristic function for UCS
    :param vertex: current vertex object
    :return: heuristic value
    """
    return vertex.cost


def bidirectional_heuristic(vertex, goal):
    """
    heuristic function for bidirectional search, uses euclidean distance
    :param vertex: current vertex object
    :param goal: x,y of goal
    :return: heuristic value
    """
    return vertex.cost + euclidean_distance_between(vertex.current, goal)


def manhattan_heuristic(vertex):
    """
    calculates heuristic value for vertex, uses manhattan distance
    :param vertex: current vertex object
    :return: heuristic value
    """
    return vertex.cost + manhattan_distance(vertex.current)


def manhattan_greedy_heuristic(vertex):
    """
    heuristic function for greedy search, uses manhattan distance
    :param vertex: current vertex object
    :return: heuristic value
    """
    return manhattan_distance(vertex.current)


def manhattan_bidirectional_heuristic(vertex, goal):
    """
    heuristic function for greedy search, uses euclidean distance
    :param vertex: current vertex object
    :param goal: x,y of goal
    :return: heuristic value
    """
    return vertex.cost + manhattan_distance_between(vertex.current, goal)


def expand_fringe(vertex):
    """
    expands fringe
    :param vertex: vertex object of node being expanded
    :return: set of new nodes to be added to fringe
    """
    new_nodes = set()
    c_cost = vertex.cost
    if vertex.current[0] + 1 < graph_size:
        if cost((vertex.current[0] + 1, vertex.current[1])) == 0:
            pass
        else:
            new_nodes.add(Vertex(vertex, (vertex.current[0] + 1, vertex.current[1]), c_cost))
    if vertex.current[1] + 1 < graph_size:
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


def get_neighbors(current):
    """
    returns the neighbors of current x,y
    :param current: x,y of current node
    :return: list of neighbor nodes
    """
    new_nodes = list()
    if current[0] + 1 < graph_size:
        if cost((current[0] + 1, current[1])) == 0:
            pass
        else:
            new_nodes.append((current[0] + 1, current[1]))
    if current[1] + 1 < graph_size:
        if cost((current[0], current[1] + 1)) == 0:
            pass
        else:
            new_nodes.append((current[0], current[1] + 1))
    if current[0] - 1 >= 0:
        if cost((current[0] - 1, current[1])) == 0:
            pass
        else:
            new_nodes.append((current[0] - 1, current[1]))
    if current[1] - 1 >= 0:
        if cost((current[0], current[1] - 1)) == 0:
            pass
        else:
            new_nodes.append((current[0], current[1] - 1))

    return new_nodes


def plot_path(end_vertex, s, g, di_prev=None, no_weight=False):
    """
    plots the path found by an algorithm
    :param end_vertex: ending vertex, list with vertices
    :param s: starting x,y
    :param g: goal x,y
    :param di_prev: if true, dijkstra's algorithm previous dict for pathing dijkstra
    :param no_weight: if true, search was on uniform cost board
    :return: none
    """
    f = 1
    for end in end_vertex:
        fig = plt.figure(f)
        plt.title(end_vertex[end])

        plt.axis('off')
        plt.xlim(-1, graph_size)
        plt.ylim(-1, graph_size)

        plt.plot(s[0], s[1], 'go', label='Start')
        plt.plot(g[0], g[1], 'bo', label='Goal')

        obstacles_x = list()
        obstacles_y = list()
        for i in range(len(obstacles_list)):
            obstacles_x.append(obstacles_list[i][0])
            obstacles_y.append(obstacles_list[i][1])

        plt.plot(obstacles_x, obstacles_y, 'r.', label='Obstacles')

        if not no_weight:
            plt.imshow(weight_matrix)
            plt.colorbar()

        if isinstance(end, tuple) and di_prev:
            e, l = end
            path = [e]
            vertex = e
            while di_prev[vertex]:
                path.append(di_prev[vertex])
                vertex = di_prev[vertex]
            path.reverse()
            path_x = list()
            path_y = list()
            for i in range(len(path)):
                path_x.append(path[i][0])
                path_y.append(path[i][1])

            if no_weight:
                plt.plot(path_x, path_y, 'k-')
                plt.axis('on')
            else:
                plt.plot(path_x, path_y, 'w-')

        else:
            vertex = end
            path = [vertex.current]
            while vertex.previous:
                path.append(vertex.previous.current)
                vertex = vertex.previous
            path.reverse()
            path_x = list()
            path_y = list()
            for i in range(len(path)):
                path_x.append(path[i][0])
                path_y.append(path[i][1])

            if no_weight:
                plt.plot(path_x, path_y, 'k-')
                plt.axis('on')
            else:
                plt.plot(path_x, path_y, 'w-')

        plt.legend()
        f += 1

    plt.show(block=False)
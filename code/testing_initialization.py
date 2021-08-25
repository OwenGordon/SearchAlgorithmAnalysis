"""
Author: Owen Gordon, owgordon@iu.edu
Initializes board and attributes of board for a run of a search
"""

import noise
import random

'''
These are the Parameters to Change and Test
The Iterative Deepening Algorithms need a size below 10 to complete in any reasonable time
'''
size = 100
weight_factor = None
obstacles = None
beam_size = None
num_runs = None

visualize = False

presets = list()
presets.append(((0, 0), (size - 1, size - 1)))

def get_size():
    """
    gets size of the board
    :return: board size
    """
    return size


def get_beam_size():
    """
    gets current beam size
    :return: beam size
    """
    return beam_size


def _initialize(size_, weight_factor_, obstacles_percentage, no_weight=False, beam_size_=None, num_runs_=None, visualize_=False, preset=0):
    """
    initializes a board state
    :param size_: size of board
    :param weight_factor_: maximum weight value in board
    :param obstacles_percentage: percentage of board as obstacles
    :param no_weight: if true, then uniform weight of board
    :param beam_size_: if true sets beam size
    :param num_runs_: if true executes multiple times
    :param visualize_: if true, visualizes board
    :param preset: if greater than 0, initializes board according to a predefined option
    :return: tuple containing weight matrix, obstacales list, source node, goal node
    """
    global size
    size = size_

    if no_weight:
        weight_matrix = [[1 for x in range(size)] for y in range(size)]
    else:
        weight_matrix = [[0 for x in range(size)] for y in range(size)]

        global weight_factor
        weight_factor = weight_factor_

        for x in range(size):
            for y in range(size):
                weight_matrix[x][y] = abs(noise.snoise2(x * 0.03, y * 0.03)) * weight_factor

    if preset == 0:
        source = random.randint(0, int(size / 2 - 1)), random.randint(0, int(size / 2 - 1))
        goal = random.randint(int(size / 2), size - 1), random.randint(int(size / 2), size - 1)
    else:
        source, goal = presets[preset - 1]

    obstacles_list = list()

    global obstacles
    if obstacles_percentage:
        obstacles = (size * size) * (obstacles_percentage/100)
    else:
        obstacles = 0

    obstacles = int(obstacles)

    for i in range(obstacles):
        obstacle = random.randint(0, size - 1), random.randint(0, size - 1)
        while obstacle in obstacles_list or obstacle == source or obstacle == goal:
            obstacle = random.randint(0, size - 1), random.randint(0, size - 1)
        weight_matrix[obstacle[1]][obstacle[0]] = 0
        obstacles_list.append(obstacle)

    global visualize
    visualize = visualize_

    if beam_size_:
        global beam_size
        beam_size = beam_size_

    if num_runs_:
        global num_runs
        num_runs = num_runs_

    return weight_matrix, obstacles_list, source, goal

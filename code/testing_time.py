"""
Author: Owen Gordon, owgordon@iu.edu
Performs algorithm trials for time and space complexity analysis
"""
import time
from testing_algorithms import *
from testing_helpers import *
from testing_initialization import get_beam_size, visualize


def specific_testing(min_size, max_size, step_size, weight_factor, obstacles_percentage, normalization, beam_size=1,
                     manhattan=False, euclidean=False, no_weight=False, end_at_found=True, bfs=False, dfs=False,
                     iddfs=False, ucs=False, greedy=False, astar=False, idastar=False, beam=False, di=False, bi=False,
                     test_beam=False):
    """
    Gather data for time and space complexity analysis
    :param min_size: minimum board size
    :param max_size: maximum board size
    :param step_size: step size between minimum and maximum
    :param weight_factor: max board weight
    :param obstacles_percentage: percent of board as obstalces
    :param normalization: number of trials to average over
    :param beam_size: chosen beam size
    :param manhattan: if true, manhattan heuristic is used
    :param euclidean: if true, euclidean heuristic is used
    :param no_weight: if true, board has uniform cost
    :param end_at_found: if false, dijkstra's algorithm will search entire board
    :param bfs: if true, bfs is tested
    :param dfs: if true, dfs is tested
    :param iddfs: if true, iddfst is tested
    :param ucs: if true, ucs is tested
    :param greedy: if true, greedy is tested
    :param astar: if true, astar is tested
    :param idastar: if true, idastar is tested
    :param beam: if true, beam is tested
    :param di: if true, dijkstras is tested
    :param bi: if true, bidirectional is tested
    :param test_beam: if true, all beam lengths are tested
    :return: testing dict with all testing info
    """
    testing_dict = dict()

    '''
    Iterates from min board to max board, incrementing every step_size amount
    '''
    for i in range(min_size, max_size + 1, step_size):

        print('SIZE=' + str(i))

        '''
        Tests BFS algorithm
        '''
        if bfs:
            if i == min_size:
                testing_dict['BFS'] = dict()

            print('BFS on a graph on size of ' + str(i))
            bfs_times = list()
            bfs_lengths = list()
            bfs_costs = list()
            for j in range(normalization):
                initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight)
                source = get_source()
                goal = get_goal()
                starting_vertex = Vertex(None, source, 0)

                start = time.time()
                bfs_end, bfs_length = uninformed_search(starting_vertex, goal)
                end = time.time()
                bfs_cost = bfs_end.cost
                if bfs_end.current != goal:
                    print('Uninformed BFS found the wrong goal')
                    bfs_time = None
                    bfs_length = None
                    bfs_cost = None
                else:
                    bfs_time = end - start
                    print('Time: ' + str(bfs_time))
                    if visualize: plot_path(bfs_end)
                bfs_times.append(bfs_time)
                bfs_lengths.append(bfs_length)
                bfs_costs.append(bfs_cost)
            testing_dict['BFS'][i] = bfs_times, bfs_lengths, bfs_costs
            print()

        '''
        Tests DFS algorithm
        '''
        if dfs:
            if i == min_size:
                testing_dict['DFS'] = dict()

            print('DFS on a graph on size of ' + str(i))
            dfs_times = list()
            dfs_lengths = list()
            dfs_costs = list()
            for j in range(normalization):
                initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                source = get_source()
                goal = get_goal()
                starting_vertex = Vertex(None, source, 0)

                start = time.time()
                dfs_end, dfs_length = uninformed_search(starting_vertex, goal, dfs=True)
                end = time.time()
                dfs_cost = dfs_end.cost
                if dfs_end.current != goal:
                    print('Uninformed DFS found the wrong goal')
                    dfs_time = None
                    dfs_length = None
                    dfs_cost = None
                else:
                    dfs_time = end - start
                    print('Time: ' + str(dfs_time))
                    if visualize: plot_path(dfs_end)
                dfs_times.append(dfs_time)
                dfs_lengths.append(dfs_length)
                dfs_costs.append(dfs_cost)
            testing_dict['DFS'][i] = dfs_times, dfs_lengths, dfs_costs
            print()

        '''
        Tests IDDFS algorithm
        '''
        if iddfs and i < 10:
            if i == min_size:
                testing_dict['IDDFS'] = dict()

            print('IDDFS on a graph on size of ' + str(i))
            iddfs_times = list()
            iddfs_lengths = list()
            iddfs_costs = list()
            for j in range(normalization):
                initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                source = get_source()
                goal = get_goal()
                starting_vertex = Vertex(None, source, 0)

                start = time.time()
                iddfs_end, iddfs_depth = IDDFS(starting_vertex, goal)
                end = time.time()
                iddfs_cost = iddfs_end.cost
                if iddfs_end.current != goal:
                    print('Uninformed Iterative Deepening DFS found the wrong goal')
                    iddfs_time = None
                    iddfs_length = None
                    iddfs_cost = None
                else:
                    iddfs_time = end - start
                    print('Time: ' + str(iddfs_time))
                    if visualize: plot_path(iddfs_end)
                iddfs_times.append(iddfs_time)
                iddfs_lengths.append(4 ** iddfs_depth)
                iddfs_costs.append(iddfs_cost)
            testing_dict['IDDFS'][i] = iddfs_times, iddfs_lengths, iddfs_costs
            print()

        '''
        Tests UCS algorithm
        '''
        if ucs:
            if i == min_size:
                testing_dict['UCS'] = dict()

            print('UCS on a graph on size of ' + str(i))
            ucs_times = list()
            ucs_lengths = list()
            ucs_costs = list()
            for j in range(normalization):
                initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                source = get_source()
                goal = get_goal()
                starting_vertex = Vertex(None, source, 0)

                start = time.time()
                ucs_end, ucs_length = UCS(starting_vertex, goal)
                end = time.time()
                ucs_cost = ucs_end.cost
                if ucs_end.current != goal:
                    print('Informed UCS found the wrong goal')
                    ucs_time = None
                    ucs_length = None
                    ucs_cost = None
                else:
                    ucs_time = end - start
                    print('Time: ' + str(ucs_time))
                    if visualize: plot_path(ucs_end)
                ucs_times.append(ucs_time)
                ucs_lengths.append(ucs_length)
                ucs_costs.append(ucs_cost)
            testing_dict['UCS'][i] = ucs_times, ucs_lengths, ucs_costs
            print()

        '''
        Tests Greedy algorithm
        '''
        if greedy:
            if manhattan and euclidean:
                if i == min_size:
                    testing_dict['Greedy Manhattan'] = dict()
                    testing_dict['Greedy Euclidean'] = dict()
            else:
                if i == min_size:
                    testing_dict['Greedy'] = dict()

            '''
            Tests specified heuristic
            '''
            if manhattan and euclidean:
                print('Greedy Search using Manhattan distance on a graph on size of ' + str(i))
                greedy_times = list()
                greedy_lengths = list()
                greedy_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    greedy_end, greedy_length = greedy_search(starting_vertex, goal, manhattan=True)
                    end = time.time()
                    greedy_cost = greedy_end.cost
                    if greedy_end.current != goal:
                        print('Informed Greedy Search found the wrong goal')
                        greedy_time = None
                        greedy_length = None
                        greedy_cost = None
                    else:
                        greedy_time = end - start
                        print('Time: ' + str(greedy_time))
                        if visualize: plot_path(greedy_end)
                    greedy_times.append(greedy_time)
                    greedy_lengths.append(greedy_length)
                    greedy_costs.append(greedy_cost)
                testing_dict['Greedy Manhattan'][i] = greedy_times, greedy_lengths, greedy_costs
                print()

                print('Greedy Search using Euclidean distance on a graph on size of ' + str(i))
                greedy_times = list()
                greedy_lengths = list()
                greedy_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    greedy_end, greedy_length = greedy_search(starting_vertex, goal)
                    end = time.time()
                    greedy_cost = greedy_end.cost
                    if greedy_end.current != goal:
                        print('Informed Greedy Search found the wrong goal')
                        greedy_time = None
                        greedy_length = None
                        greedy_cost = None
                    else:
                        greedy_time = end - start
                        print('Time: ' + str(greedy_time))
                        if visualize: plot_path(greedy_end)
                    greedy_times.append(greedy_time)
                    greedy_lengths.append(greedy_length)
                    greedy_costs.append(greedy_cost)
                testing_dict['Greedy Euclidean'][i] = greedy_times, greedy_lengths, greedy_costs

            elif euclidean:
                print('Greedy Search on a graph on size of ' + str(i))
                greedy_times = list()
                greedy_lengths = list()
                greedy_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    greedy_end, greedy_length = greedy_search(starting_vertex, goal)
                    end = time.time()
                    greedy_cost = greedy_end.cost
                    if greedy_end.current != goal:
                        print('Informed Greedy Search found the wrong goal')
                        greedy_time = None
                        greedy_length = None
                        greedy_cost = None
                    else:
                        greedy_time = end - start
                        print('Time: ' + str(greedy_time))
                        if visualize: plot_path(greedy_end)
                    greedy_times.append(greedy_time)
                    greedy_lengths.append(greedy_length)
                    greedy_costs.append(greedy_cost)
                testing_dict['Greedy'][i] = greedy_times, greedy_lengths, greedy_costs
            elif manhattan:
                print('Greedy Search on a graph on size of ' + str(i))
                greedy_times = list()
                greedy_lengths = list()
                greedy_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    greedy_end, greedy_length = greedy_search(starting_vertex, goal, manhattan=True)
                    end = time.time()
                    greedy_cost = greedy_end.cost
                    if greedy_end.current != goal:
                        print('Informed Greedy Search found the wrong goal')
                        greedy_time = None
                        greedy_length = None
                        greedy_cost = None
                    else:
                        greedy_time = end - start
                        print('Time: ' + str(greedy_time))
                        if visualize: plot_path(greedy_end)
                    greedy_times.append(greedy_time)
                    greedy_lengths.append(greedy_length)
                    greedy_costs.append(greedy_cost)
                testing_dict['Greedy'][i] = greedy_times, greedy_lengths, greedy_costs
            else:
                print('Greedy Search on a graph on size of ' + str(i))
                greedy_times = list()
                greedy_lengths = list()
                greedy_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    greedy_end, greedy_length = greedy_search(starting_vertex, goal)
                    end = time.time()
                    greedy_cost = greedy_end.cost
                    if greedy_end.current != goal:
                        print('Informed Greedy Search found the wrong goal')
                        greedy_time = None
                        greedy_length = None
                        greedy_cost = None
                    else:
                        greedy_time = end - start
                        print('Time: ' + str(greedy_time))
                        if visualize: plot_path(greedy_end)
                    greedy_times.append(greedy_time)
                    greedy_lengths.append(greedy_length)
                    greedy_costs.append(greedy_cost)
                testing_dict['Greedy'][i] = greedy_times, greedy_lengths, greedy_costs

            print()

        '''
        Tests A* algorithm
        '''
        if astar:
            if manhattan and euclidean:
                if i == min_size:
                    testing_dict['A Star Manhattan'] = dict()
                    testing_dict['A Star Euclidean'] = dict()
            else:
                if i == min_size:
                    testing_dict['A Star'] = dict()

            '''
            Tests specified heuristic
            '''
            if manhattan and euclidean:
                print('A* using Manhattan distance on a graph on size of ' + str(i))
                a_star_times = list()
                a_star_lengths = list()
                a_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    a_star_end, a_star_length = a_star(starting_vertex, goal, manhattan=True)
                    end = time.time()
                    a_star_cost = a_star_end.cost
                    if a_star_end.current != goal:
                        print('Informed A* found the wrong goal')
                        a_star_time = None
                        a_star_length = None
                        a_star_cost = None
                    else:
                        a_star_time = end - start
                        print('Time ' + str(a_star_time))
                        if visualize: plot_path(a_star_end)
                    a_star_times.append(a_star_time)
                    a_star_lengths.append(a_star_length)
                    a_star_costs.append(a_star_cost)
                testing_dict['A Star Manhattan'][i] = a_star_times, a_star_lengths, a_star_costs
                print()

                print('A* using Euclidean distance on a graph on size of ' + str(i))
                a_star_times = list()
                a_star_lengths = list()
                a_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    a_star_end, a_star_length = a_star(starting_vertex, goal)
                    end = time.time()
                    a_star_cost = a_star_end.cost
                    if a_star_end.current != goal:
                        print('Informed A* found the wrong goal')
                        a_star_time = None
                        a_star_length = None
                        a_star_cost = None
                    else:
                        a_star_time = end - start
                        print('Time ' + str(a_star_time))
                        if visualize: plot_path(a_star_end)
                    a_star_times.append(a_star_time)
                    a_star_lengths.append(a_star_length)
                    a_star_costs.append(a_star_cost)
                testing_dict['A Star Euclidean'][i] = a_star_times, a_star_lengths, a_star_costs
            elif euclidean:
                print('A* on a graph on size of ' + str(i))
                a_star_times = list()
                a_star_lengths = list()
                a_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    a_star_end, a_star_length = a_star(starting_vertex, goal)
                    end = time.time()
                    a_star_cost = a_star_end.cost
                    if a_star_end.current != goal:
                        print('Informed A* found the wrong goal')
                        a_star_time = None
                        a_star_length = None
                        a_star_cost = None
                    else:
                        a_star_time = end - start
                        print('Time ' + str(a_star_time))
                        if visualize: plot_path(a_star_end)
                    a_star_times.append(a_star_time)
                    a_star_lengths.append(a_star_length)
                    a_star_costs.append(a_star_cost)
                testing_dict['A Star'][i] = a_star_times, a_star_lengths, a_star_costs
            elif manhattan:
                print('A* on a graph on size of ' + str(i))
                a_star_times = list()
                a_star_lengths = list()
                a_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    a_star_end, a_star_length = a_star(starting_vertex, goal, manhattan=True)
                    end = time.time()
                    a_star_cost = a_star_end.cost
                    if a_star_end.current != goal:
                        print('Informed A* found the wrong goal')
                        a_star_time = None
                        a_star_length = None
                        a_star_cost = None
                    else:
                        a_star_time = end - start
                        print('Time ' + str(a_star_time))
                        if visualize: plot_path(a_star_end)
                    a_star_times.append(a_star_time)
                    a_star_lengths.append(a_star_length)
                    a_star_costs.append(a_star_cost)
                testing_dict['A Star'][i] = a_star_times, a_star_lengths, a_star_costs
            else:
                print('A* on a graph on size of ' + str(i))
                a_star_times = list()
                a_star_lengths = list()
                a_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    a_star_end, a_star_length = a_star(starting_vertex, goal)
                    end = time.time()
                    a_star_cost = a_star_end.cost
                    if a_star_end.current != goal:
                        print('Informed A* found the wrong goal')
                        a_star_time = None
                        a_star_length = None
                        a_star_cost = None
                    else:
                        a_star_time = end - start
                        print('Time ' + str(a_star_time))
                        if visualize: plot_path(a_star_end)
                    a_star_times.append(a_star_time)
                    a_star_lengths.append(a_star_length)
                    a_star_costs.append(a_star_cost)
                testing_dict['A Star'][i] = a_star_times, a_star_lengths, a_star_costs

            print()

        '''
        Tests IDA* algorithm
        '''
        if idastar and False:
            if manhattan and euclidean:
                if i == min_size:
                    testing_dict['IDA Star Manhattan'] = dict()
                    testing_dict['IDA Star Euclidean'] = dict()
            else:
                if i == min_size:
                    testing_dict['IDA Star'] = dict()

            '''
            Tests specified heuristic
            '''
            if manhattan and euclidean:
                print('IDA* using Manhattan distance on a graph on size of ' + str(i))
                ida_star_times = list()
                ida_star_lengths = list()
                ida_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    ida_star_end, ida_star_depth = ida_star(starting_vertex, goal, manhattan=True)
                    end = time.time()
                    ida_star_cost = ida_star_end.cost
                    if ida_star_end.current != goal:
                        print('Informed IDA* found the wrong goal')
                        ida_star_time = None
                        ida_star_length = None
                        ida_star_cost = None
                    else:
                        ida_star_time = end - start
                        print('Time: ' + str(ida_star_time))
                        if visualize: plot_path(ida_star_end)
                    ida_star_times.append(ida_star_time)
                    ida_star_lengths.append(ida_star_length)
                    ida_star_costs.append(ida_star_cost)
                testing_dict['IDA Star Manhattan'][i] = ida_star_times, ida_star_lengths, ida_star_costs
                print()

                print('IDA* using euclidean on a graph on size of ' + str(i))
                ida_star_times = list()
                ida_star_lengths = list()
                ida_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    ida_star_end, ida_star_depth = ida_star(starting_vertex, goal)
                    end = time.time()
                    ida_star_cost = ida_star_end.cost
                    if ida_star_end.current != goal:
                        print('Informed IDA* found the wrong goal')
                        ida_star_time = None
                        ida_star_length = None
                        ida_star_cost = None
                    else:
                        ida_star_time = end - start
                        print('Time: ' + str(ida_star_time))
                        if visualize: plot_path(ida_star_end)
                    ida_star_times.append(ida_star_time)
                    ida_star_lengths.append(ida_star_length)
                    ida_star_costs.append(ida_star_cost)
                testing_dict['IDA Star Euclidean'][i] = ida_star_times, ida_star_lengths, ida_star_costs

            elif euclidean:
                print('IDA* on a graph on size of ' + str(i))
                ida_star_times = list()
                ida_star_lengths = list()
                ida_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    ida_star_end, ida_star_depth = ida_star(starting_vertex, goal)
                    end = time.time()
                    ida_star_cost = ida_star_end.cost
                    if ida_star_end.current != goal:
                        print('Informed IDA* found the wrong goal')
                        ida_star_time = None
                        ida_star_length = None
                        ida_star_cost = None
                    else:
                        ida_star_time = end - start
                        print('Time: ' + str(ida_star_time))
                        if visualize: plot_path(ida_star_end)
                    ida_star_times.append(ida_star_time)
                    ida_star_lengths.append(ida_star_length)
                    ida_star_costs.append(ida_star_cost)
                testing_dict['IDA Star'][i] = ida_star_times, ida_star_lengths, ida_star_costs
            elif manhattan:
                print('IDA* on a graph on size of ' + str(i))
                ida_star_times = list()
                ida_star_lengths = list()
                ida_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    ida_star_end, ida_star_depth = ida_star(starting_vertex, goal, manhattan=True)
                    end = time.time()
                    ida_star_cost = ida_star_end.cost
                    if ida_star_end.current != goal:
                        print('Informed IDA* found the wrong goal')
                        ida_star_time = None
                        ida_star_length = None
                        ida_star_cost = None
                    else:
                        ida_star_time = end - start
                        print('Time: ' + str(ida_star_time))
                        if visualize: plot_path(ida_star_end)
                    ida_star_times.append(ida_star_time)
                    ida_star_lengths.append(ida_star_length)
                    ida_star_costs.append(ida_star_cost)
                testing_dict['IDA Star'][i] = ida_star_times, ida_star_lengths, ida_star_costs
            else:
                print('IDA* on a graph on size of ' + str(i))
                ida_star_times = list()
                ida_star_lengths = list()
                ida_star_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    ida_star_end, ida_star_depth = ida_star(starting_vertex, goal)
                    end = time.time()
                    ida_star_cost = ida_star_end.cost
                    if ida_star_end.current != goal:
                        print('Informed IDA* found the wrong goal')
                        ida_star_time = None
                        ida_star_length = None
                        ida_star_cost = None
                    else:
                        ida_star_time = end - start
                        print('Time: ' + str(ida_star_time))
                        if visualize: plot_path(ida_star_end)
                    ida_star_times.append(ida_star_time)
                    ida_star_lengths.append(ida_star_length)
                    ida_star_costs.append(ida_star_cost)
                testing_dict['IDA Star'][i] = ida_star_times, ida_star_lengths, ida_star_costs
            print()

        '''
        Tests Beam algorithm
        '''
        if beam and test_beam:
            for k in range(1, 5):
                beam_size = k
                if i == min_size:
                    testing_dict['Beam=' + str(beam_size)] = dict()

                print('Beam (length=' + str(beam_size) + ') on a graph on size of ' + str(i))
                beam_times = list()
                beam_lengths = list()
                beam_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, beam_size=beam_size,
                               visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    beam_end, beam_length = beam_search(starting_vertex, goal, beam_size)
                    end = time.time()
                    beam_cost = beam_end.cost
                    if beam_end.current != goal:
                        print('Informed Beam Search (length=' + str(beam_size) + ') found the wrong goal')
                        beam_time = None
                        beam_length = None
                        beam_cost = None
                    else:
                        beam_time = end - start
                        print('Time: ' + str(beam_time))
                        if visualize: plot_path(beam_end)
                    beam_times.append(beam_time)
                    beam_lengths.append(beam_length)
                    beam_costs.append(beam_cost)
                testing_dict['Beam=' + str(beam_size)][i] = beam_times, beam_lengths, beam_costs
                print()
        elif beam:
            if manhattan and euclidean:
                if i == min_size:
                    testing_dict['Beam Manhattan'] = dict()
                    testing_dict['Beam Euclidean'] = dict()
            else:
                if i == min_size:
                    testing_dict['Beam'] = dict()

            '''
            Tests specified heuristic
            '''
            if manhattan and euclidean:
                print('Beam using Manhattan distance (length=' + str(beam_size) + ') on a graph on size of ' + str(i))
                beam_times = list()
                beam_lengths = list()
                beam_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, beam_size=beam_size,
                               visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    beam_end, beam_length = beam_search(starting_vertex, goal, beam_size, manhattan=True)
                    end = time.time()
                    beam_cost = beam_end.cost
                    if beam_end.current != goal:
                        print('Informed Beam Search found the wrong goal')
                        beam_time = None
                        beam_length = None
                        beam_cost = None
                    else:
                        beam_time = end - start
                        print('Time: ' + str(beam_time))
                        if visualize: plot_path(beam_end)
                    beam_times.append(beam_time)
                    beam_lengths.append(beam_length)
                    beam_costs.append(beam_cost)
                testing_dict['Beam Manhattan'][i] = beam_times, beam_lengths, beam_costs
                print()

                print('Beam using Euclidean distance (length=' + str(beam_size) + ') on a graph on size of ' + str(i))
                beam_times = list()
                beam_lengths = list()
                beam_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, beam_size=beam_size,
                               visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    beam_end, beam_length = beam_search(starting_vertex, goal, beam_size)
                    end = time.time()
                    beam_cost = beam_end.cost
                    if beam_end.current != goal:
                        print('Informed Beam Search found the wrong goal')
                        beam_time = None
                        beam_length = None
                        beam_cost = None
                    else:
                        beam_time = end - start
                        print('Time: ' + str(beam_time))
                        if visualize: plot_path(beam_end)
                    beam_times.append(beam_time)
                    beam_lengths.append(beam_length)
                    beam_costs.append(beam_cost)
                testing_dict['Beam Euclidean'][i] = beam_times, beam_lengths, beam_costs

            elif euclidean:
                print('Beam (length=' + str(beam_size) + ') on a graph on size of ' + str(i))
                beam_times = list()
                beam_lengths = list()
                beam_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, beam_size=beam_size,
                               visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    beam_end, beam_length = beam_search(starting_vertex, goal, beam_size)
                    end = time.time()
                    beam_cost = beam_end.cost
                    if beam_end.current != goal:
                        print('Informed Beam Search found the wrong goal')
                        beam_time = None
                        beam_length = None
                        beam_cost = None
                    else:
                        beam_time = end - start
                        print('Time: ' + str(beam_time))
                        if visualize: plot_path(beam_end)
                    beam_times.append(beam_time)
                    beam_lengths.append(beam_length)
                    beam_costs.append(beam_cost)
                testing_dict['Beam'][i] = beam_times, beam_lengths, beam_costs
            elif manhattan:
                print('Beam (length=' + str(beam_size) + ') on a graph on size of ' + str(i))
                beam_times = list()
                beam_lengths = list()
                beam_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, beam_size=beam_size,
                               visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    beam_end, beam_length = beam_search(starting_vertex, goal, beam_size, manhattan=True)
                    end = time.time()
                    beam_cost = beam_end.cost
                    if beam_end.current != goal:
                        print('Informed Beam Search found the wrong goal')
                        beam_time = None
                        beam_length = None
                        beam_cost = None
                    else:
                        beam_time = end - start
                        print('Time: ' + str(beam_time))
                        if visualize: plot_path(beam_end)
                    beam_times.append(beam_time)
                    beam_lengths.append(beam_length)
                    beam_costs.append(beam_cost)
                testing_dict['Beam'][i] = beam_times, beam_lengths, beam_costs

            else:
                print('Beam (length=' + str(beam_size) + ') on a graph on size of ' + str(i))
                beam_times = list()
                beam_lengths = list()
                beam_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, beam_size=beam_size,
                               visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)

                    start = time.time()
                    beam_end, beam_length = beam_search(starting_vertex, goal, beam_size, manhattan=True)
                    end = time.time()
                    beam_cost = beam_end.cost
                    if beam_end.current != goal:
                        print('Informed Beam Search found the wrong goal')
                        beam_time = None
                        beam_length = None
                        beam_cost = None
                    else:
                        beam_time = end - start
                        print('Time: ' + str(beam_time))
                        if visualize: plot_path(beam_end)
                    beam_times.append(beam_time)
                    beam_lengths.append(beam_length)
                    beam_costs.append(beam_cost)
                testing_dict['Beam'][i] = beam_times, beam_lengths, beam_costs

            print()

        '''
        Tests Dijkstra's algorithm
        '''
        if di:
            if not end_at_found:
                if i == min_size:
                    testing_dict['Dijkstras Full'] = dict()

            if i == min_size:
                testing_dict['Dijkstras'] = dict()

            print('Dijkstras on a graph on size of ' + str(i))
            di_times = list()
            di_lengths = list()
            di_costs = list()
            for j in range(normalization):
                initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                source = get_source()
                goal = get_goal()
                weight_matrix = get_weight_matrix()

                start = time.time()
                di_end, di_cost, di_prev = dijkstras(source, goal, weight_matrix)
                end = time.time()
                total_nodes_analyzed = 0
                for d in di_cost:
                    if math.isfinite(di_cost[d]):
                        total_nodes_analyzed += 1
                di_cost = di_cost[di_end]
                if di_end != goal:
                    print('Informed Dijkstras search found the wrong goal')
                    di_time = None
                    di_cost = None
                else:
                    di_time = end - start
                    print('Time: ' + str(di_time))
                    if visualize: plot_path(di_end, di_prev=di_prev)
                di_times.append(di_time)
                di_lengths.append(total_nodes_analyzed)
                di_costs.append(di_cost)
            testing_dict['Dijkstras'][i] = di_times, di_lengths, di_costs
            print()

            '''
            Tests Dijkstra's on full graph
            '''
            if not end_at_found:
                print('Dijkstras searching full graph, on a graph on size of ' + str(i))
                di_times = list()
                di_lengths = list()
                di_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    weight_matrix = get_weight_matrix()

                    start = time.time()
                    di_end, di_cost, di_prev = dijkstras(source, goal, weight_matrix, end_at_found=end_at_found)
                    end = time.time()
                    total_nodes_analyzed = 0
                    for d in di_cost:
                        if math.isfinite(di_cost[d]):
                            total_nodes_analyzed += 1
                    di_cost = di_cost[di_end]
                    if di_end != goal:
                        print('Informed Dijkstras search found the wrong goal')
                        di_time = None
                        di_cost = None
                    else:
                        di_time = end - start
                        print('Time: ' + str(di_time))
                        if visualize: plot_path(di_end, di_prev=di_prev)
                    di_times.append(di_time)
                    di_lengths.append(total_nodes_analyzed)
                    di_costs.append(di_cost)
                testing_dict['Dijkstras Full'][i] = di_times, di_lengths, di_costs
                print()

        '''
        Tests Bidirectional algorithm
        '''
        if bi:
            if manhattan and euclidean:
                if i == min_size:
                    testing_dict['Bidirectional Manhattan'] = dict()
                    testing_dict['Bidirectional Euclidean'] = dict()
            else:
                if i == min_size:
                    testing_dict['Bidirectional'] = dict()

            '''
            Tests specified heuristic
            '''
            if manhattan and euclidean:
                print('Bidirectional using Manhattan distance on a graph on size of ' + str(i))
                bi_times = list()
                bi_lengths = list()
                bi_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)
                    goal_vertex = Vertex(None, goal, 0)

                    start = time.time()
                    bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex, manhattan=True)
                    end = time.time()
                    bi_cost = bi_end.cost
                    if bi_end.current != goal:
                        print('Informed Bidirectional Search found the wrong goal')
                        bi_time = None
                        bi_length = None
                        bi_cost = None
                    else:
                        bi_time = end - start
                        print('Time: ' + str(bi_time))
                        if visualize: plot_path(bi_end)
                    bi_times.append(bi_time)
                    bi_lengths.append(bi_length)
                    bi_costs.append(bi_cost)
                testing_dict['Bidirectional Manhattan'][i] = bi_times, bi_lengths, bi_costs
                print()

                print('Bidirectional using Euclidean distance on a graph on size of ' + str(i))
                bi_times = list()
                bi_lengths = list()
                bi_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)
                    goal_vertex = Vertex(None, goal, 0)

                    start = time.time()
                    bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex)
                    end = time.time()
                    bi_cost = bi_end.cost
                    if bi_end.current != goal:
                        print('Informed Bidirectional Search found the wrong goal')
                        bi_time = None
                        bi_length = None
                        bi_cost = None
                    else:
                        bi_time = end - start
                        print('Time: ' + str(bi_time))
                        if visualize: plot_path(bi_end)
                    bi_times.append(bi_time)
                    bi_lengths.append(bi_length)
                    bi_costs.append(bi_cost)
                testing_dict['Bidirectional Euclidean'][i] = bi_times, bi_lengths, bi_costs

            elif euclidean:
                print('Bidirectional on a graph on size of ' + str(i))
                bi_times = list()
                bi_lengths = list()
                bi_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)
                    goal_vertex = Vertex(None, goal, 0)

                    start = time.time()
                    bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex)
                    end = time.time()
                    bi_cost = bi_end.cost
                    if bi_end.current != goal:
                        print('Informed Bidirectional Search found the wrong goal')
                        bi_time = None
                        bi_length = None
                        bi_cost = None
                    else:
                        bi_time = end - start
                        print('Time: ' + str(bi_time))
                        if visualize: plot_path(bi_end)
                    bi_times.append(bi_time)
                    bi_lengths.append(bi_length)
                    bi_costs.append(bi_cost)
                testing_dict['Bidirectional'][i] = bi_times, bi_lengths, bi_costs
            elif manhattan:
                print('Bidirectional on a graph on size of ' + str(i))
                bi_times = list()
                bi_lengths = list()
                bi_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)
                    goal_vertex = Vertex(None, goal, 0)

                    start = time.time()
                    bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex, manhattan=True)
                    end = time.time()
                    bi_cost = bi_end.cost
                    if bi_end.current != goal:
                        print('Informed Bidirectional Search found the wrong goal')
                        bi_time = None
                        bi_length = None
                        bi_cost = None
                    else:
                        bi_time = end - start
                        print('Time: ' + str(bi_time))
                        if visualize: plot_path(bi_end)
                    bi_times.append(bi_time)
                    bi_lengths.append(bi_length)
                    bi_costs.append(bi_cost)
                testing_dict['Bidirectional'][i] = bi_times, bi_lengths, bi_costs
            else:
                print('Bidirectional on a graph on size of ' + str(i))
                bi_times = list()
                bi_lengths = list()
                bi_costs = list()
                for j in range(normalization):
                    initialize(i, weight_factor, obstacles_percentage, no_weight=no_weight, visualize=visualize)
                    source = get_source()
                    goal = get_goal()
                    beam_size = get_beam_size()
                    starting_vertex = Vertex(None, source, 0)
                    goal_vertex = Vertex(None, goal, 0)

                    start = time.time()
                    bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex)
                    end = time.time()
                    bi_cost = bi_end.cost
                    if bi_end.current != goal:
                        print('Informed Bidirectional Search found the wrong goal')
                        bi_time = None
                        bi_length = None
                        bi_cost = None
                    else:
                        bi_time = end - start
                        print('Time: ' + str(bi_time))
                        if visualize: plot_path(bi_end)
                    bi_times.append(bi_time)
                    bi_lengths.append(bi_length)
                    bi_costs.append(bi_cost)
                testing_dict['Bidirectional'][i] = bi_times, bi_lengths, bi_costs
            print()

    return testing_dict

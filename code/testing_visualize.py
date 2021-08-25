"""
Author: Owen Gordon, owgordon@iu.edu
Performs an iteration of algorithm searches on a specific size to plot the algorithm path
"""
import time
from testing_algorithms import *
from testing_helpers import *


def broad_testing(size, weight_factor, obstacles_percentage, num_runs, beam_size=1, test_beam=False, manhattan=False,
                  euclidean=False, end_at_found=True, no_weight=False, visualize=False, preset=0):
    """
    Gathers the algorithm data for scores, as well as plotting the path
    :param size: size of board
    :param weight_factor: maximum weight of board
    :param obstacles_percentage: percentage of obstalces in board
    :param num_runs: number of different runs
    :param beam_size: beam size
    :param test_beam: if true, tests all
    :param manhattan: if true, manhattan distance is used for heuristic
    :param euclidean: if true, euclidean distance is used for heuristic
    :param end_at_found: if false, dijkstra's will search full graph
    :param no_weight: if true, the board has uniform weight cost
    :param visualize: if true, the paths of the algorithms will be visualized
    :param preset: if greater than 0 a preset will be used
    :return: testing dict with algorithm information
    """
    small = False
    if size < 10:
        small = True
        print('You chose a small size, all algorithms will be tested')
    else:
        print('You chose a large size, iterative deepening algorithms will be excluded')

    testing_dict = dict()

    attribute_dict = dict()
    attribute_dict['Size'] = size
    if no_weight:
        attribute_dict['Weight Factor'] = 1
    else:
        attribute_dict['Weight Factor'] = weight_factor
    attribute_dict['Obstacles'] = int((size * size) * (obstacles_percentage/100))
    attribute_dict['Beam Size'] = beam_size

    testing_dict['Attributes'] = attribute_dict

    for i in range(num_runs):
        initialize(size, weight_factor, obstacles_percentage, no_weight=no_weight, beam_size=beam_size, num_runs=num_runs, visualize=visualize, preset=preset)

        weight_matrix = get_weight_matrix()

        source = get_source()
        goal = get_goal()

        algorithm_performance_dict = dict()

        starting_vertex = Vertex(None, source, 0)
        goal_vertex = Vertex(None, goal, 0)

        end_dict = dict()

        '''
        Tests BFS
        '''
        start = time.time()
        bfs_end, bfs_length = uninformed_search(starting_vertex, goal)
        end = time.time()
        bfs_cost = bfs_end.cost
        end_dict[bfs_end] = 'BFS, cost=' + str(bfs_cost)
        if bfs_end.current != goal:
            print('Uninformed BFS found the wrong goal')
            bfs_time = None
            bfs_cost = None
        else:
            bfs_time = end - start
            print('Uninformed BFS took: ' + str(bfs_time) + ' seconds, and found a path cost of ' + str(
                bfs_cost) + ' after exploring ' + str(bfs_length) + ' nodes')

        algorithm_performance_dict['BFS'] = bfs_time, bfs_cost, bfs_length

        '''
        Tests DFS
        '''
        start = time.time()
        dfs_end, dfs_length = uninformed_search(starting_vertex, goal, dfs=True)
        end = time.time()
        dfs_cost = dfs_end.cost
        end_dict[dfs_end] = 'DFS, cost=' + str(dfs_cost)
        if dfs_end.current != goal:
            print('Uninformed DFS found the wrong goal')
            dfs_time = None
            dfs_cost = None
        else:
            dfs_time = end - start
            print('Uninformed DFS took: ' + str(dfs_time) + ' seconds, and found a path cost of ' + str(
                dfs_cost) + ' after exploring ' + str(dfs_length) + ' nodes')

        algorithm_performance_dict['DFS'] = dfs_time, dfs_cost, dfs_length

        if small:
            start = time.time()
            iddfs_end, iddfs_depth = IDDFS(starting_vertex, goal)
            end = time.time()
            iddfs_cost = iddfs_end.cost
            end_dict[iddfs_end] = 'IDDFS, cost=' + str(iddfs_cost)
            if iddfs_end.current != goal:
                print('Uninformed Iterative Deepening DFS found the wrong goal')
                iddfs_time = None
                iddfs_cost = None
            else:
                iddfs_time = end - start
                print('Uninformed Iterative Deepening DFS took: ' + str(iddfs_time) + ' seconds, and found a path cost of ' + str(
                    iddfs_cost) + ' after exploring a depth of ' + str(iddfs_depth))

            algorithm_performance_dict['IDDFS'] = iddfs_time, iddfs_cost, 4 ** iddfs_depth

        '''
        Tests UCS
        '''
        start = time.time()
        ucs_end, ucs_length = UCS(starting_vertex, goal)
        end = time.time()
        ucs_cost = ucs_end.cost
        end_dict[ucs_end] = 'UCS, cost=' + str(ucs_cost)
        if ucs_end.current != goal:
            print('Informed UCS found the wrong goal')
            ucs_time = None
            ucs_cost = None
        else:
            ucs_time = end - start
            print('Informed UCS took: ' + str(ucs_time) + ' seconds, and found a path cost of ' + str(
                ucs_cost) + ' after exploring ' + str(ucs_length) + ' nodes')

        algorithm_performance_dict['UCS'] = ucs_time, ucs_cost, ucs_length

        '''
        Tests Greedy algorithm using specified heuristic
        '''
        if euclidean and manhattan:
            start = time.time()
            greedy_end, greedy_length = greedy_search(starting_vertex, goal, manhattan=True)
            end = time.time()
            greedy_cost = greedy_end.cost
            end_dict[greedy_end] = 'Greedy Manhattan, cost=' + str(greedy_cost)
            if greedy_end.current != goal:
                print('Informed Greedy Search using Manhattan distance found the wrong goal')
                greedy_time = None
                greedy_cost = None
            else:
                greedy_time = end - start
                print('Informed Greedy Search using Manhattan distance took: ' + str(greedy_time) + ' seconds, and found a path cost of ' + str(
                    greedy_cost) + ' after exploring ' + str(greedy_length) + ' nodes')

            algorithm_performance_dict['Greedy Manhattan'] = greedy_time, greedy_cost, greedy_length

            start = time.time()
            greedy_end, greedy_length = greedy_search(starting_vertex, goal)
            end = time.time()
            greedy_cost = greedy_end.cost
            end_dict[greedy_end] = 'Greedy Euclidean, cost=' + str(greedy_cost)
            if greedy_end.current != goal:
                print('Informed Greedy Search using Euclidean distance found the wrong goal')
                greedy_time = None
                greedy_cost = None
            else:
                greedy_time = end - start
                print('Informed Greedy Search using Euclidean distance took: ' + str(greedy_time) + ' seconds, and found a path cost of ' + str(
                    greedy_cost) + ' after exploring ' + str(greedy_length) + ' nodes')

            algorithm_performance_dict['Greedy Euclidean'] = greedy_time, greedy_cost, greedy_length
        elif euclidean:
            start = time.time()
            greedy_end, greedy_length = greedy_search(starting_vertex, goal)
            end = time.time()
            greedy_cost = greedy_end.cost
            end_dict[greedy_end] = 'Greedy, cost=' + str(greedy_cost)
            if greedy_end.current != goal:
                print('Informed Greedy Search using found the wrong goal')
                greedy_time = None
                greedy_cost = None
            else:
                greedy_time = end - start
                print('Informed Greedy Search using took: ' + str(greedy_time) + ' seconds, and found a path cost of ' + str(
                    greedy_cost) + ' after exploring ' + str(greedy_length) + ' nodes')

            algorithm_performance_dict['Greedy'] = greedy_time, greedy_cost, greedy_length
        elif manhattan:
            start = time.time()
            greedy_end, greedy_length = greedy_search(starting_vertex, goal, manhattan=True)
            end = time.time()
            greedy_cost = greedy_end.cost
            end_dict[greedy_end] = 'Greedy, cost=' + str(greedy_cost)
            if greedy_end.current != goal:
                print('Informed Greedy Search using found the wrong goal')
                greedy_time = None
                greedy_cost = None
            else:
                greedy_time = end - start
                print('Informed Greedy Search using took: ' + str(greedy_time) + ' seconds, and found a path cost of ' + str(
                    greedy_cost) + ' after exploring ' + str(greedy_length) + ' nodes')

            algorithm_performance_dict['Greedy'] = greedy_time, greedy_cost, greedy_length
        else:
            start = time.time()
            greedy_end, greedy_length = greedy_search(starting_vertex, goal)
            end = time.time()
            greedy_cost = greedy_end.cost
            end_dict[greedy_end] = 'Greedy, cost=' + str(greedy_cost)
            if greedy_end.current != goal:
                print('Informed Greedy Search using found the wrong goal')
                greedy_time = None
                greedy_cost = None
            else:
                greedy_time = end - start
                print('Informed Greedy Search using took: ' + str(greedy_time) + ' seconds, and found a path cost of ' + str(
                    greedy_cost) + ' after exploring ' + str(greedy_length) + ' nodes')

            algorithm_performance_dict['Greedy'] = greedy_time, greedy_cost, greedy_length

        '''
        Tests A* algorithm using specified heuristic
        '''
        if manhattan and euclidean:
            start = time.time()
            a_star_end, a_star_length = a_star(starting_vertex, goal, manhattan=True)
            end = time.time()
            a_star_cost = a_star_end.cost
            end_dict[a_star_end] = 'A* Manhattan, cost=' + str(a_star_cost)
            if a_star_end.current != goal:
                print('Informed A* using Manhattan distance found the wrong goal')
                a_star_time = None
                a_star_cost = None
            else:
                a_star_time = end - start
                print('Informed A* using Manhattan distance took: ' + str(a_star_time) + ' seconds, and found a path cost of ' + str(
                    a_star_cost) + ' after exploring ' + str(a_star_length) + ' nodes')

            algorithm_performance_dict['A Star Manhattan'] = a_star_time, a_star_cost, a_star_length

            start = time.time()
            a_star_end, a_star_length = a_star(starting_vertex, goal)
            end = time.time()
            a_star_cost = a_star_end.cost
            end_dict[a_star_end] = 'A* Euclidean, cost=' + str(a_star_cost)
            if a_star_end.current != goal:
                print('Informed A* using Euclidean distance found the wrong goal')
                a_star_time = None
                a_star_cost = None
            else:
                a_star_time = end - start
                print('Informed A* using Euclidean distance took: ' + str(a_star_time) + ' seconds, and found a path cost of ' + str(
                    a_star_cost) + ' after exploring ' + str(a_star_length) + ' nodes')

            algorithm_performance_dict['A Star Euclidean'] = a_star_time, a_star_cost, a_star_length
        elif euclidean:
            start = time.time()
            a_star_end, a_star_length = a_star(starting_vertex, goal)
            end = time.time()
            a_star_cost = a_star_end.cost
            end_dict[a_star_end] = 'A*, cost=' + str(a_star_cost)
            if a_star_end.current != goal:
                print('Informed A* found the wrong goal')
                a_star_time = None
                a_star_cost = None
            else:
                a_star_time = end - start
                print('Informed A* took: ' + str(a_star_time) + ' seconds, and found a path cost of ' + str(
                    a_star_cost) + ' after exploring ' + str(a_star_length) + ' nodes')

            algorithm_performance_dict['A Star'] = a_star_time, a_star_cost, a_star_length
        elif manhattan:
            start = time.time()
            a_star_end, a_star_length = a_star(starting_vertex, goal, manhattan=True)
            end = time.time()
            a_star_cost = a_star_end.cost
            end_dict[a_star_end] = 'A*, cost=' + str(a_star_cost)
            if a_star_end.current != goal:
                print('Informed A* found the wrong goal')
                a_star_time = None
                a_star_cost = None
            else:
                a_star_time = end - start
                print('Informed A* took: ' + str(a_star_time) + ' seconds, and found a path cost of ' + str(
                    a_star_cost) + ' after exploring ' + str(a_star_length) + ' nodes')

            algorithm_performance_dict['A Star'] = a_star_time, a_star_cost, a_star_length
        else:
            start = time.time()
            a_star_end, a_star_length = a_star(starting_vertex, goal)
            end = time.time()
            a_star_cost = a_star_end.cost
            end_dict[a_star_end] = 'A*, cost=' + str(a_star_cost)
            if a_star_end.current != goal:
                print('Informed A* found the wrong goal')
                a_star_time = None
                a_star_cost = None
            else:
                a_star_time = end - start
                print('Informed A* took: ' + str(a_star_time) + ' seconds, and found a path cost of ' + str(
                    a_star_cost) + ' after exploring ' + str(a_star_length) + ' nodes')

            algorithm_performance_dict['A Star'] = a_star_time, a_star_cost, a_star_length

        '''
        Tests IDA* algorithm using specified heuristic, doesn't work so it will never run
        '''
        if False:
            if euclidean and manhattan:
                start = time.time()
                ida_star_end, ida_star_depth = ida_star(starting_vertex, goal, manhattan=True)
                end = time.time()
                ida_star_cost = ida_star_end.cost
                end_dict[ida_star_end] = 'IDA* Manhattan'
                if ida_star_end.current != goal:
                    print('Informed IDA* using Manhattan distance found the wrong goal')
                    ida_star_time = None
                    ida_star_cost = None
                else:
                    ida_star_time = end - start
                    print('Informed IDA* using Manhattan distance took: ' + str(ida_star_time) + ' seconds, and found a path cost of ' + str(
                        ida_star_cost) + ' after exploring a depth of ' + str(ida_star_depth))

                algorithm_performance_dict['IDA Star Manhattan'] = ida_star_time, ida_star_cost, ida_star_depth

                start = time.time()
                ida_star_end, ida_star_depth = ida_star(starting_vertex, goal)
                end = time.time()
                ida_star_cost = ida_star_end.cost
                end_dict[ida_star_end] = 'IDA* Euclidean'
                if ida_star_end.current != goal:
                    print('Informed IDA* using Euclidean distance found the wrong goal')
                    ida_star_time = None
                    ida_star_cost = None
                else:
                    ida_star_time = end - start
                    print('Informed IDA* using Euclidean distance took: ' + str(ida_star_time) + ' seconds, and found a path cost of ' + str(
                        ida_star_cost) + ' after exploring a depth of ' + str(ida_star_depth))

                algorithm_performance_dict['IDA Star Euclidean'] = ida_star_time, ida_star_cost, ida_star_depth
            elif euclidean:
                start = time.time()
                ida_star_end, ida_star_depth = ida_star(starting_vertex, goal)
                end = time.time()
                ida_star_cost = ida_star_end.cost
                end_dict[ida_star_end] = 'IDA*'
                if ida_star_end.current != goal:
                    print('Informed IDA* found the wrong goal')
                    ida_star_time = None
                    ida_star_cost = None
                else:
                    ida_star_time = end - start
                    print('Informed IDA* took: ' + str(ida_star_time) + ' seconds, and found a path cost of ' + str(
                        ida_star_cost) + ' after exploring a depth of ' + str(ida_star_depth))

                algorithm_performance_dict['IDA Star'] = ida_star_time, ida_star_cost, ida_star_depth
            elif manhattan:
                start = time.time()
                ida_star_end, ida_star_depth = ida_star(starting_vertex, goal, manhattan=True)
                end = time.time()
                ida_star_cost = ida_star_end.cost
                end_dict[ida_star_end] = 'IDA*'
                if ida_star_end.current != goal:
                    print('Informed IDA* found the wrong goal')
                    ida_star_time = None
                    ida_star_cost = None
                else:
                    ida_star_time = end - start
                    print('Informed IDA* took: ' + str(ida_star_time) + ' seconds, and found a path cost of ' + str(
                        ida_star_cost) + ' after exploring a depth of ' + str(ida_star_depth))

                algorithm_performance_dict['IDA Star'] = ida_star_time, ida_star_cost, ida_star_depth
            else:
                start = time.time()
                ida_star_end, ida_star_depth = ida_star(starting_vertex, goal)
                end = time.time()
                ida_star_cost = ida_star_end.cost
                end_dict[ida_star_end] = 'IDA*'
                if ida_star_end.current != goal:
                    print('Informed IDA* found the wrong goal')
                    ida_star_time = None
                    ida_star_cost = None
                else:
                    ida_star_time = end - start
                    print('Informed IDA* took: ' + str(ida_star_time) + ' seconds, and found a path cost of ' + str(
                        ida_star_cost) + ' after exploring a depth of ' + str(ida_star_depth))

                algorithm_performance_dict['IDA Star'] = ida_star_time, ida_star_cost, ida_star_depth

        '''
        Tests Beam algorithm
        '''
        if test_beam:
            for j in range(1, 5):
                start = time.time()
                beam_end, beam_length = beam_search(starting_vertex, goal, j)
                end = time.time()
                beam_cost = beam_end.cost
                end_dict[beam_end] = 'Beam, length=' + str(j) + ', cost=' + str(beam_cost)
                if beam_end.current != goal:
                    print('Informed Beam Search (beam length=' + str(j) + ') found the wrong goal')
                    beam_time = None
                    beam_cost = None
                else:
                    beam_time = end - start
                    print('Informed Beam Search (beam length=' + str(j) + ') took: ' + str(
                        beam_time) + ' seconds, and found a path cost of ' + str(
                        beam_cost) + ' after exploring ' + str(beam_length) + ' nodes')
                algorithm_performance_dict['Beam=' + str(j)] = beam_time, beam_cost, beam_length

        else:
            if manhattan and euclidean:
                start = time.time()
                beam_end, beam_length = beam_search(starting_vertex, goal, beam_size, manhattan=True)
                end = time.time()
                beam_cost = beam_end.cost
                end_dict[beam_end] = 'Beam Manhattan, cost=' + str(beam_cost)
                if beam_end.current != goal:
                    print('Informed Beam Search using Manhattan distance found the wrong goal')
                    beam_time = None
                    beam_cost = None
                else:
                    beam_time = end - start
                    print('Informed Beam Search using Manhattan distance (beam length=' + str(beam_size) + ') took: ' + str(
                        beam_time) + ' seconds, and found a path cost of ' + str(
                        beam_cost) + ' after exploring ' + str(beam_length) + ' nodes')

                algorithm_performance_dict['Beam Manhattan'] = beam_time, beam_cost, beam_length

                start = time.time()
                beam_end, beam_length = beam_search(starting_vertex, goal, beam_size)
                end = time.time()
                beam_cost = beam_end.cost
                end_dict[beam_end] = 'Beam Euclidean, cost=' + str(beam_cost)
                if beam_end.current != goal:
                    print('Informed Beam Search using Euclidean distance found the wrong goal')
                    beam_time = None
                    beam_cost = None
                else:
                    beam_time = end - start
                    print('Informed Beam Search using Euclidean (beam length=' + str(beam_size) + ') took: ' + str(
                        beam_time) + ' seconds, and found a path cost of ' + str(
                        beam_cost) + ' after exploring ' + str(beam_length) + ' nodes')

                algorithm_performance_dict['Beam Euclidean'] = beam_time, beam_cost, beam_length
            elif euclidean:
                start = time.time()
                beam_end, beam_length = beam_search(starting_vertex, goal, beam_size)
                end = time.time()
                beam_cost = beam_end.cost
                end_dict[beam_end] = 'Beam, cost=' + str(beam_cost)
                if beam_end.current != goal:
                    print('Informed Beam Search found the wrong goal')
                    beam_time = None
                    beam_cost = None
                else:
                    beam_time = end - start
                    print('Informed Beam Search (beam length=' + str(beam_size) + ') took: ' + str(
                        beam_time) + ' seconds, and found a path cost of ' + str(
                        beam_cost) + ' after exploring ' + str(beam_length) + ' nodes')

                algorithm_performance_dict['Beam'] = beam_time, beam_cost, beam_length
            elif manhattan:
                start = time.time()
                beam_end, beam_length = beam_search(starting_vertex, goal, beam_size, manhattan=True)
                end = time.time()
                beam_cost = beam_end.cost
                end_dict[beam_end] = 'Beam, cost=' + str(beam_cost)
                if beam_end.current != goal:
                    print('Informed Beam Search found the wrong goal')
                    beam_time = None
                    beam_cost = None
                else:
                    beam_time = end - start
                    print('Informed Beam Search (beam length=' + str(beam_size) + ') took: ' + str(
                        beam_time) + ' seconds, and found a path cost of ' + str(
                        beam_cost) + ' after exploring ' + str(beam_length) + ' nodes')

                algorithm_performance_dict['Beam'] = beam_time, beam_cost, beam_length
            else:
                start = time.time()
                beam_end, beam_length = beam_search(starting_vertex, goal, beam_size)
                end = time.time()
                beam_cost = beam_end.cost
                end_dict[beam_end] = 'Beam, cost=' + str(beam_cost)
                if beam_end.current != goal:
                    print('Informed Beam Search found the wrong goal')
                    beam_time = None
                    beam_cost = None
                else:
                    beam_time = end - start
                    print('Informed Beam Search (beam length=' + str(beam_size) + ') took: ' + str(beam_time) + ' seconds, and found a path cost of ' + str(
                        beam_cost) + ' after exploring ' + str(beam_length) + ' nodes')

                algorithm_performance_dict['Beam'] = beam_time, beam_cost, beam_length

        '''
        Tests Dijkstra's algorithm
        '''
        start = time.time()
        di_end, di_cost, di_prev = dijkstras(source, goal, weight_matrix)
        end = time.time()
        total_nodes_analyzed = 0
        for d in di_cost:
            if math.isfinite(di_cost[d]):
                total_nodes_analyzed += 1
        di_cost = di_cost[di_end]
        end_dict[di_end, 'short'] = 'Dijkstras, cost=' + str(di_cost)
        if di_end != goal:
            print('Informed Dijkstras search found the wrong goal')
            di_time = None
            di_cost = None
        else:
            di_time = end - start
            print('Informed Dijkstras search took: ' + str(di_time) + ' seconds, and found a path cost of ' + str(
                di_cost) + ' after exploring ' + str(total_nodes_analyzed) + ' nodes')

        algorithm_performance_dict['Dijkstras'] = di_time, di_cost, total_nodes_analyzed

        if not end_at_found:
            start = time.time()
            di_end, di_cost, di_prev = dijkstras(source, goal, weight_matrix, end_at_found=end_at_found)
            end = time.time()
            total_nodes_analyzed = 0
            for d in di_cost:
                if math.isfinite(di_cost[d]):
                    total_nodes_analyzed += 1
            di_cost = di_cost[di_end]
            end_dict[di_end, 'long'] = 'Dijkstras Full, cost=' + str(di_cost)
            if di_end != goal:
                print('Informed Dijkstras search found the wrong goal')
                di_time = None
                di_cost = None
            else:
                di_time = end - start
                print('Informed Dijkstras search on full graph took: ' + str(di_time) + ' seconds, and found a path cost of ' + str(
                    di_cost) + ' after exploring ' + str(total_nodes_analyzed) + ' nodes')
            algorithm_performance_dict['Dijkstras Full'] = di_time, di_cost, total_nodes_analyzed

        '''
        Tests Bidirectinal algorithm with specified heuristics
        '''
        if manhattan and euclidean:
            start = time.time()
            bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex, manhattan=True)
            end = time.time()
            bi_cost = bi_end.cost
            end_dict[bi_end] = 'Bidirectional Manhattan, cost=' + str(bi_cost)
            if bi_end.current != goal:
                print('Informed Bidirectional Search using Manhattan distance found the wrong goal')
                bi_time = None
                bi_cost = None
            else:
                bi_time = end - start
                print(
                    'Informed Bidirectional Search using Manhattan distance took: ' + str(bi_time) + ' seconds, and found a path cost of ' + str(
                        bi_cost) + ' after exploring ' + str(bi_length) + ' nodes')

            algorithm_performance_dict['Bidirectional Manhattan'] = bi_time, bi_cost, bi_length

            start = time.time()
            bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex)
            end = time.time()
            bi_cost = bi_end.cost
            end_dict[bi_end] = 'Bidirectional Euclidean, cost=' + str(bi_cost)
            if bi_end.current != goal:
                print('Informed Bidirectional Search using Euclidean distance found the wrong goal')
                bi_time = None
                bi_cost = None
            else:
                bi_time = end - start
                print(
                    'Informed Bidirectional Search using Euclidean distance took: ' + str(bi_time) + ' seconds, and found a path cost of ' + str(
                        bi_cost) + ' after exploring ' + str(bi_length) + ' nodes')

            algorithm_performance_dict['Bidirectional Euclidean'] = bi_time, bi_cost, bi_length
        elif euclidean:
            start = time.time()
            bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex)
            end = time.time()
            bi_cost = bi_end.cost
            end_dict[bi_end] = 'Bidirectional, cost=' + str(bi_cost)
            if bi_end.current != goal:
                print('Informed Bidirectional Search found the wrong goal')
                bi_time = None
                bi_cost = None
            else:
                bi_time = end - start
                print(
                    'Informed Bidirectional Search took: ' + str(bi_time) + ' seconds, and found a path cost of ' + str(
                        bi_cost) + ' after exploring ' + str(bi_length) + ' nodes')

            algorithm_performance_dict['Bidirectional'] = bi_time, bi_cost, bi_length
        elif manhattan:
            start = time.time()
            bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex, manhattan=True)
            end = time.time()
            bi_cost = bi_end.cost
            end_dict[bi_end] = 'Bidirectional, cost=' + str(bi_cost)
            if bi_end.current != goal:
                print('Informed Bidirectional Search found the wrong goal')
                bi_time = None
                bi_cost = None
            else:
                bi_time = end - start
                print(
                    'Informed Bidirectional Search took: ' + str(bi_time) + ' seconds, and found a path cost of ' + str(
                        bi_cost) + ' after exploring ' + str(bi_length) + ' nodes')

            algorithm_performance_dict['Bidirectional'] = bi_time, bi_cost, bi_length
        else:
            start = time.time()
            bi_end, bi_length = bidirectional_search(starting_vertex, goal_vertex)
            end = time.time()
            bi_cost = bi_end.cost
            end_dict[bi_end] = 'Bidirectional, cost=' + str(bi_cost)
            if bi_end.current != goal:
                print('Informed Bidirectional Search found the wrong goal')
                bi_time = None
                bi_cost = None
            else:
                bi_time = end - start
                print('Informed Bidirectional Search took: ' + str(bi_time) + ' seconds, and found a path cost of ' + str(
                    bi_cost) + ' after exploring ' + str(bi_length) + ' nodes')

            algorithm_performance_dict['Bidirectional'] = bi_time, bi_cost, bi_length

        testing_dict[i] = algorithm_performance_dict

        if visualize: plot_path(end_dict, source, goal, di_prev=di_prev, no_weight=no_weight)

    return testing_dict

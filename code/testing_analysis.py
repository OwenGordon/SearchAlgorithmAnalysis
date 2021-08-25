"""
Author: Owen Gordon, owgordon@iu.edu
Analyzes results from testing_time and testing_visualize files
"""
from testing_visualize import broad_testing
from testing_time import specific_testing
import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
import math

def broad_testing_analysis(size, weight_factor, obstacles_percentage, num_runs, beam_size=1, test_beam=False,
                           manhattan=False, euclidean=False, end_at_found=True, no_weight=False, visualize=False, preset=0):
    """
    Analyzes broad testing
    :param size: size of board
    :param weight_factor: maximum weight of board
    :param obstacles_percentage: percentage of board as obstacles
    :param num_runs: number of runs to complete
    :param beam_size: beam size
    :param test_beam: if true, beam lengths 1-4 are tested
    :param manhattan: if true, manhattan distance is used for heuristic
    :param euclidean: if true, euclidean distance is used for heuristic
    :param end_at_found: if false, dijkstra explores entire board
    :param no_weight: if true, the board is a uniform weight
    :param visualize: if true, the paths are visualized
    :param preset: if true, a preset board setup is used
    :return: none
    """

    testing_dict = broad_testing(size, weight_factor, obstacles_percentage, num_runs, beam_size=beam_size,
                                 test_beam=test_beam, manhattan=manhattan, euclidean=euclidean,
                                 end_at_found=end_at_found, no_weight=no_weight, visualize=visualize, preset=preset)
    attributes = testing_dict.pop('Attributes')

    algo_times = dict()
    algo_costs = dict()
    algo_lengths = dict()

    for test in testing_dict:
        algo_performance = testing_dict[test]

        if test == 0:
            for algo in algo_performance:
                algo_times[algo] = [algo_performance[algo][0]]
                algo_costs[algo] = [algo_performance[algo][1]]
                algo_lengths[algo] = [algo_performance[algo][2]]
        else:
            for algo in algo_performance:
                algo_times[algo].append(algo_performance[algo][0])
                algo_costs[algo].append(algo_performance[algo][1])
                algo_lengths[algo].append(algo_performance[algo][2])

    '''
    Parses data for algorithm performance
    '''
    speeds = [[0 for x in range(len(algo_times))] for y in range(len(testing_dict))]
    for i in range(len(testing_dict)):
        j = 0
        for algo in algo_times:
            speeds[i][j] = algo_times[algo][i]
            j += 1

    costs = [[0 for x in range(len(algo_costs))] for y in range(len(testing_dict))]
    for i in range(len(testing_dict)):
        j = 0
        for algo in algo_costs:
            if algo_costs[algo][i] is None:
                algo_costs[algo][i] = math.inf
            costs[i][j] = algo_costs[algo][i]
            j += 1

    lengths = [[0 for x in range(len(algo_lengths))] for y in range(len(testing_dict))]
    for i in range(len(testing_dict)):
        j = 0
        for algo in algo_lengths:
            lengths[i][j] = algo_lengths[algo][i]
            j += 1

    '''
    Displays algorithm performance
    '''
    print()
    print('Algorithm Performance Metric (lower is better) calculated with: \n((algorithm cost - lowest cost) + number of nodes visited) * algorithm time')
    print('Attributes:', 'Size: ' + str(attributes['Size']) + ', Weight Factor: ' + str(
        attributes['Weight Factor']) + ', Obstacles: ' + str(attributes['Obstacles']))
    for i in range(len(testing_dict)):
        lowest_cost = min(costs[i])
        for algo in algo_times:
            if algo_times[algo][i] is None:
                print(algo, 'Did not finish')
            else:
                print(algo, (((10000 * (algo_costs[algo][i] - lowest_cost)) + (10 * algo_lengths[algo][i])) * (algo_times[algo][i] * .1)) / 100)
        print()


def specific_testing_analysis(min_size, max_size, step_size, weight_factor, obstacles_percentage, normalization,
                              beam_size=1, manhattan=False, euclidean=False, no_weight=False, end_at_found=True,
                              visualize=False, bfs=False, dfs=False, iddfs=False, ucs=False, greedy=False, astar=False,
                              idastar=False, beam=False, di=False, bi=False, test_beam=False):
    """
    Analyzes time/space complexity testing
    :param min_size: minimum board size
    :param max_size: maximum board size
    :param step_size: step size for iterating over sizes
    :param weight_factor: maximum board weight
    :param obstacles_percentage: percentage of board as obstacles
    :param normalization: number of trials to average data over
    :param beam_size: beam length
    :param manhattan: if true, manhattan distance is used for heuristic
    :param euclidean: if true, euclidean distance is used for heuristic
    :param no_weight: if true, board is uniform cost
    :param end_at_found: if false, dijkstra explores entire board
    :param visualize: if true, plots are visualized
    :param bfs: if true, bfs is executed
    :param dfs: if true, dfs is executed
    :param iddfs: if true, iddfs is executed
    :param ucs: if true, ucs is executed
    :param greedy: if true, greedy is executed
    :param astar: if true, a* is executed
    :param idastar: if true, ida* is executed
    :param beam: if true, beam is executed
    :param di: if true, dijkstras is executed
    :param bi: if true, bidirectional is executed
    :param test_beam: if true, all beam lengths are tested
    :return: none
    """
    testing_dict = specific_testing(min_size, max_size, step_size, weight_factor, obstacles_percentage, normalization,
                                    beam_size=beam_size, manhattan=manhattan, euclidean=euclidean, no_weight=no_weight,
                                    end_at_found=end_at_found, bfs=bfs, dfs=dfs, iddfs=iddfs, ucs=ucs, greedy=greedy,
                                    astar=astar, idastar=idastar, beam=beam, di=di, bi=bi, test_beam=test_beam)

    if no_weight:
        weight_factor = 0

    if visualize:
        plt.figure(1)

        # plots time complexity
        plt.title('Time vs. Size, Weight Factor=' + str(weight_factor) + '\nPercentage of board as obstacles=' + str(obstacles_percentage) +'%, normalization=' + str(normalization))
        plt.ylabel('Time (s)')
        plt.xlabel('Graph Size (nxn)')
        for algo in testing_dict:
            sizes = list()
            times = list()
            for size in testing_dict[algo]:
                sizes.append(size)
                time_total = 0
                num_times = 0
                for time in testing_dict[algo][size][0]:
                    if time:
                        time_total += time
                        num_times += 1

                if num_times == 0:
                    times.append(math.inf)
                else:
                    times.append(time_total / num_times)

            f = interpolate.interp1d(sizes, times)
            x = np.linspace(min(sizes), max(sizes), 1000)

            plt.plot(x, f(x), '-', label=algo)

        plt.legend()

        plt.figure(2)

        # plots space complexity
        plt.title('Nodes Expanded vs. Size, Weight Factor=' + str(weight_factor) + '\nPercentage of board as obstacles=' + str(
            obstacles_percentage) + '%, normalization=' + str(normalization))
        plt.ylabel('Nodes')
        plt.xlabel('Graph Size (nxn)')
        for algo in testing_dict:
            sizes = list()
            lengths = list()
            for size in testing_dict[algo]:
                sizes.append(size)
                length_total = 0
                num_times = 0
                for length in testing_dict[algo][size][1]:
                    if length:
                        length_total += length
                        num_times += 1

                if num_times == 0:
                    lengths.append(math.inf)
                else:
                    lengths.append(length_total / num_times)

            f = interpolate.interp1d(sizes, lengths)
            x = np.linspace(min(sizes), max(sizes), 1000)

            plt.plot(x, f(x), '-', label=algo)

        plt.legend()

    averages = dict()
    for algo in testing_dict:
        for size in testing_dict[algo]:
            if size not in averages.keys():
                averages[size] = dict()

            if algo not in averages[size].keys():
                averages[size][algo] = list()

            time_total = 0
            num_times = 0
            for time in testing_dict[algo][size][0]:
                if time:
                    time_total += time
                    num_times += 1

            if num_times != 0:
                averages[size][algo].append(time_total / num_times)
            else:
                averages[size][algo].append(math.inf)

            length_total = 0
            num_times = 0
            for length in testing_dict[algo][size][1]:
                if length:
                    length_total += length
                    num_times += 1
            if num_times != 0:
                averages[size][algo].append(length_total / num_times)
            else:
                averages[size][algo].append(math.inf)

            cost_total = 0
            num_times = 0
            for cost in testing_dict[algo][size][2]:
                if cost:
                    cost_total += cost
                    num_times += 1
            if num_times != 0:
                averages[size][algo].append(cost_total / num_times)
            else:
                averages[size][algo].append(math.inf)

    average_times = dict()
    average_lengths = dict()
    average_costs = dict()

    for size in averages:
        for algo in averages[size]:
            if size not in average_times.keys():
                average_times[size] = list()
                average_lengths[size] = list()
                average_costs[size] = list()

            average_times[size].append(averages[size][algo][0])
            average_lengths[size].append(averages[size][algo][1])
            average_costs[size].append(averages[size][algo][2])

    performance_dict = dict()
    for size in averages:
        lowest_cost = min(average_costs[size])
        for algo in averages[size]:
            if algo not in performance_dict.keys():
                performance_dict[algo] = dict()

            performance_dict[algo][size] = (10000 * (averages[size][algo][2] - lowest_cost) + (10 * averages[size][algo][1]) * (averages[size][algo][0] * .1) / 100)

    if visualize:
        plt.figure(3)

        # plots algorithm score
        plt.title('Algorithm Score vs. Size\nScore=((algorithm cost-lowest cost)+number of nodes visited)*algorithm time')
        plt.ylabel('Score')
        plt.xlabel('Graph Size (nxn)')

        for algo in performance_dict:
            sizes = list()
            scores = list()
            for size in performance_dict[algo]:
                sizes.append(size)
                scores.append(performance_dict[algo][size])

            f = interpolate.interp1d(sizes, scores)
            x = np.linspace(min(sizes), max(sizes))

            plt.plot(x, f(x), '-', label=algo)

        plt.legend()
        plt.show(block=False)

    # don't need to save data files any more
    # data_file = open('Data/' + str(min_size) + '_' + str(max_size) + '_' + str(step_size) + '_' + str(weight_factor) + '_' + str(obstacles_percentage) + '_' + str(normalization) + '_' + str(len(algos)) + '.csv', 'w')
    # for algo in testing_dict:
    #     data_file.write(algo + '\n')
    #     for size in testing_dict[algo]:
    #         data_file.write('Size,' + str(size) + ',\n')
    #         data_file.write('Time,Length,Cost,Score,\n')
    #         for i in range(3):
    #             data_file.write(str(averages[size][algo][i]) + ',')
    #         data_file.write(str(performance_dict[algo][size]) + ',')
    #         data_file.write('\n')
    #         for i in range(len(testing_dict[algo][size][0])):
    #             for j in range(len(testing_dict[algo][size])):
    #                 if testing_dict[algo][size][j][i]:
    #                     data_file.write(str(testing_dict[algo][size][j][i]) + ',')
    #             data_file.write('\n')
    #         data_file.write('\n')
    #
    # data_file.close()


'''
These are the two ways we can test data so far.
The first one can display the paths, or just show time and cost and number of nodes visited
The arguments are (size, max weight, obstacle%, beam length, number of full runs, plot paths)
The arguments for the second one are (min size, max size, step size, max weight, obstacle%, beam length, 
    number of iterations to average across, each algorithm turned on or off)
'''
# broad_testing_analysis(97, 13, 12, 1, manhattan=True, end_at_found=False, visualize=True, beam_size=3, test_beam=True)
# specific_testing_analysis(10, 100, 15, 30, 10, 50, end_at_found=False, euclidean=True, manhattan=True, di=True, visualize=True, dfs=True, bfs=True, ucs=True, greedy=True, astar=True, beam=True, bi=True, test_beam=True)
# specific_testing_analysis(40, 100, 15, 30, 10, 5, beam=True, test_beam=True, visualize=True)

'''
Handles logic for user interface for testing
'''
finished = False

while not finished:
    path = input('Choose which path you would like to simulate:'
                 '\n\t1. Test every algorithm against the same board of a fixed size'
                 '\n\t2. Test specific algorithms and see how they perform with growing sizes of boards'
                 '\n\t3. Test a preset graph with interesting results'
                 '\nEnter 1, 2 or 3: ')

    path = int(path)

    if path == 1:
        size = input('Enter a board size for your NxN board, below 10 is small, 10-100 is medium, 100+ is large\nEnter a number: ')
        try:
            size = int(size)
            print('Your choice for size is ' + str(size))
        except ValueError:
            print('Not a number, using size of 100')
            size = 100

        weight_factor = input('Enter the maximum node cost for your board, or enter 0 for a uniform cost board\nEnter a number: ')
        try:
            weight_factor = int(weight_factor)
        except ValueError:
            print('Not a number, using maximum cost of 20')
            weight_factor = 20
        no_weight = False
        if weight_factor == 0:
            no_weight = True
            print('You chose a uniform weight board')
        else:
            print('Your choice for maximum cost is ' + str(weight_factor))

        obstacle_percentage = input('Enter the percentage of obstacles on your board, 3 is a small amount, 3-10 is medium medium amount,\n10+ is a large amount, and there may be no path to the goal\nEnter a number: ')
        try:
            obstacle_percentage = int(obstacle_percentage)
            print('Your choice for obstacle percentage is ' + str(obstacle_percentage))
        except ValueError:
            print('Not a number, using obstacle percentage of 10%')
            obstacle_percentage = 10

        distance = input('What type of distance calculation would you like to use:\n\t1. Euclidean Distance\n\t2. Manhattan Distance\n\t3. Both\nEnter 1, 2, or 3: ')
        try:
            distance = int(distance)
        except ValueError:
            print('Not a number, using euclidean distance')
            distance = 1
        euclidean = False
        manhattan = False
        if distance == 1:
            euclidean = True
            print('Euclidean distance chosen for heuristic calculation')
        elif distance == 2:
            manhattan = True
            print('Manhattan distance chosen for heuristic calculation')
        elif distance == 3:
            euclidean = True
            manhattan = True
            print('Euclidean and Manhattan distance chosen for heuristic calculation')

        di_choice = input('Would you like Dijkstras algorithm to terminate when a goal is found?\nEnter \'y\' for yes or \'n\' for no: ')
        end_at_found = True
        if di_choice == 'n':
            end_at_found = False
            print('Dijkstras algorithm will search the whole board')
        else:
            print('Dijkstras algorithm will only search until the goal is found')

        beam_choice = input('What beam length would you like to use? Enter a number 1 - 4 for a length, or 0 to test all lengths\nEnter a number: ')
        try:
            beam_choice = int(beam_choice)
        except ValueError:
            print('Not a number, using beam length of 3')
            beam_choice = 3
        test_beam = False
        if beam_choice == 0:
            test_beam = True
            print('All beam lengths will be tested')
        else:
            print('Your choice for beam length is ' + str(beam_choice))

        v_choice = input('Would you like to visualize the algorithm paths?\nEnter \'y\' for yes or \'n\' for no: ')
        visualize = True
        if v_choice == 'n':
            visualize = False
            print('You chose to not visualize the paths')
        else:
            print('You chose to visualize the path')

        broad_testing_analysis(size, weight_factor, obstacle_percentage, 1, manhattan=manhattan, euclidean=euclidean,
                               end_at_found=end_at_found, no_weight=no_weight, test_beam=test_beam, visualize=visualize,
                               beam_size=beam_choice)
    elif path == 2:
        min_size = input('Enter a minimum board size for your trials, 5 or 10 are recommended starting sizes\nEnter a number: ')
        try:
            min_size = int(min_size)
            print('Your choice for size is ' + str(min_size))
        except ValueError:
            print('Not a number, using min size of 10')
            min_size = 10

        max_size = input('Enter a max board size for trials, 75, 100, 150, and 200 are recommended ending sizes\nEnter a number: ')
        try:
            max_size = int(max_size)
            print('Your choice for size is ' + str(max_size))
        except ValueError:
            print('Not a number, using max size of 100')
            max_size = 100

        step_size = input('Enter a step size for incrementing the board size, 1, 2, 5, 10, 15 are good choices\nThe smaller the step, the more accurate the function will be\nEnter a number: ')
        try:
            step_size = int(step_size)
            print('Your choice for step size is ' + str(step_size))
        except ValueError:
            print('Not a number, using step size of 2')
            step_size = 2

        weight_factor = input('Enter the maximum node cost for your board, or enter 0 for a uniform cost board\nEnter a number: ')
        try:
            weight_factor = int(weight_factor)
        except ValueError:
            print('Not a number, using maximum cost of 20')
            weight_factor = 20
        no_weight = False
        if weight_factor == 0:
            no_weight = True
            print('You chose a uniform weight board')
        else:
            print('Your choice for maximum cost is ' + str(weight_factor))

        obstacle_percentage = input('Enter the percentage of obstacles on your board, 3 is a small anount, 3-10 is medium medium amount,\n10+ is a large amount, and there may be no path to the goal\nEnter a number: ')
        try:
            obstacle_percentage = int(obstacle_percentage)
            print('Your choice for obstacle percentage is ' + str(obstacle_percentage))
        except ValueError:
            print('Not a number, using obstacle percentage of 10%')
            obstacle_percentage = 10

        normalization = input('Choose the number of trials you would like to complete to average over, 5, 10, 20, 50 are solid benchmarks, \nThe higher the number, the more accurate the function at the end\nEnter a number: ')
        try:
            normalization = int(normalization)
            print('Your choice the number of trials is ' + str(normalization))
        except ValueError:
            print('Not a number, using 50 trials')
            normalization = 50

        distance = input('What type of distance calculation would you like to use:\n\t1. Euclidean Distance\n\t2. Manhattan Distance\n\t3. Both\nEnter 1, 2, or 3: ')
        try:
            distance = int(distance)
        except ValueError:
            print('Not a number, using euclidean distance')
            distance = 1
        euclidean = False
        manhattan = False
        if distance == 1:
            euclidean = True
            print('Euclidean distance chosen for heuristic calculation')
        elif distance == 2:
            manhattan = True
            print('Manhattan distance chosen for heuristic calculation')
        elif distance == 3:
            euclidean = True
            manhattan = True
            print('Euclidean and Manhattan distance chosen for heuristic calculation')


        print('Now select the algorithms you would like to test\nYou will be presented with an algorithm, enter \'y\' to include algorithm in test or \'n\' to exlude algorithm')
        bfs_choice = input('BFS: ')
        bfs = False
        if bfs_choice == 'y':
            bfs = True
        dfs_choice = input('DFS: ')
        dfs = False
        if dfs_choice == 'y':
            dfs = True
        iddfs = False
        if min_size < 10:
            iddfs_choice = input('IDDFS: ')
            if iddfs_choice == 'y':
                iddfs = True
        ucs_choice = input('UCS: ')
        ucs = False
        if ucs_choice == 'y':
            ucs = True
        greedy_choice = input('Greedy: ')
        greedy = False
        if greedy_choice == 'y':
            greedy = True
        a_star_choice = input('A*: ')
        astar = False
        if a_star_choice == 'y':
            astar = True
        beam_choice = input('Beam: ')
        beam = False
        test_beam = False
        if beam_choice == 'y':
            beam_choice = input(
                'What beam length would you like to use? Enter a number 1 - 4 for a length, or 0 to test all lengths\nEnter a number: ')
            try:
                beam_choice = int(beam_choice)
            except ValueError:
                print('Not a number, using beam length of 3')
                beam_choice = 3
            test_beam = False
            if beam_choice == 0:
                test_beam = True
            beam = True
        di_choice = input('Dijkstra: ')
        di = False
        end_at_found = True
        if di_choice == 'y':
            di_choice = input(
                'Would you like Djikstras algorithm to terminate when a goal is found?\nEnter \'y\' or \'n\': ')
            if di_choice == 'n':
                end_at_found = False
            di = True
        bi_choice = input('Bidirectional: ')
        bi = False
        if bi_choice == 'y':
            bi = True

        v_choice = input('Would you like to visualize performance at the end?\nEnter \'y\' or \'n\': ')
        visualize = True
        if v_choice == 'n':
            visualize = False
            print('You chose to not visualize the paths')
        else:
            print('You chose to visualize the path')

        specific_testing_analysis(min_size, max_size, step_size, weight_factor, obstacle_percentage, normalization, no_weight=no_weight, end_at_found=end_at_found, euclidean=euclidean, manhattan=manhattan,
                                  visualize=visualize, beam_size=beam_choice, test_beam=test_beam, bfs=bfs, dfs=dfs, iddfs=iddfs, ucs=ucs, greedy=greedy, astar=astar, beam=beam, di=di, bi=bi)
    elif path == 3:
        preset = input('Choose a preset option: '
                       '\n\t1. Bottom Left to Top Right'
                       '\nEnter a number: ')
        try:
            preset = int(preset)
        except ValueError:
            print('Not a number, preset 1 chosen')
            preset = 1
        if preset == 1:
            print('Testing Bottom left to top right path')
            broad_testing_analysis(100, 20, 7, 1, manhattan=True, euclidean=True, end_at_found=False, test_beam=True,
                               visualize=True, beam_size=3, preset=preset)
        else:
            print('Invalid option')
    else:
        print('Invalid selection')

    choice = input('Enter c to continue, otherwise enter q to quit: ')
    if choice == 'q':
        finished = True

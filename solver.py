#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math


def compute_euclidean_distance_matrix(locations):
    """Creates callback to return distance between points."""
    # coord_0, coord_1 = locations[node_0], locations[node_1]

    # return math.hypot((coord_0[1] - coord_1[1]),(coord_0[0] - coord_1[0]))
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = (
                    math.hypot((from_node[0] - to_node[0]),
                                (from_node[1] - to_node[1])))
    return distances

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    parts = lines[0].split()
 
    data = {}
    data['num_cust'] = int(parts[0])
    data['num_vehicles'] = int(parts[1])
    data['vehicle_capacities'] = [int(parts[2])] * int(parts[1])
    data['depot'] = 0
    
    demand = []
    points = []
    for i in range(1, data['num_cust']+1):
        line = lines[i]
        parts = line.split()
        demand.append(int(parts[0]))
        points.append([float(parts[1]), float(parts[2])])
    
    data['demands'] = demand
    data['distance_matrix'] = compute_euclidean_distance_matrix(points)
    
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                            data['num_vehicles'], data['depot'])
    
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    
    
    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]*1000
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    
    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    
    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]
    
    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')
    
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    
    # Solve the problem.
    search_parameters.local_search_metaheuristic = (
    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.seconds = 300
    search_parameters.log_search = True
    
    solution = routing.SolveWithParameters(search_parameters)
    
    
    # Print solution on console.
    total_distance = 0
    total_load = 0
    route1 = dict()
    
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route = []
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            route.append(previous_index)
            index = solution.Value(routing.NextVar(index))
            
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            
        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
                                                  route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        route1['%s' % vehicle_id] = route
        route1['%s' % vehicle_id][0] = 0
        route1['%s' % vehicle_id].append(0)
        print(plan_output)
        total_distance += route_distance
        total_load += route_load
    print('Total distance of all routes: {}m'.format(total_distance))
    print('Total load of all routes: {}'.format(total_load))
    
    total_distance = total_distance/1000
    
    outputData = '%.2f' % total_distance + ' ' + str(0) + '\n'
    for v in range(0, data['num_vehicles']):
        outputData += ' '.join(map(str, route1[str(v)]))+'\n'
 
    return outputData    


import sys

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:

        print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/vrp_5_4_1)')


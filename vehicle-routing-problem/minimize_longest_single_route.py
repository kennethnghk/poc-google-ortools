# https://developers.google.com/optimization/routing/vrp?hl=zh-TW
from __future__ import print_function
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

from matplotlib import pyplot as plt
from scipy import spatial
import numpy as np

def get_distance(x1, x2):
    # return spatial.distance.euclidean(x1, x2)
    return abs(x1[0] - x2[0]) + abs(x1[1]-x2[1])

def get_all_distances(locations):
    size = len(locations)
    all_distances = [[]] * size
    for i in range(size):
        distances_row = [0] * size
        for j in range(size):
            distance = 0
            if i != j:
                distance = get_distance(locations[i], locations[j])
            distances_row[j] = distance
        all_distances[i] = distances_row
    return all_distances

def create_data_model(locations):
    data = {}
    distances = get_all_distances(locations)
    data["distances"] = distances
    data["num_locations"] = len(locations)
    data["num_vehicles"] = 10
    data["depot"] = 0
    return data

def create_distance_callback(data):
  distances = data["distances"]

  def distance_callback(from_node, to_node):
    return distances[from_node][to_node]
  return distance_callback

def add_distance_dimension(routing, distance_callback):
    distance = 'Distance'
    maximum_distance = 3000  # Maximum distance per vehicle.
    routing.AddDimension(
        distance_callback,
        0,  # null slack
        maximum_distance,
        True,  # start cumul to zero
        distance
    )
  
    distance_dimension = routing.GetDimensionOrDie(distance)
    # Try to minimize the max distance among vehicles.
    distance_dimension.SetGlobalSpanCostCoefficient(100)

def print_solution(data, routing, assignment):
    total_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_dist = 0
        while not routing.IsEnd(index):
            node_index = routing.IndexToNode(index)
            next_node_index = routing.IndexToNode(
                assignment.Value(routing.NextVar(index)))
            route_dist += routing.GetArcCostForVehicle(node_index, next_node_index, vehicle_id)
            plan_output += ' {0} ->'.format(node_index)
            index = assignment.Value(routing.NextVar(index))
        plan_output += ' {}\n'.format(routing.IndexToNode(index))
        plan_output += 'Distance of route: {}m\n'.format(route_dist)
        print(plan_output)
        total_distance += route_dist
    print('Total distance of all routes: {}m'.format(total_distance))

def main(locations):
    data = create_data_model(locations)

    routing = pywrapcp.RoutingModel(
        data["num_locations"], 
        data["num_vehicles"], 
        data["depot"]
    )

    # Define weight of each edge
    distance_callback = create_distance_callback(data)
    routing.SetArcCostEvaluatorOfAllVehicles(distance_callback)
    add_distance_dimension(routing, distance_callback)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC) # pylint: disable=no-member

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
        print_solution(data, routing, assignment)

locations = np.array([
    [456, 320], # location 0
    [228, 0],    # location 1
    [912, 0],    # location 2
    [0, 80],     # location 3
    [114, 80],   # location 4
    [570, 160],  # location 5
    [798, 160],  # location 6
    [342, 240],  # location 7
    [684, 240],  # location 8
    [570, 400],  # location 9
    [912, 400],  # location 10
    [114, 480],  # location 11
    [228, 480],  # location 12
    [342, 560],  # location 13
    [684, 560],  # location 14
    [0, 640],    # location 15
    [798, 640]   # location 16
])

main(locations)

# plt.scatter(locations[:,0], locations[:,1])
# plt.show()
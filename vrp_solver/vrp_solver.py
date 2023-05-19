from functools import partial
from ortools.constraint_solver import routing_enums_pb2, routing_parameters_pb2
from ortools.constraint_solver import pywrapcp


class DataModel:
    def __init__(self, locations, demands, num_vehicles, vehicle_capacity, depot):
        self.locations = locations
        self.num_locations = len(self.locations)
        self.demands = demands
        self.num_vehicles = num_vehicles
        self.vehicle_capacity = vehicle_capacity
        self.depot = depot

    def get_data_model(self):
        data = {}

        data['locations'] = self.locations
        data['num_locations'] = self.num_locations
        data['demands'] = self.demands
        data['num_vehicles'] = self.num_vehicles
        data['vehicle_capacity'] = self.vehicle_capacity
        data['depot'] = self.depot

        print(data)

        return data

    def update_location(self, new_locations):
        self.locations = new_locations
        self.num_locations = len(self.locations)

    def update_demands(self, new_demands):
        self.demands = new_demands

    def update_num_vehicles(self, new_num_vehicles):
        self.num_vehicles = new_num_vehicles

    def update_vehicle_capatity(self, new_vehicle_capacity):
        self.vehicle_capacity = new_vehicle_capacity


class ConstraintVehicleRouting:
    def __init__(self, data: DataModel):
        self.data = data.get_data_model()
        self.manager = pywrapcp.RoutingIndexManager(self.data['num_locations'],
                                                    self.data['num_vehicles'],
                                                    self.data['depot'])
        self.routing = pywrapcp.RoutingModel(self.manager)

    def manhattan_distance(position_1: tuple, position_2: tuple):
        return abs(position_1[0] - position_2[0]) + abs(position_1[1] - position_2[1])

    def create_distance_evaluator(self):
        _distance = {}

        for from_node in range(self.data['num_locations']):
            _distance[from_node] = {}
            for to_node in range(self.data['num_locations']):
                if from_node == to_node:
                    _distance[from_node][to_node] = 0
                else:
                    # _distance[from_node][to_node] = self.manhattan_distance(self.data['locations'][from_node], self.data['locations'][to_node])
                    _distance[from_node][to_node] = abs(self.data['locations'][from_node][0] - self.data['locations'][to_node][0]) + abs(
                        self.data['locations'][from_node][1] - self.data['locations'][to_node][1])

        def distance_evaluator(manager: pywrapcp.RoutingIndexManager, from_node, to_node):
            return _distance[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

        return distance_evaluator

    def create_demand_evaluator(self):
        _demands = self.data['demands']

        def demand_evaluator(manager: pywrapcp.RoutingIndexManager, node):
            return _demands[manager.IndexToNode(node)]

        return demand_evaluator

    def add_capacity_constraints(self,  demand_evaluator_index):

        capacity = 'Capacity'

        self.routing.AddDimension(
            demand_evaluator_index,
            0,
            self.data['vehicle_capacity'],
            True,
            capacity
        )

    def print_solution(self, assignment: pywrapcp.Assignment):
        print(f'Objective: {assignment.ObjectiveValue()}')
        capacity_dimension: pywrapcp.RoutingDimension
        capacity_dimension = self.routing.GetDimensionOrDie('Capacity')
        total_distance = 0
        total_load = 0

        for vehicle_id in range(self.manager.GetNumberOfVehicles()):
            index = self.routing.Start(vehicle_id)
            plan_output = f'Route for vehicle {vehicle_id}:\n'
            distance = 0
            while not self.routing.IsEnd(index):
                load_var = capacity_dimension.CumulVar(index)

                plan_output += f'{self.manager.IndexToNode(index)} Load({assignment.Value(load_var)}) ->'
                previous_index = index

                index = assignment.Value(self.routing.NextVar(index))
                distance += self.routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)

            load_var = capacity_dimension.CumulVar(index)

            plan_output += f'{self.manager.IndexToNode(index)} Load ({assignment.Value(load_var)})\n'

            plan_output += f'Distance of the route: {distance} m\n'

            plan_output += f'Load of the route: {assignment.Value(load_var)}'

            print(plan_output)

            total_distance += distance
            total_load += assignment.Value(load_var)

        print(f'Total Distance of all routes: {total_distance} m')
        print(f'Total Load of all routes: {total_load}')

    def solve(self):
        distance_evaluator_index = self.routing.RegisterTransitCallback(
            partial(self.create_distance_evaluator(), self.manager))

        self.routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)

        demand_evaluator_index = self.routing.RegisterUnaryTransitCallback(
            partial(self.create_demand_evaluator(), self.manager))

        self.add_capacity_constraints(demand_evaluator_index)

        search_parameters: routing_parameters_pb2.RoutingSearchParameters

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()

        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )

        search_parameters.time_limit.FromSeconds(2)

        search_parameters.log_search = True

        solution: pywrapcp.Assignment

        solution = self.routing.SolveWithParameters(search_parameters)

        if solution:
            self.print_solution(solution)
        else:
            print('No solution found!')

from pathlib import Path
from dataclasses import dataclass
import yaml
import graphviz

from ortools.constraint_solver import routing_enums_pb2, routing_parameters_pb2
from ortools.constraint_solver import pywrapcp


@dataclass
class DataModel:
    weights: float
    demands: list
    pickups_deliveries: list
    vehicle_capacities: list
    starts: list
    ends: list
    depot: int

    @property
    def num_locations(self) -> int:
        return len(self.weights)

    @property
    def num_vehicles(self) -> int:
        return len(self.vehicle_capacities)


@dataclass(frozen=True)
class GraphVisualization:
    data: DataModel

    def draw_network_graph(self, filename: str) -> None:
        def node_label(index: int) -> int:
            if index == 0:
                return f"Node {index}\nDepot"
            if self.data.demands:
                return f"Node {index}\nDemand: {self.data.demands[index]}\n"
            return f"Node {index}\n"

        graph = graphviz.Digraph(name="network")

        num_locations = self.data.num_locations

        for i in range(num_locations):
            for j in range(i+1, num_locations):
                name_i, name_j = f"Node{i}", f"Node{j}"
                graph.node(name=name_i, label=node_label(i))
                graph.node(name=name_j, label=node_label(j))
                graph.edge(name_i, name_j, label=str(self.data.weights[i][j]))

        ext = Path(filename).suffix or ".png"
        graph.render(
            cleanup=True, format=ext[1:], outfile=filename, engine="dot")

        print(f"The network graph has been saved to {filename}.")

    def draw_route_graph(
        self,
        filename: str,
        routing: pywrapcp.RoutingModel,
        manager: pywrapcp.RoutingIndexManager,
        assignment: pywrapcp.Assignment
    ) -> None:
        def node_label(index: int) -> str:
            if index == 0:
                return f"Node {index}\nDepot"
            if self.data.demands:
                return f"Node {index}\nDemand: {self.data.demands[index]}\n"
            return f"Node {index}\n"

        graph = graphviz.Digraph(name="route")

        for vehicle_id in range(self.data.num_vehicles):
            index: int = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                node_index: int = manager.IndexToNode(index)
                next_var: pywrapcp.IntVar = routing.NextVar(index)
                next_index: int = assignment.Value(next_var)
                next_node_index: int = manager.IndexToNode(next_index)
                weight = self.data.weights[node_index][next_node_index]

                name_current, name_next = f"Node{node_index}", f"Node{next_node_index}"
                graph.node(name=name_current, label=node_label(node_index))
                graph.node(name=name_next, label=node_label(next_node_index))
                graph.edge(name_current, name_next, label=str(weight))

                index = next_index
        ext = Path(filename).suffix or ".png"
        graph.render(
            cleanup=True, format=ext[1:], outfile=filename, engine="sfdp")

        print(f"The route graph has been saved to {filename}.")


@dataclass(frozen=True)
class Node:
    index: int
    load: int


@dataclass
class ConstraintVehicleRouting:
    data: DataModel

    def print_solution(
        self,
        routing: pywrapcp.RoutingModel,
        manager: pywrapcp.RoutingIndexManager,
        assignment: pywrapcp.Assignment
    ) -> None:
        def create_node(
            manager: pywrapcp.RoutingIndexManager,
            assignment: pywrapcp.Assignment,
            capacity_dimension: pywrapcp.RoutingDimension,
            index: int
        ) -> Node:
            node_index: int = manager.IndexToNode(index)

            cap_var: pywrapcp.IntVar = capacity_dimension.CumulVar(index)
            load: int = assignment.Value(cap_var)

            return Node(node_index, load)

        capacity_dimension: pywrapcp.RoutingDimension = routing.GetDimensionOrDie(
            "Capacity")

        total_distance: float = 0.0
        total_load: float = 0.0

        for vehicle_id in range(self.data.num_vehicles):
            index: int = routing.Start(vehicle_id)
            nodes: list[Node] = []
            route_distance: float = 0.0
            route_load: float = 0.0

            while not routing.IsEnd(index):
                node = create_node(manager, assignment,
                                   capacity_dimension, index)
                nodes.append(node)

                node_index = manager.IndexToNode(index)
                route_load += self.data.demands[node_index]

                next_var: pywrapcp.IntVar = routing.NextVar(index)
                previous_index = index
                index = assignment.Value(next_var)

                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)

            node = create_node(manager, assignment, capacity_dimension, index)
            nodes.append(node)

            total_distance += route_distance
            total_load += route_load

            route = "\n ->".join(
                f"Node {node.index:2d}: Load({node.load:2d})" for node in nodes)

            print(
                f"Route for vehicle {vehicle_id}:\n   {route}\n"
                f"Distance of the route: {route_distance}(m)\n"
                f"Load of the route: {route_load}"
            )

        print(f"Total distance of all routes: {total_distance} (m)")
        print(f"Total load of all routes: {total_load}")

        dropped_nodes = "Dropped nodes: "
        for node in range(routing.Size()):
            if routing.IsStart(node) or routing.IsEnd(node):
                continue
            if assignment.Value(routing.NextVar(node)) == node:
                dropped_nodes += f" {manager.IndexToNode(node)}"
        print(dropped_nodes)

    def set_edge_weights(
        self,
        routing: pywrapcp.RoutingModel,
        manager: pywrapcp.RoutingIndexManager
    ) -> None:

        def weight_callback(from_index: int, to_index: int) -> int:

            from_node: int = manager.IndexToNode(from_index)
            to_node: int = manager.IndexToNode(to_index)

            return self.data.weights[from_node][to_node]

        weight_callback_index: int = routing.RegisterTransitCallback(
            weight_callback)

        routing.SetArcCostEvaluatorOfAllVehicles(weight_callback_index)

        routing.AddDimension(
            evaluator_index=weight_callback_index,
            slack_max=0,
            capacity=3000,
            fix_start_cumul_to_zero=True,
            name="Distance"
        )

    def add_capacity_constraints(
            self,
            routing: pywrapcp.RoutingModel,
            manager: pywrapcp.RoutingIndexManager) -> None:

        def demand_callback(from_index: int) -> int:
            from_node: int = manager.IndexToNode(from_index)
            return self.data.demands[from_node]

        demand_callback_index: int = routing.RegisterUnaryTransitCallback(
            demand_callback)

        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            slack_max=0,
            vehicle_capacities=self.data.vehicle_capacities,
            fix_start_cumul_to_zero=True,
            name="Capacity")

    def add_pickups_and_deliveries(
            self,
            routing: pywrapcp.RoutingModel,
            manager: pywrapcp.RoutingIndexManager) -> None:

        distance_dimension: pywrapcp.RoutingDimension = routing.GetDimensionOrDie(
            "Distance")
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        for request in self.data.pickups_deliveries:
            pickup_index: int = manager.IndexToNode(request[0])
            delivery_index: int = manager.IndexToNode(request[1])
            routing.AddPickupAndDelivery(pickup_index, delivery_index)
            routing.solver().Add(
                routing.VehicleVar(
                    pickup_index) == routing.VehicleVar(delivery_index)
            )
            routing.solver().Add(
                distance_dimension.CumulVar(
                    pickup_index) <= distance_dimension.CumulVar(delivery_index)
            )

    def add_penalties_and_dropping_visits(
        self,
        routing: pywrapcp.RoutingModel,
        manager: pywrapcp.RoutingIndexManager
    ) -> None:
        penalty = 1000
        for node in range(1, self.data.num_locations):
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    def solve(self) -> None:
        manager = pywrapcp.RoutingIndexManager(
            self.data.num_locations,
            self.data.num_vehicles,
            self.data.depot)
        routing = pywrapcp.RoutingModel(manager)

        self.set_edge_weights(routing, manager)

        if self.data.demands:
            self.add_capacity_constraints(routing, manager)

        if self.data.pickups_deliveries:
            self.add_pickups_and_deliveries(routing, manager)

        self.add_penalties_and_dropping_visits(routing, manager)

        search_params: routing_parameters_pb2.RoutingSearchParameters = pywrapcp.DefaultRoutingSearchParameters()

        search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

        search_params.time_limit.seconds = 2

        search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH

        search_params.log_search = True

        assignment: pywrapcp.Assignment = routing.SolveWithParameters(
            search_params)

        if not assignment:
            print("No solution found")
            return

        self.print_solution(routing, manager, assignment)

        graph = GraphVisualization(self.data)

        graph.draw_network_graph(
            "network.png")

        graph.draw_route_graph(
            "route.png", routing, manager, assignment)


def main():
    data_file = "/home/quanghuy20nd/vrpSolver_ws/src/vrp_solver/include/vrp_solver/cvrp_data.example.yaml"
    data: dict
    with open(data_file, encoding="utf-8") as file:
        data = yaml.safe_load(file)

    dt_weights: float
    dt_demands: list
    dt_pickups_deliveries: list
    dt_vehicle_capacities: list
    dt_starts: list
    dt_ends: list
    dt_depot: int

    dt_weights = data['weights'] if 'weights' in data else None

    dt_demands = data['demands'] if 'demands' in data else None

    dt_pickups_deliveries = data['pickups_deliveries'] if 'pickups_deliveries' in data else None

    dt_vehicle_capacities = data['vehicle_capacities'] if 'vehicle_capacities' in data else None

    dt_starts = data['starts'] if 'starts' in data else None

    dt_ends = data['ends'] if 'ends' in data else None

    dt_depot = data['depot'] if 'depot' in data else None

    data_model = DataModel(
        weights=dt_weights,
        demands=dt_demands,
        pickups_deliveries=dt_pickups_deliveries,
        vehicle_capacities=dt_vehicle_capacities,
        starts=dt_starts,
        ends=dt_ends,
        depot=dt_depot
    )

    cvrp = ConstraintVehicleRouting(data_model)

    cvrp.solve()


if __name__ == "__main__":
    main()

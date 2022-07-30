#include <cstdint>
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "C:\Users\tima\Downloads\or-tools_VisualStudio2022-64bit_v9.3.10497\examples\java\RandomTsp\src\main\java\com\google\protobuf\duration.pb.h"
#include "C:\Users\tima\Downloads\or-tools_VisualStudio2022-64bit_v9.3.10497\examples\java\RandomTsp\src\main\java\com\google\ortools\constraint_solver\routing.h"
#include "C:\Users\tima\Downloads\or-tools_VisualStudio2022-64bit_v9.3.10497\examples\java\RandomTsp\src\main\java\com\google\ortools\constraint_solver\routing_enums.pb.h"
#include "C:\Users\tima\Downloads\or-tools_VisualStudio2022-64bit_v9.3.10497\examples\java\RandomTsp\src\main\java\com\google\ortools\constraint_solver\routing_index_manager.h"
#include "C:\Users\tima\Downloads\or-tools_VisualStudio2022-64bit_v9.3.10497\examples\java\RandomTsp\src\main\java\com\google\ortools\constraint_solver\routing_parameters.h"

using namespace std;
namespace fs = filesystem;



struct Point {
    double x;
    double y;
};

double dist(Point p1, Point p2) {
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}

void readInput(std::vector<std::vector<double>>& matrix, int N, std::vector<Point>& points) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (i == j) {
                matrix[i][j] = 0;
            }
            else {
                matrix[i][j] = dist(points[i], points[j]);
            }
        }
    }
}

namespace operations_research {
    struct DataModel {
        std::vector<std::vector<double>> distance_matrix;
        std::vector<std::int64_t> demands;
        std::vector<std::int64_t> vehicle_capacities;
        std::int64_t num_vehicles = 0;
        RoutingIndexManager::NodeIndex depot{ 0 };
    };

    void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
        const RoutingModel& routing, const Assignment& solution) {
        int64_t total_distance{ 0 };
        int64_t total_load{ 0 };
        for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
            int64_t index = routing.Start(vehicle_id);
            // LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
            int64_t route_distance{ 0 };
            int64_t route_load{ 0 };
            std::stringstream route;
            while (routing.IsEnd(index) == false) {
                int64_t node_index = manager.IndexToNode(index).value();
                route_load += data.demands[node_index];
                // route << node_index << " Load(" << route_load << ") -> ";
                int64_t previous_index = index;
                index = solution.Value(routing.NextVar(index));
                route_distance += routing.GetArcCostForVehicle(previous_index, index,
                    int64_t{ vehicle_id });
            }
            /*
             LOG(INFO) << route.str() << manager.IndexToNode(index).value();
             LOG(INFO) << "Distance of the route: " << route_distance << "m";
             LOG(INFO) << "Load of the route: " << route_load;
             */
            total_distance += route_distance;
            total_load += route_load;
        }
        std::cout << total_distance << std::endl;     // LOG(INFO) before
        /*
           LOG(INFO) << "Total load of all routes: " << total_load;
           LOG(INFO) << "";
           LOG(INFO) << "Advanced usage:";
           LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
         */
    }

    void VrpCapacity(DataModel init_data) {
        DataModel data = init_data;
        RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
            data.depot);
        RoutingModel routing(manager);
        const int transit_callback_index = routing.RegisterTransitCallback(
            [&data, &manager](int64_t from_index, int64_t to_index) -> int64_t {
                // Convert from routing variable Index to distance matrix NodeIndex.
                int from_node = manager.IndexToNode(from_index).value();
                int to_node = manager.IndexToNode(to_index).value();
                return data.distance_matrix[from_node][to_node];
            });
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
        const int demand_callback_index = routing.RegisterUnaryTransitCallback(
            [&data, &manager](int64_t from_index) -> int64_t {
                // Convert from routing variable Index to demand NodeIndex.
                int from_node = manager.IndexToNode(from_index).value();
                return data.demands[from_node];
            });
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,    // transit callback index
            int64_t{ 0 },               // null capacity slack
            data.vehicle_capacities,  // vehicle maximum capacities
            true,                     // start cumul to zero
            "Capacity");
        RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
        search_parameters.set_first_solution_strategy(
            FirstSolutionStrategy::PATH_CHEAPEST_ARC);
        search_parameters.set_local_search_metaheuristic(
            LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
        search_parameters.mutable_time_limit()->set_seconds(300);
        const Assignment* solution = routing.SolveWithParameters(search_parameters);
        PrintSolution(data, manager, routing, *solution);
    }
}

int main(int argc, char** argv) {
    std::string path = "C:/Users/tima/Downloads/data";
    auto it = fs::directory_iterator(path);
    std::vector<fs::path> array_path;
    copy_if(fs::begin(it), fs::end(it), std::back_inserter(array_path),
        [](const auto& entry) {
            return fs::is_regular_file(entry);
        });
    for (auto& p : array_path) {
        std::ifstream fin;
        fin.open(p.string());
        std::cout << p.string() << std::endl;
        int64_t N, num_vehicles_one, vehicle_capacities_one;
        fin >> N >> num_vehicles_one >> vehicle_capacities_one;
        std::vector<int64_t> buffer(num_vehicles_one, vehicle_capacities_one);
        operations_research::DataModel data;
        data.vehicle_capacities = buffer;
        data.num_vehicles = num_vehicles_one;
        std::vector<Point> points(N);
        std::vector<int64_t> buf(N);
        for (int i = 0; i < N; i++) {
            int64_t demand;
            Point p;
            fin >> demand >> p.x >> p.y;
            points[i] = p;
            buf[i] = demand;
        }
        std::vector<std::vector<double> > matrix(N, std::vector<double>(N));
        readInput(matrix, N, points);
        data.demands = buf;
        data.distance_matrix = matrix;
        operations_research::VrpCapacity(data);
    }
    return EXIT_SUCCESS;
}
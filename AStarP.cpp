#include "AStarP.h"

#include <limits>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <mpi.h>

AStarP::AStarP(int task_id, int tasks_size, Graph graph, int startVertex, int endVertex, int numberOfRows, int numberOfColumns) :
    task_id(task_id),
    tasks_size(tasks_size),
    graph(graph),
    startVertex(startVertex),
    endVertex(endVertex),
    numberOfRows(numberOfRows),
    numberOfColumns(numberOfColumns) {
    initializeVariables();
}

AStarP::AStarP() {

}

void AStarP::initializeVariables() {
    debug = false;
    countOfExpansions = 0;
    end_reached = false;
    bestPathCost = std::numeric_limits<double>::infinity();
    adjacencyLists = graph.getAdjList();
    numberOfVertices = graph.getNumberOfVertices();
    closedSet = std::vector<bool>(numberOfVertices, false);
    gScore = std::vector<double>(numberOfVertices, std::numeric_limits<double>::infinity());
    cameFrom = std::vector<int>(numberOfVertices, -1);

    message_counter = 0;
    t_max = 0;
    is_active = true;
    terminate = false;
    waiting_for_control_wave_to_come_back = false;
}

void AStarP::run() {
    find_the_path();

    MPI_Barrier(MPI_COMM_WORLD);

    std::cout << task_id << " " << countOfExpansions << "\n";
    reconstruct_the_path(); 
}

void AStarP::process() {
    if (!openSet.empty()) {
        is_active = true;
        std::pair<int,double> currentVertex = expandNextVertex();
        if (currentVertex.second > bestPathCost) {
            clearOpenSet();
        }
        if (currentVertex.first == endVertex) {
            bestPathCost = gScore[currentVertex.first];
            end_reached = true;
            send_end_reached(currentVertex.second);
            return;
        }
        exploreAllNeighbors(currentVertex.first);
    }
}

std::pair<int,double> AStarP::expandNextVertex() {
    countOfExpansions++;
    std::pair<int,double> currentVertex = openSet.top();
    openSet.pop();
    closedSet[currentVertex.first] = true;
    return currentVertex;
}

void AStarP::exploreAllNeighbors(int currentVertex) {
    for (const auto& neighbor : adjacencyLists[currentVertex]) {
        if (neighbor.first == cameFrom[currentVertex]) {
            continue;
        }
        int neighborId = neighbor.first;
        double edgeCost = neighbor.second;
        
        double tentativeGScore = gScore[currentVertex] + edgeCost;

        int processor_for_neighbor = hash_vertex_to_processor(neighborId);
        if (processor_for_neighbor == task_id) {
            exploreVertex(neighborId, tentativeGScore, currentVertex);
        } else {
            send_vertex_to_its_processor(processor_for_neighbor, neighborId, tentativeGScore, currentVertex);
        }
    }
}

void AStarP::exploreVertex(int vertex_id, int tentative_g_score, int came_from) {
    if (tentative_g_score < gScore[vertex_id]) {
        cameFrom[vertex_id] = came_from;
        gScore[vertex_id] = tentative_g_score;
        double h = heuristicFunction(vertex_id);
        openSet.push({vertex_id, tentative_g_score + h});
        return;
    }
}

double AStarP::heuristicFunction(int vertex) {
    // chebyshev metric
    //return std::max(std::abs(vertex % numberOfColumns - endVertex % numberOfColumns), std::abs(vertex / numberOfRows - endVertex / numberOfRows)) * 10;
    
    // euclidean metric 
    int vertexRow = vertex / numberOfColumns;
    int vertexCol = vertex % numberOfColumns;
    int endRow = endVertex / numberOfColumns;
    int endCol = endVertex % numberOfColumns;
    return std::sqrt(std::pow(vertexRow - endRow, 2) + std::pow(vertexCol - endCol, 2))*10;
}

void AStarP::find_the_path() {
    if (task_id == hash_vertex_to_processor(startVertex)){
        gScore[startVertex] = 0;
        openSet.push({startVertex, 0});
    } 
    while (!terminate) {
        while (is_active) {
            is_active = false;
            receive_basic_messages();
            receive_control_messages();
            receive_end_reached();
            process();
        }

        if (end_reached) {
            if (!waiting_for_control_wave_to_come_back) {
                start_control_wave();
            }
            receive_control_messages();
        } else {
            is_active = true;
        }
        terminate = terminate || receive_terminate();
    }
}

void AStarP::reconstruct_the_path() {
    bool dont_send = false;
    std::vector<int> path;
    if (task_id == hash_vertex_to_processor(endVertex)) {
        int previousVertex = cameFrom[endVertex];
        path = {endVertex, previousVertex};
        if (hash_vertex_to_processor(previousVertex) == task_id) {
            dont_send = true;
        } else {
            send_path(path);
        }
    }
    while (true) {
        if (!dont_send) {
            path = receive_path();
            if (path.size() == 0) {
                break;
            }
        } 
        if (path.back() == startVertex) {
            send_path_found();
            print_path(path);
            break;
        }
        int previousVertex = cameFrom[path.back()];
        path.push_back(previousVertex);
        if (hash_vertex_to_processor(previousVertex) == task_id) {
            dont_send = true;
        } else {
            send_path(path);
            dont_send = false;
        }
    }
}

void AStarP::send_vertex_to_its_processor(int target_task_id, int vertex_id, int tentativeGScore, int coming_from)                                                                   {
    std::vector<int> sendBuffer = {clock, vertex_id, tentativeGScore, coming_from};
    MPI_Bsend(sendBuffer.data(), 4, MPI_INT, target_task_id, (int)TAG::BASIC_MESSAGE, MPI_COMM_WORLD);
    message_counter++;
}

void AStarP::receive_basic_messages(){
    while (true) { // todo: refactor this while cycle
        MPI_Status status;
        int flag;
        MPI_Iprobe(MPI_ANY_SOURCE, (int)TAG::BASIC_MESSAGE, MPI_COMM_WORLD, &flag, &status);
        if (!flag) {
            return;
        }
        std::vector<int> receiveBuffer(4);
        MPI_Recv(receiveBuffer.data(), 4, MPI_INT, MPI_ANY_SOURCE, (int)TAG::BASIC_MESSAGE, MPI_COMM_WORLD, &status);
        message_counter--;
        t_max = std::max(receiveBuffer[0], t_max);
        int new_vertex_id = receiveBuffer[1];
        int new_tentative_g_score = receiveBuffer[2];
        int new_came_from = receiveBuffer[3];
        exploreVertex(new_vertex_id, new_tentative_g_score, new_came_from); 
        is_active = true;
    }
}

void AStarP::start_control_wave() {
    clock++;
    std::vector<int> sendBuffer = {clock, message_counter, (int)(false), task_id};
    int receiver_id = (task_id+1) % tasks_size;
    MPI_Bsend(sendBuffer.data(), 4, MPI_INT, receiver_id, (int)TAG::CONTROL_MESSAGE, MPI_COMM_WORLD);
}

void AStarP::receive_control_messages() {
    while (true) { // TODO: refactor this while cycle
        MPI_Status status;
        int flag;
        MPI_Iprobe(MPI_ANY_SOURCE, (int)TAG::CONTROL_MESSAGE, MPI_COMM_WORLD, &flag, &status);
        if (!flag) {
            return;
        }
        std::vector<int> receiveBuffer(4);
        MPI_Recv(receiveBuffer.data(), 4, MPI_INT, MPI_ANY_SOURCE, (int)TAG::CONTROL_MESSAGE, MPI_COMM_WORLD, &status);
        int control_clock = receiveBuffer[0];
        int controlCounterCum = receiveBuffer[1];
        bool control_failed = (bool)receiveBuffer[2];
        int sender_id = receiveBuffer[3];
        clock = std::max(clock, control_clock);
        if (sender_id == task_id) {  // wave made it full round
            if ((controlCounterCum == 0) && (!control_failed)) {  // control was successful -> terminate
                terminate = true;
                send_terminate_all();
                break;
            } else {  // control was not successful -> set to active
                waiting_for_control_wave_to_come_back = false;
                is_active = true;
            }
        } else {  // send it further
            std::vector<int> sendBuffer = {control_clock, controlCounterCum + message_counter, control_failed || (t_max >= control_clock), sender_id};
            int receiver_id = (task_id + 1) % tasks_size;
            MPI_Bsend(sendBuffer.data(), 4, MPI_INT, receiver_id, (int)TAG::CONTROL_MESSAGE, MPI_COMM_WORLD);
        }
    }
}

void AStarP::send_terminate_all() {
    int b = 0;
    for (int i = 0; i < tasks_size; i++) {
        if (i == task_id) { continue;}
        MPI_Bsend(&b, 1, MPI_INT, i, (int)TAG::TERMINATE_MESSAGE, MPI_COMM_WORLD);
    }
}

bool AStarP::receive_terminate() {
    MPI_Status status;
    int flag;
    MPI_Iprobe(MPI_ANY_SOURCE, (int)TAG::TERMINATE_MESSAGE, MPI_COMM_WORLD, &flag, &status);
    if (flag) {
        return true;
    }
    return false;
}

void AStarP::send_end_reached(double cost) {
    for (int i = 0; i < tasks_size; i++) {
        if (i == task_id) { continue;}
        MPI_Bsend(&cost, 1, MPI_DOUBLE, i, (int)TAG::END_REACHED, MPI_COMM_WORLD);
    }
}

void AStarP::receive_end_reached() {
    MPI_Status status;
    int flag;
    MPI_Iprobe(MPI_ANY_SOURCE, (int)TAG::END_REACHED, MPI_COMM_WORLD, &flag, &status);
    if (flag) {
        double newCost;
        MPI_Recv(&newCost, 1, MPI_DOUBLE, MPI_ANY_SOURCE, (int)TAG::END_REACHED, MPI_COMM_WORLD, &status);
        bestPathCost = newCost;
        end_reached = true;
    }
}

void AStarP::send_path(std::vector<int> path) {
    int pathSize = path.size();
    int destination = hash_vertex_to_processor(path[pathSize-1]);
     
    MPI_Send(&pathSize, 1, MPI_INT, destination, (int)TAG::PATH_SIZE, MPI_COMM_WORLD);
    MPI_Send(path.data(), pathSize, MPI_INT, destination, (int)TAG::PATH, MPI_COMM_WORLD);
}

std::vector<int> AStarP::receive_path() {
    int pathSize;
    MPI_Recv(&pathSize, 1, MPI_INT, MPI_ANY_SOURCE, (int)TAG::PATH_SIZE, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
    if (pathSize == 0) {
        return std::vector<int>();
    } else {
        std::vector<int> path(pathSize);
        // std::cout << "Path was received by task " << task_id << std::endl;
        MPI_Recv(path.data(), pathSize, MPI_INT, MPI_ANY_SOURCE, (int)TAG::PATH, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        return path;
    }
}

void AStarP::send_path_found() {
    for (int i = 0; i < tasks_size; i++) {
        if (i == task_id) { continue; }
        int zero = 0;
        MPI_Send(&zero, 1, MPI_INT, i, (int)TAG::PATH_SIZE, MPI_COMM_WORLD);
    }
}

void AStarP::print_path(std::vector<int> path) {
    for (auto it = path.rbegin(); it != path.rend(); it++) {
        std::cout << *it << " -> ";
    }
}

int AStarP::hash_vertex_to_processor(int vertex){
    return vertex % tasks_size;
}

void AStarP::clearOpenSet() {
    openSet = std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, PairComparator<std::pair<int, double>>>();
}

#ifndef ASTARP_H
#define ASTARP_H

#include <Graph.h>

#include <vector>
#include <queue>

enum class TAG {
    BASIC_MESSAGE,
    CONTROL_MESSAGE,
    TERMINATE_MESSAGE,
    END_REACHED,
    PATH_SIZE,
    PATH
};

template<typename T>
struct PairComparator {
    bool operator()(const T& lhs, const T& rhs) const {
        return lhs.second > rhs.second;
    }
};

class AStarP {
public:
    AStarP(int task_id, int tasks_size, Graph graph, int startVertex, int endVertex, int numberOfRows, int numberOfColumns);
    AStarP();


private:
    Graph                   graph;
    int                     numberOfVertices;
    int                     startVertex;
    int                     endVertex;
    int                     numberOfRows;
    int                     numberOfColumns;

    double                  bestPathCost;
    int                     countOfExpansions;
    bool                    end_reached;
    std::vector<int>        cameFrom;   
    std::vector<bool>       closedSet;
    std::vector<double>     gScore;
    std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, PairComparator<std::pair<int, double>>> openSet;
    std::vector<std::vector<std::pair<int,double>>> adjacencyLists;

    int     task_id;
    int     tasks_size;
    int     message_counter;
    int     t_max;
    int     clock;
    bool    is_active;
    bool    terminate;
    bool    waiting_for_control_wave_to_come_back;
    bool    debug;

public:
    void run();

private:
    void initializeVariables();
    void process();
    std::pair<int, double>  expandNextVertex();
    void exploreAllNeighbors(int currentVertex);
    void exploreVertex(int vertex_id, int tentative_g_score, int came_from);
    double heuristicFunction(int vertex);

    void find_the_path();
    void reconstruct_the_path();

    void send_vertex_to_its_processor(int target_task_id, int vertex_id, int tentativeGScore, int coming_from);
    void receive_basic_messages();

    void start_control_wave();
    void receive_control_messages();

    void send_terminate_all();
    bool receive_terminate();

    void send_end_reached(double cost);
    void receive_end_reached();

    void send_path(std::vector<int> path);
    std::vector<int> receive_path();
    void send_path_found();
    void print_path(std::vector<int> path);


    int hash_vertex_to_processor(int vertex);

private:
    void clearOpenSet();
};


#endif 
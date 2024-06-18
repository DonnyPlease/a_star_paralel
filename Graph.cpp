#include <Graph.h>

#include <iostream>
#include <vector>
#include <limits>


Graph::Graph(int vertices) {
    numberOfVertices = vertices;
    adjacencyLists.resize(numberOfVertices, std::vector<std::pair<int, double>>());
    for (int i = 0; i < numberOfVertices; i++) {
        adjacencyLists[i].reserve(10);
    }
}

Graph::Graph() :
    numberOfVertices(0),
    adjacencyLists(std::vector<std::vector<std::pair<int,double>>>()) {
}

Graph::Graph(const Graph &graph) :
    numberOfVertices(graph.numberOfVertices),
    adjacencyLists(graph.adjacencyLists) {
}

void Graph::addEdge(int vertex1, int vertex2, int cost) {
    adjacencyLists[vertex1].emplace_back(std::pair<int, double>({vertex2, cost})); 
    adjacencyLists[vertex2].emplace_back(std::pair<int, double>({vertex1, cost})); 
}

std::vector<std::pair<int, double>> Graph::getNeighbors(int vertex) {
    return adjacencyLists[vertex];
}

std::vector<std::vector<std::pair<int, double>>> Graph::getAdjList() {
    return adjacencyLists;
}

int Graph::getNumberOfVertices(){
    return numberOfVertices;
}

void Graph::printGraph() {
    for (int i = 0; i < numberOfVertices; ++i) {
        std::cout << "Vertex " << i << ":";
        for (size_t j = 0; j < adjacencyLists[i].size(); ++j) {
            std::cout << " -> " << adjacencyLists[i][j].first;
        }
        std::cout << std::endl;
    }
}

int Graph::getCost(int u, int v) {
    return adjacencyLists[u][v].second;
}

void Graph::removeEdge(int u, int v) {
    for (auto it = adjacencyLists[u].begin(); it != adjacencyLists[u].end(); it++) {
        if (it->first == v) {
            adjacencyLists[u].erase(it);
            break;
        }
    }
    for (auto it = adjacencyLists[v].begin(); it != adjacencyLists[v].end(); it++) {
        if (it->first == u) {
            adjacencyLists[v].erase(it);
            break;
        }
    }
}

Graph GraphFactory::createFullyConnectedGraph(int vertices) {
    Graph graph(vertices);
    for (int i = 0; i < vertices; i++) {
        for (int j = i + 1; j < vertices; j++) {
            graph.addEdge(i, j);
        }
    }
    return graph;
}

Graph GraphFactory::create2DGridGraph(int rows, int columns) {
    Graph graph(rows*columns);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < columns; j++) {
            int currentVertex = i*columns + j;
            if (j < columns-1) {
                graph.addEdge(currentVertex, currentVertex+1, 10);
            }
            if (i < rows-1) {
                graph.addEdge(currentVertex, currentVertex+columns, 10);
                if (j < columns-1) {
                    graph.addEdge(currentVertex, currentVertex+columns+1, 14);
                }
                if (j > 0) {
                    graph.addEdge(currentVertex, currentVertex+columns-1, 14);
                }
            }
        }
    }
    return graph;
}


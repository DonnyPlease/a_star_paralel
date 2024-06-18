#include <Graph.h>
#include <AStarP.h>

#include <mpi.h>


int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);

    int tasks_size;
    MPI_Comm_size(MPI_COMM_WORLD, &tasks_size);

    int task_id;
    MPI_Comm_rank(MPI_COMM_WORLD, &task_id);

    int numberOfRows = 1001;
    int numberOfColumns = 1001;

    Graph graph = GraphFactory::create2DGridGraph(numberOfRows, numberOfColumns);
    int startVertex = 0;
    int endVertex = 500500;

    AStarP astarProcessor(task_id, tasks_size, graph, startVertex, endVertex, numberOfRows, numberOfColumns);
    astarProcessor.run();

    MPI_Finalize();

}
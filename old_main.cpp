#include <mpi.h>
#include <vector>
#include <iostream>
#include <sstream>
#include <random>
#include <algorithm>


std::vector<int> initializeNumbers(int task_id) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(1,10);
    int random_number = distr(gen);
    std::cout << "Task " << task_id << " generated " << random_number << std::endl;
    std::vector<int> numbers;
    for (int i = 1; i < random_number; i++) {
        numbers.push_back(i);
    }
    std::shuffle(numbers.begin(), numbers.end(), gen);
    return numbers;
}

void digest_incoming_basic_messages(std::vector<int> & numbers, int &counter, int& t_max, int task_id) {
    MPI_Status status;
    int flag;
    MPI_Iprobe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &flag, &status);

    while (flag) {
        std::cout << task_id << " is reading basic message" << std::endl; 
        std::vector<int> rBuff;
        rBuff.resize(2);

        std::cout << "Task " << task_id << " is  receiving a basic message." << std::endl;
        MPI_Recv(&rBuff[0], 2, MPI_INT, MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &status);
        counter--;
        t_max = std::max(rBuff[0], t_max);
        std::cout << "Task " << task_id << " received number " << rBuff[1] << " from Task " << status.MPI_SOURCE << std::endl;
        numbers.push_back(rBuff[1]);

        // Probe again for more incoming basic messages
        MPI_Iprobe(MPI_ANY_SOURCE, 0, MPI_COMM_WORLD, &flag, &status);
    }
}
void startControlWave(int &counter, int &clock, int& t_max, int task_id) {
    clock++;
    std::vector<int> sendBuffer = {clock, counter, (int)(false), task_id};
    MPI_Bsend(sendBuffer.data(), 4, MPI_INT, (task_id+1)%4, 1, MPI_COMM_WORLD);
    std::cout << "Task " << task_id << " started a ControlWave" << std::endl;
}

void sendTerminate(int task_id, int size) {
    int b = 0;
    for (int i = 0; i < size; i++) {
        if (i == task_id) {
            continue;
        }
        MPI_Bsend(&b, 1, MPI_INT, i, 2, MPI_COMM_WORLD);
    }
}

void digest_incoming_control_messages(int &counter, int &clock, int& t_max, int task_id, bool &terminate, bool& isActive, int n_tasks, bool& waitingForWaveToComBack) {
    MPI_Status status;
    int flag;
    MPI_Iprobe(MPI_ANY_SOURCE, 1, MPI_COMM_WORLD, &flag, &status);
    
    while (flag) {
        std::cout << task_id << " is reading control message" << std::endl; 
        std::vector<int> receiveBuffer;
        receiveBuffer.resize(4);
        std::cout << "Task " << task_id << " is  receiving a control message." << std::endl;
        MPI_Recv(&receiveBuffer[0], 4, MPI_INT, MPI_ANY_SOURCE, 1, MPI_COMM_WORLD, &status);
        std::cout << "Task " << task_id << " is replying to control wave from: " << receiveBuffer[3] << std::endl;
        clock = std::max(clock, receiveBuffer[0]);
        if (receiveBuffer[3] == task_id) {                        // check who initiated the control wave
            if ((receiveBuffer[1] == 0) && !(bool)(receiveBuffer[2])) {   // check whether the condition of termination is met
                terminate = true;     
                                           // terminate all
                std::cout << "Sending TERMINATe to all tasks " << task_id << "   is active: " << isActive<< std::endl;
                sendTerminate(task_id, n_tasks);
                break;
            } else {    
                // try again this is done elsewhere
                waitingForWaveToComBack = false;
                isActive = true;
            }
        } else {
            std::vector<int> sendBuffer = {receiveBuffer[0], receiveBuffer[1] + counter, (int)((bool)receiveBuffer[2] || (t_max >= receiveBuffer[0])), receiveBuffer[3]};
            if (isActive) {
                sendBuffer[2] = 1;
            }
            MPI_Bsend(&sendBuffer[0], 4, MPI_INT, (task_id+1)%4, 1, MPI_COMM_WORLD);
        }
        // Probe again for more incoming control messages: tag == 1
        MPI_Iprobe(MPI_ANY_SOURCE, 1, MPI_COMM_WORLD, &flag, &status);
    }
}

void do_what_you_must(std::vector<int>& numbers, bool &isActive, int &counter, int& clock, int task_id) {
    // "resolve" numbers
    if (!numbers.empty()) {
        isActive = true;
        int curr = numbers.back();
        numbers.pop_back();

        if (curr > 3 || curr == task_id) {
            std::cout << "Task " << task_id << " resolving number " << curr << std::endl;
        } else {
            counter++;
            std::vector<int> buff = {clock, curr};
            MPI_Bsend(buff.data(), 2, MPI_INT, curr, 0, MPI_COMM_WORLD);
            std::cout << "Task " << task_id << " sending number " << curr << " to Task " << curr << std::endl;
        }
    }
}

bool terminateMessageReceived() {
     MPI_Status status;
    int flag;
    MPI_Iprobe(MPI_ANY_SOURCE, 2, MPI_COMM_WORLD, &flag, &status);
    if (flag) {
        return true;
    }
    return false;
}

int main(int argc, char* argv[]) {
    MPI_Init(&argc, &argv);

    int num_tasks;
    MPI_Comm_size(MPI_COMM_WORLD, &num_tasks);

    int task_id;
    MPI_Comm_rank(MPI_COMM_WORLD, &task_id);
    std::cout << task_id << " STARTING" << std::endl;

    std::vector<int> numbers = initializeNumbers(task_id);

    int counter = 0;
    int clock = 0;
    int t_max = 0;

    bool isActive = true;
    bool terminate = false;

    bool waitingForWaveToComeBack = false;

    while (!terminate) {
        while (isActive) {
            std::cout << task_id << " is still active" << std::endl;
            isActive = false;
            digest_incoming_basic_messages(numbers, counter, t_max, task_id);
            digest_incoming_control_messages(counter, clock, t_max, task_id, terminate, isActive, num_tasks, waitingForWaveToComeBack);
            do_what_you_must(numbers, isActive, counter, clock, task_id);
        }

        std::cout << task_id << " is not active" << std::endl;
        if (!waitingForWaveToComeBack) {
            startControlWave(counter, clock, t_max, task_id);
            waitingForWaveToComeBack = true;
        }
        digest_incoming_control_messages(counter, clock, t_max, task_id, terminate, isActive, num_tasks, waitingForWaveToComeBack);

        terminate = terminate || terminateMessageReceived();
    }
    std::cout << "TERMINATING task " << task_id << std::endl;
    MPI_Finalize();
    return 0;
}
#include "WPILib.h"
#include <stdio.h>
#include <string.h>
#include <inetLib.h>
#include <sockLib.h>
#include <fcntl.h>
#include <queue>
#include <vector>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <stdarg.h>

class RAWCServer {
public:
    RAWCServer(unsigned int port);
    ~RAWCServer();
    void handle();

    void print(char* format, ...);

    class Stack
    {
        public:
            Stack();
            ~Stack();
            void lock();
            void unlock();
            std::queue<char*>* queue();
        private:
            std::queue<char*>* m_queue;
            pthread_mutex_t* mutex;
    };
private:
    int count;
    int fd;
    static void* handleConnection(void* arg);
    std::vector<RAWCServer::Stack*> stacks;
};

typedef struct threadArgs {
	int client;
	RAWCServer::Stack* stack;
} threadArgs;

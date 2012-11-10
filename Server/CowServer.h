#include "WPILib.h"
#include <stdio.h>
#include <string.h>
#include <inetLib.h>
#include <sockLib.h>
#include <fcntl.h>
#include <ioctl.h>
#include <queue>
#include <vector>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <stdarg.h>

class CowServer {
public:
    CowServer(unsigned int port);
    ~CowServer();
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
    std::vector<CowServer::Stack*> stacks;
};

typedef struct threadArgs {
	int client;
	CowServer::Stack* stack;
	std::vector<CowServer::Stack*>* stacks;
} threadArgs;

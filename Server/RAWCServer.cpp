#include "RAWCServer.h"

RAWCServer::RAWCServer(unsigned int port)
{
    count = 0;
    fd = 0;
    int sock_fd;
    struct sockaddr_in server;

    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(sock_fd == -1)
    {
        printf("Error: couldn't create socket on port %d!\n", port);
    }
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(port);

    int status = bind(sock_fd, (sockaddr*)&server, sizeof(server));
    if(status == -1)
    {
        printf("Error: couldn't bind socket on port %d\n", port);
    }

    status = listen(sock_fd, 10);
    if(status == -1)
    {
        printf("Error: couldn't listen to socket on port %d\n", port);
    }

    fd = sock_fd;
    fcntl(fd, F_SETFL, O_NONBLOCK);
}

RAWCServer::~RAWCServer()
{

}

void* RAWCServer::handleConnection(void* arg)
{
	printf("Handler thread started\n");
	int client = ((threadArgs*)arg)->client;
	Stack* stack = ((threadArgs*)arg)->stack;

	while(true)
	{
		stack->lock();
		while(!stack->queue()->empty())
		{
			char* s = stack->queue()->front();
			stack->queue()->pop();
			send(client, s, strlen(s), 0);
		}
		stack->unlock();
		Wait(0.1);
	}

	delete (threadArgs*)arg;
	return NULL;
}

void RAWCServer::print(char* format, ...)
{
	char buffer[256];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, 256, format, args);
	for(std::vector<RAWCServer::Stack*>::iterator it = stacks.begin(); it != stacks.end(); it++)
	{
		(*it)->lock();
		(*it)->queue()->push(buffer);
		(*it)->unlock();
	}
	va_end(args);
}

void RAWCServer::handle()
{
    struct sockaddr client;
    int s_client = sizeof(client);
    int result = accept(fd, &client, &s_client);
    if(result == -1 && errno == EWOULDBLOCK)
    {
    	return;
    }

    printf("Connection!\n");

    threadArgs* args = new threadArgs();

    Stack* s = new Stack();
    stacks.push_back(s);

    args->client = result;
    args->stack = s;

    pthread_t thread;
    pthread_create(&thread, NULL, &handleConnection, args);
}

RAWCServer::Stack::Stack()
{
	mutex = new pthread_mutex_t;
	m_queue = new std::queue<char*>();
	pthread_mutex_init(mutex, NULL);
}

RAWCServer::Stack::~Stack()
{
	pthread_mutex_destroy(mutex);
	delete mutex;
}

std::queue<char*>* RAWCServer::Stack::queue()
{
	return m_queue;
}

void RAWCServer::Stack::lock()
{
	pthread_mutex_lock(mutex);
}

void RAWCServer::Stack::unlock()
{
	pthread_mutex_unlock(mutex);
}


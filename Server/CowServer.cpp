#include "CowServer.h"

CowServer::CowServer(unsigned int port)
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
    int on = TRUE;
    ioctl(fd, FIONBIO, (int)&on);
    
    printf("Successfully started server on port %d\n", port);
}

CowServer::~CowServer()
{

}

void* CowServer::handleConnection(void* arg)
{
	printf("Handler thread started\n");
	int client = ((threadArgs*)arg)->client;
	Stack* stack = ((threadArgs*)arg)->stack;
	std::vector<CowServer::Stack*>* stacks = ((threadArgs*)arg)->stacks;

	while(true)
	{
		stack->lock();
		while(!stack->queue()->empty())
		{
			char* s = stack->queue()->front();
			stack->queue()->pop();
			for(int i = 0; i < strlen(s); i++)
			{
				char c = toascii(s[i]);
				int result = send(client, &c, 1, 0);
				if(result == -1)
				{
					for(std::vector<CowServer::Stack*>::iterator it = stacks->begin(); it != stacks->end(); it++)
					{
						if(*it == stack)
						{
							stacks->erase(it);
						}
					}
					delete stack;
					delete (threadArgs*)arg;
					return NULL;
				}	
			}
		}
		stack->unlock();
		Wait(0.1);
	}
	return NULL;
}

void CowServer::print(char* format, ...)
{
	char buffer[256];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, 256, format, args);
	//printf("Printing to network: %s\n", buffer);
	for(std::vector<CowServer::Stack*>::iterator it = stacks.begin(); it != stacks.end(); it++)
	{
		(*it)->lock();
		(*it)->queue()->push(buffer);
		(*it)->unlock();
	}
	va_end(args);
}

void CowServer::handle()
{
    struct sockaddr client;
    int s_client = sizeof(client);
    //return;
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
    args->stacks = &stacks;

    pthread_t thread;
    pthread_create(&thread, NULL, &handleConnection, args);
}

CowServer::Stack::Stack()
{
	mutex = new pthread_mutex_t;
	m_queue = new std::queue<char*>();
	pthread_mutex_init(mutex, NULL);
}

CowServer::Stack::~Stack()
{
	pthread_mutex_destroy(mutex);
	delete m_queue;
	delete mutex;
}

std::queue<char*>* CowServer::Stack::queue()
{
	return m_queue;
}

void CowServer::Stack::lock()
{
	pthread_mutex_lock(mutex);
}

void CowServer::Stack::unlock()
{
	pthread_mutex_unlock(mutex);
}


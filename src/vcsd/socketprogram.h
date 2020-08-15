#include <sys/socket.h>
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>

#define MAX_CLIENTS 2

//////////////////////////////////////////////
// socket connection data structure (mutex not implemented!)
///////////////////////////////////////////////

struct client_struct
{
  int id;
  int sockfd;
  sockaddr_in sockaddr;
} clients[MAX_CLIENTS];

void initClients(void)
{
  for (int k = 0; k < MAX_CLIENTS; k++)
  {
    clients[k].sockfd = -1;
  }
}
int get_sockfd(int idx)
{
  return clients[idx].sockfd;
}
int findEmptyClient(void)
{
  for (int k = 0; k < MAX_CLIENTS; k++)
  {
    if (clients[k].sockfd < 0)
      return k;
  }
  return -1;
}

int findClientByID(int id)
{
  for (int k = 0; k < MAX_CLIENTS; k++)
  {
    if (clients[k].sockfd < 0)
      continue;
    if (clients[k].id == id)
      return k;
  }
  return -1;
}

void addClient(int id, int sockfd, sockaddr_in sockaddr)
{
  int idx = findEmptyClient();
  if (idx < 0)
  {
    printf("error: no empty entries\n");
    exit(1);
  }

  clients[idx].id = id;
  clients[idx].sockfd = sockfd;
  clients[idx].sockaddr = sockaddr;
  printf("client[%d] added (id=%d)\n", idx, id);
}

void deleteClient(int id)
{
  int idx = findClientByID(id);
  if (idx < 0)
  {
    printf("error: unknown id\n");
    exit(1);
  }
  close(clients[idx].sockfd);
  clients[idx].sockfd = -1;
}

int startupServer(char *ipaddr, int portno)
{
  struct sockaddr_in sockaddr;
  int sockfd;
  int val = 1;

  // create an unnamed socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);

  // set a socket option to reuse the server address
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val)) != 0)
  {
    printf("error: setsockopt(): %s\n", strerror(errno));
    return -1;
  }

  // name the socket with the server address
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_addr.s_addr = inet_addr(ipaddr); //htonl(INADDR_ANY);
  sockaddr.sin_port = htons(portno);
  memset(&sockaddr.sin_zero, 0, 8);

  if (bind(sockfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
  {
    printf("error: bind(): %s\n", strerror(errno));
    return -1;
  }

  // set the maximum number of pending connection requests
  if (listen(sockfd, 10) != 0)
  {
    printf("error: listen(): %s\n", strerror(errno));
    return -1;
  }

  return sockfd;
}
#include <cstddef>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>
#include <thread>
#include <iostream>
#include <poll.h>

#include <or_protocol/or_node.h>


namespace or_protocol {


void *get_in_addr(struct sockaddr *sa)
{
  if (sa->sa_family == AF_INET)
    return &(((struct sockaddr_in*)sa)->sin_addr);
  return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


ORNode::ORNode(std::string _IP, int _port) :
  run(true),
  port(_port),
  IP(_IP)
{
  recv_thread = std::thread(&ORNode::recv_loop, this);
  send_thread = std::thread(&ORNode::send_loop, this);

  std::cout << "[ORNode] started send/recv threads" << std::endl;
}


ORNode::~ORNode() {
  run = false;
  shutdown(recv_sockfd, SHUT_RD); // forces recvfrom command to terminate
  send_thread.join();
  recv_thread.join();
}


void ORNode::recv_loop()
{
  struct addrinfo hints, *res, *p;
  struct sockaddr_storage addr;
  const int bufflen = 100;
  char buff[bufflen];
  int status, num_bytes;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE;

  std::string port_str = std::to_string(port);
  if ((status = getaddrinfo(NULL, port_str.c_str(), &hints, &res)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(status));
    run = false;
    return;
  }

  // loop through all the results and bind to the first we can
  for (p = res; p != NULL; p = p->ai_next) {

    if ((recv_sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      perror("[recv_loop] socket");
      continue;
    }

    if (bind(recv_sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(recv_sockfd);
      perror("[recv_loop] bind");
      continue;
    }

    break;
  }

  if (p == NULL) {
    fprintf(stderr, "[recv_loop] failed to bind socket\n");
    run = false;
    return;
  }

  freeaddrinfo(res);

  socklen_t addr_len = sizeof addr;
  while (run) {

    if ((num_bytes = recvfrom(recv_sockfd, buff, bufflen - 1, 0,
                              (struct sockaddr *)&addr, &addr_len)) == -1) {
      perror("[recv_loop] recvfrom");
      run = false;
      return;
    }

    char s[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, get_in_addr((struct sockaddr *)&addr), s,
                  sizeof s) == NULL) {
      perror("[recv_loop] inet_ntop");
      continue;
    }

    buff[num_bytes] = '\0';
    std::cout << "received message from " << s << ": " << buff << std::endl;
  }

  std::cout << "exiting recv_loop()" << std::endl;
}


void ORNode::send_loop()
{
  // get destination address (broadcast)

  int send_sockfd;
  struct sockaddr_in bcast_addr;
  struct hostent *he;
  int bcast = 1;

  size_t last_dot = IP.rfind(".");
  if (last_dot == std::string::npos) {
    fprintf(stderr, "[send_loop] failed to construct broadcast address");
    exit(EXIT_FAILURE);
  }
  std::string bcast_addr_str = IP.substr(0, last_dot) + std::string(".255");
  std::cout << "[send_loop] using broadcast address: " << bcast_addr_str << std::endl;

  if ((he = gethostbyname(bcast_addr_str.c_str())) == NULL) {
    perror("[send_loop] gethostbyname");
    exit(EXIT_FAILURE);
  }

  if ((send_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("[send_loop] socket");
    exit(EXIT_FAILURE);
  }

  // this call is what allows broadcast packets to be sent:
  if (setsockopt(send_sockfd, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof bcast) == -1) {
    perror("[send_loop] setsockopt (SO_BROADCAST)");
    exit(EXIT_FAILURE);
  }

  bcast_addr.sin_family = AF_INET;    // host byte order
  bcast_addr.sin_port = htons(port);  // short, network byte order
  bcast_addr.sin_addr = *((struct in_addr *)he->h_addr);
  memset(bcast_addr.sin_zero, '\0', sizeof bcast_addr.sin_zero);

  // build message
  std::string msg_str = "hello from " + IP;
  const char* msg = msg_str.c_str();

  int numbytes;
  while (run) {
    if ((numbytes = sendto(send_sockfd, msg, strlen(msg), 0,
                           (struct sockaddr *)&bcast_addr, sizeof bcast_addr)) == -1) {
      perror("[send_loop] sendto");
      break;
    }
    std::cout << "sent " << numbytes << " bytes to " << bcast_addr_str << std::endl;

    sleep(5);
  }

  close(send_sockfd);
}


} // namespace or_protocol

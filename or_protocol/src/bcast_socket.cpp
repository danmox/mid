#include <arpa/inet.h>
#include <cstddef>
#include <cstdlib>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#include <or_protocol/bcast_socket.h>


namespace or_protocol {


void* get_in_addr(struct sockaddr* sa)
{
  if (sa->sa_family == AF_INET)
    return &(((struct sockaddr_in*)sa)->sin_addr);
  return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


BCastSocket::BCastSocket(std::string _IP, int _port, buff_recv_func _recv_handle)
{
  port = _port;
  my_IP = _IP;
  recv_handle = _recv_handle;

  my_sa.sin_family = AF_INET;
  inet_pton(AF_INET, my_IP.c_str(), &(my_sa.sin_addr));
  my_sa.sin_port = htons(port);

  run = init_send_socket();
  recv_thread = std::thread(&BCastSocket::recv_loop, this);
}


BCastSocket::~BCastSocket()
{
  run = false;
  close(send_sockfd);
  shutdown(recv_sockfd, SHUT_RD);  // forces recvfrom command to terminate
  recv_thread.join();
}


// listen for incoming UDP messages on the supplied port
void BCastSocket::recv_loop()
{
  struct addrinfo hints, *res, *p;
  struct sockaddr_storage addr;
  const int bufflen = 1500;
  int status, num_bytes;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;       // IPv4
  hints.ai_socktype = SOCK_DGRAM;  // UDP
  hints.ai_flags = AI_PASSIVE;     // fill in my IP for me

  std::string port_str = std::to_string(port);
  if ((status = getaddrinfo(NULL, port_str.c_str(), &hints, &res)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(status));
    run = false;
    return;
  }

  for (p = res; p != NULL; p = p->ai_next) {

    if ((recv_sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      perror("[BCastSocket] socket");
      continue;
    }

    if (bind(recv_sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(recv_sockfd);
      perror("[BCastSocket] bind");
      continue;
    }

    break;
  }

  if (p == NULL) {
    fprintf(stderr, "[BCastSocket] failed to bind socket\n");
    run = false;
    return;
  }

  freeaddrinfo(res);

  socklen_t addr_len = sizeof addr;
  while (run) {

    buffer_ptr buff_ptr(new char[bufflen]);

    if ((num_bytes = recvfrom(recv_sockfd, buff_ptr.get(), bufflen - 1, 0,
                              (struct sockaddr*)&addr, &addr_len)) == -1) {
      perror("[BCastSocket] recvfrom");
      run = false;
      return;
    }

    // NOTE currently, broadcast messages originating from the send_loop() are
    // picked up in the recv_loop() of the same machine -- there must be a way
    // of configuring the system to automatically do this but for now just
    // manually filter them
    if (((struct sockaddr_in*)&addr)->sin_addr.s_addr == my_sa.sin_addr.s_addr)
      continue;

    char s[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, get_in_addr((struct sockaddr*)&addr), s,
                  sizeof s) == NULL) {
      perror("[BCastSocket] inet_ntop");
      continue;
    }

    if (num_bytes >= bufflen) {
      fprintf(stderr, "[BCastSocket] Insufficient buffer size. Dropping message.");
      continue;
    }

    if (num_bytes > 0)
      recv_handle(buff_ptr, num_bytes);
  }
}


bool BCastSocket::init_send_socket()
{
  // get destination address (broadcast)

  struct hostent* he;
  int bcast = 1;

  size_t last_dot = my_IP.rfind(".");
  if (last_dot == std::string::npos) {
    fprintf(stderr, "[BCastSocket] failed to construct broadcast address");
    return false;
  }
  bcast_IP = my_IP.substr(0, last_dot) + std::string(".255");
  std::cout << "[BCastSocket] using broadcast address: " << bcast_IP << std::endl;

  if ((he = gethostbyname(bcast_IP.c_str())) == NULL) {
    perror("[BCastSocket] gethostbyname");
    return false;
  }

  if ((send_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("[BCastSocket] socket");
    return false;
  }

  // allow broadcast packets to be sent:
  if (setsockopt(send_sockfd, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof bcast) == -1) {
    perror("[BCastSocket] setsockopt (SO_BROADCAST)");
    return false;
  }

  bcast_addr.sin_family = AF_INET;    // host byte order
  bcast_addr.sin_port = htons(port);  // short, network byte order
  bcast_addr.sin_addr = *((struct in_addr*)he->h_addr);
  memset(bcast_addr.sin_zero, '\0', sizeof bcast_addr.sin_zero);

  return true;
}


bool BCastSocket::send(const char* buff, int buff_size)
{
  int numbytes;
  if ((numbytes = sendto(send_sockfd, buff, buff_size, 0,
                         (struct sockaddr*)&bcast_addr, sizeof bcast_addr)) == -1) {
    perror("[BCastSocket] sendto");
    return false;
  }

  if (numbytes != buff_size) {
    fprintf(stderr, "[BCastSocket] buff contains %d bytes but only %d sent", buff_size, numbytes);
    return false;
  }

  return true;
}


}  // namespace or_protocol

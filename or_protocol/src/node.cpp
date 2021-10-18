#include <chrono>
#include <csignal>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>

#include <or_protocol/or_node.h>


volatile bool run = true;


void handler(int s)
{
  run = false;
}


int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cout << "[main] usage: or_test <ip address>" << std::endl;
    return 0;
  }

  struct sigaction siginthandler;
  siginthandler.sa_handler = handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  or_protocol::ORNode node(argv[1], 4568);

  while (run)
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

  return 0;
}

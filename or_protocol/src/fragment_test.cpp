#include <atomic>
#include <chrono>
#include <csignal>
#include <fmt/format.h>
#include <or_protocol/bcast_socket.h>
#include <or_protocol/constants.h>
#include <or_protocol/types.h>
#include <string.h>
#include <vector>


std::atomic<bool> run = true;


void signal_handler(int s)
{
  fmt::print("[main] received shutdown signal {}\n", s);
  run = false;
}


void buff_cb([[maybe_unused]]or_protocol::buffer_ptr& buff, int size)
{
  fmt::print("[main] received buffer with {} bytes\n", size);
}


int main(int argc, char** argv)
{
  if (argc < 3) {
    fmt::print("[main] usage: fragment_test <ip address> <transmit> [<size>]\n");
    return EXIT_FAILURE;
  }

  bool transmit;
  int bytes = 1472;
  if (std::string(argv[2]) == "transmit") {
    transmit = true;
    if (argc == 4) {
      bytes = std::stoi(argv[3]);
    }
    fmt::print("[main] using payload size of {}", bytes);
  } else if (std::string(argv[2]) == "receive") {
    transmit = false;
  } else {
    fmt::print("[main] unknown transmit option {}\n", argv[2]);
    return EXIT_FAILURE;
  }

  struct sigaction siginthandler;
  siginthandler.sa_handler = signal_handler;
  sigemptyset(&siginthandler.sa_mask);
  siginthandler.sa_flags = 0;
  sigaction(SIGINT, &siginthandler, NULL);

  or_protocol::BCastSocket sock(argv[1], or_protocol::OR_PROTOCOL_PORT, buff_cb);

  while (run && sock.is_running()) {
    if (transmit) {
      std::vector<char> payload(bytes, 'c');
      sock.send(payload.data(), payload.size());
      fmt::print("[main] sending payload with {} bytes\n", payload.size());
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  return EXIT_SUCCESS;
}

#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <or_protocol/network_state.h>


namespace fs = std::filesystem;


int routing_example()
{
  std::string package_path = ros::package::getPath("or_protocol");
  fs::path sample_dir = fs::path(package_path) / "test" / "samples";

  if (!fs::exists(sample_dir) || !fs::is_directory(sample_dir)) {
    std::cout << "directory: " << sample_dir << " does not exist";
    return 1;
  }

  std::vector<fs::path> sample_files;
  for (const auto& file : fs::directory_iterator(sample_dir))
    if (fs::is_regular_file(file) && file.path().extension() == ".yaml")
      sample_files.push_back(file.path());

  for (const fs::path& file : sample_files) {
    YAML::Node sample = YAML::LoadFile(file.string());

    or_protocol::ETXMap sample_link_etx;
    for (const auto& node : sample["link_etx"]) {
      const int& tx_node = node.first.as<int>();
      const YAML::Node& map_node = node.second;

      std::unordered_map<int, double> inner_map;
      for (auto& inner_node : map_node) {
        const int& rx_node = inner_node.first.as<int>();
        const double& etx = inner_node.second.as<double>();
        inner_map.emplace(rx_node, etx);
      }

      sample_link_etx.emplace(tx_node, inner_map);
    }

    or_protocol::IntVectorMap route_relays;
    for (const auto& node : sample["relay_map"]) {
      const int& relay = node.first.as<int>();
      const std::vector<int>& relays = node.second.as<std::vector<int>>();
      route_relays.emplace(relay, relays);
    }

    uint8_t src = sample["src"].as<uint8_t>();
    uint8_t dest = sample["dest"].as<uint8_t>();
    for (const auto& item : route_relays) {
      or_protocol::NetworkState ns;
      ns.set_etx_map(sample_link_etx);
      or_protocol::RoutingMap routing_map = ns.find_routes(item.first);

      std::cout << "NetworkState: " << std::endl << "  ";
      for (const int relay : routing_map[src][dest])
        std::cout << relay << ", ";
      std::cout << std::endl;
      std::cout << "Sample: " << std::endl << "  ";
      for (const int relay : route_relays[item.first])
        std::cout << relay << ", ";
      std::cout << std::endl;
    }
  }

  return 0;
}


int main()
{
  return routing_example();
}

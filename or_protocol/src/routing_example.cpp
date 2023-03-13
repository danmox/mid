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

    std::cout << file << std::endl;

    uint8_t src = sample["src"].as<uint8_t>();
    uint8_t dest = sample["dest"].as<uint8_t>();
    for (const auto& item : route_relays) {
      or_protocol::NetworkState ns;
      ns.set_etx_map(sample_link_etx);
      ns.update_routes(item.first);
      or_protocol::FixedRoutingMap routing_map = ns.get_routing_map();

      std::cout << "  NetworkState: " << std::endl << "    ";
      const or_protocol::RelayArray& ns_relays = routing_map[src][dest];
      for (size_t i = 0; i < ns_relays.size() - 1; i++)
        std::cout << int(ns_relays[i]) << ", ";
      std::cout << int(ns_relays.back()) << std::endl;

      std::cout << "  Sample: " << std::endl << "    ";
      const std::vector<int>& s_relays = route_relays[item.first];
      for (size_t i = 0; i < s_relays.size() - 1; i++)
        std::cout << s_relays[i] << ", ";
      std::cout << s_relays.back() << std::endl;
      std::cout << std::endl;
    }
  }

  // default route test
  or_protocol_msgs::Header header;
  header.src_id = 1;
  header.dest_id = 2;
  or_protocol::NetworkState ns;
  or_protocol::RelayArray default_route = ns.relays(header);
  // print default route
  std::cout << std::endl << "Default route: " << std::endl;
  for (size_t i = 0; i < default_route.size() - 1; i++)
    std::cout << int(default_route[i]) << ", ";
  std::cout << int(default_route.back()) << std::endl;

  return 0;
}


int main()
{
  return routing_example();
}

#include <filesystem>
#include <fmt/printf.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <or_protocol/network_state.h>


namespace fs = std::filesystem;


int routing_example()
{
  std::string package_path = ros::package::getPath("or_protocol");
  fs::path sample_dir = fs::path(package_path) / "test" / "samples";

  if (!fs::exists(sample_dir) || !fs::is_directory(sample_dir)) {
    fmt::print("directory: {} does not exist\n", sample_dir.c_str());
    return EXIT_FAILURE;
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

    or_protocol::IntVectorMap sample_relay_map;
    for (const auto& node : sample["relay_map"]) {
      const int& relay = node.first.as<int>();
      const std::vector<int>& relays = node.second.as<std::vector<int>>();
      sample_relay_map.emplace(relay, relays);
    }

    fmt::print("File: {}\n", file.c_str());

    int root_id;
    for (const auto& item : sample_relay_map) {
      root_id = item.first;
      break;
    }

    or_protocol::NetworkState ns;
    ns.set_etx_map(sample_link_etx);
    ns.update_routes(root_id);
    or_protocol::FixedRoutingMap computed_routing_map = ns.get_routing_map();

    int src = sample["src"].as<int>();
    int dest = sample["dest"].as<int>();
    for (const auto& item : sample_relay_map) {

      const or_protocol::RelayArray& ns_relays = computed_routing_map[src][dest][item.first];
      fmt::print("  NetworkState:\n    ");
      for (size_t i = 0; i < ns_relays.size() - 1; i++)
        fmt::print("{}, ", ns_relays[i]);
      fmt::print("{}\n", ns_relays.back());

      fmt::print("  Sample:\n    ");
      const std::vector<int>& s_relays = sample_relay_map[item.first];
      for (size_t i = 0; i < s_relays.size() - 1; i++)
        fmt::print("{}, ", s_relays[i]);
      fmt::print("{}\n\n", s_relays.back());
    }
  }

  // default route test
  or_protocol_msgs::Header header;
  header.src_id = 1;
  header.dest_id = 2;
  or_protocol::NetworkState ns;
  or_protocol::RelayArray default_route = ns.relays(header, header.src_id);
  // print default route
  fmt::print("Default rout for flow {} > {}:\n", header.src_id, header.dest_id);
  for (size_t i = 0; i < default_route.size() - 1; i++)
    fmt::print("{}, ", default_route[i]);
  fmt::print("{}\n", default_route.back());

  return 0;
}


int main()
{
  return routing_example();
}

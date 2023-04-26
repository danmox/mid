#include <queue>
#include <deque>
#include <unordered_map>

#include <or_protocol/network_state.h>
#include <or_protocol/utils.h>

#include <or_protocol_msgs/NetworkStatus.h>
#include <ros/console.h>


namespace or_protocol {


MsgStatus NodeState::update_queue(const or_protocol_msgs::Header& header)
{
  if (msg_hist_map.size() > MSG_BUFFER_CAPACITY) {
    SeqAttemptPair& oldest = msg_hist_deque.back();
    msg_hist_deque.pop_back();

    // remove the message from the map if no new attempts have been received
    if (oldest.attempt == msg_hist_map[oldest.seq].attempt)
      msg_hist_map.erase(oldest.seq);
  }

  unsigned int priority = relay_priority(header.curr_id, header);

  // NOTE ACK transmissions may fail, resulting in the source node transmitting
  // another attempt when the original message and information was actually
  // received. To avoid passing redundant information to the application we
  // distinguish between new messages that should be routed/ACKed accordingly
  // and new information that should also be passed to the application.
  MsgStatus status{false, false};
  if (msg_hist_map.count(header.seq) == 0) {
    status.is_new_msg = true;
    status.is_new_seq = true;
    msg_hist_deque.push_front(SeqAttemptPair{header.seq, header.attempt});
    msg_hist_map[header.seq] = AttemptPriorityPair{header.attempt, priority};
  } else if (msg_hist_map[header.seq].attempt < header.attempt) {
    status.is_new_msg = true;
    msg_hist_deque.push_front(SeqAttemptPair{header.seq, header.attempt});
    msg_hist_map[header.seq] = AttemptPriorityPair{header.attempt, priority};
  } else if (msg_hist_map[header.seq].priority > priority) {
    msg_hist_map[header.seq] = AttemptPriorityPair{header.attempt, priority};
  }

  return status;
}


void NodeState::ack_msg(const uint32_t seq)
{
  // future traffic can be dissabled for a message associated with seq by:
  // 1) setting the sequence number to zero so that a higher priority relay will
  //    not be found
  // 2) setting the attempts to the MAX_RETRY_COUNT so that a message with a
  //    higher number of attempts will not be received
  if (msg_hist_map.count(seq) == 0)
    msg_hist_deque.push_front(SeqAttemptPair{seq, MAX_RETRY_COUNT});
  msg_hist_map[seq] = AttemptPriorityPair{MAX_RETRY_COUNT, 0};
}


int NodeState::priority(const int seq)
{
  return msg_hist_map[seq].priority;
}


void NodeState::update_link_state(const PacketQueueItemPtr& beacon)
{
  // updates without a beacon frame are used to update the delivery probability
  // when beacon frames are dropped
  if (!beacon) {
    double elapsed_ms = (ros::Time::now() - last_beacon_stamp).toSec() * 1e3;
    if (elapsed_ms > 1.2 * (BEACON_INTERVAL + 2*BEACON_JITTER)) {
      last_beacon_stamp += ros::Duration(0, BEACON_INTERVAL * 1e6);
      delivery_probability.update(0);
      etx_seq++;
      OR_DEBUG("updating with no beacon received");
    } else {
      OR_DEBUG("elapsed_ms = %.2f", elapsed_ms);
    }
  } else {
    last_beacon_stamp = beacon->recv_time;
    delivery_probability.update(1);
    etx_seq++;
  }
}


or_protocol_msgs::ETXEntry NodeState::get_etx_entry(const int dest) const
{
  or_protocol_msgs::ETXEntry entry;
  entry.node = dest;
  entry.seq = etx_seq;
  double dp = delivery_probability.get();
  entry.etx = dp < 1.0 / ETX_MAX ? ETX_MAX : 1.0 / dp;
  return entry;
}


MsgStatus NetworkState::update_queue(const int node_id,
                                     const or_protocol_msgs::Header &header)
{
  return node_states[node_id].update_queue(header);
}


void NetworkState::ack_msg(const int node_id, const uint32_t seq)
{
  node_states[node_id].ack_msg(seq);
}


// TODO is it possible this gets called before a message has been received? This
// would result in a priority of 0 (the highest) and possible inapropriate
// inaction
int NetworkState::priority(const int node_id, const int seq)
{
  return node_states[node_id].priority(seq);
}


RelayArray NetworkState::relays(const or_protocol_msgs::Header& header)
{
  // NOTE returns all 0s (the default construction) if the given src, dest pair
  // is not in the routing table
  return routing_map[header.src_id][header.dest_id];
}


typedef std::pair<int, double> NodeCostPair;
class NodeETXPairGreater
{
  public:
    bool operator()(const NodeCostPair& lhs, const NodeCostPair& rhs)
    {
      return lhs.second > rhs.second;
    }
};
typedef std::priority_queue<NodeCostPair,
                            std::deque<NodeCostPair>,
                            NodeETXPairGreater> PriorityQueue;


void NetworkState::update_routes(const int root)
{
  std::unordered_set<int> ids;
  ETXEntryMap local_link_etx_table;
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    ids = node_ids;
    local_link_etx_table = link_etx_table;
  }

  // build link_etx used in packet route planning

  // NOTE link_etx augments link_etx_table to include self etx links and fills
  // in any missing links with ETX_MAX
  ETXMap link_etx;
  for (const int src : ids) {
    for (const int dest : ids) {
      if (local_link_etx_table[src].count(dest) == 0)
        link_etx[src][dest] = src == dest ? 0.0 : ETX_MAX;
      else
        link_etx[src][dest] = local_link_etx_table[src][dest].etx;
    }
  }

  //
  // compute path_etx and default_paths
  //

  // find the shortest path between each pair of nodes in the network
  ETXMap path_etx;
  VariableRoutingMap default_paths;
  for (const int node : ids) {

    PriorityQueue frontier;
    std::unordered_map<int, int> parent;
    std::unordered_map<int, double> cost;

    parent[node] = node;
    cost[node] = 0.0;
    frontier.push(NodeCostPair{node, 0.0});

    // shortest path from node to all other nodes
    while (!frontier.empty()) {

      NodeCostPair curr = frontier.top();
      frontier.pop();

      for (const auto& next : link_etx[curr.first]) {
        double new_cost = cost[curr.first] + next.second;
        if (cost.count(next.first) == 0 || new_cost < cost[next.first]) {
          cost[next.first] = new_cost;
          parent[next.first] = curr.first;
          frontier.push(NodeCostPair{next.first, new_cost});
        }
      }
    }

    // path ETX for the shortest paths from node to all other nodes
    for (const int dest : ids)
      path_etx[node][dest] = cost[dest];

    // reconstruct default paths
    // NOTE assumes a connected network!!
    // TODO handle disconnected networks
    for (const int dest : ids) {
      int current = dest;
      std::vector<int>& path = default_paths[node][dest];
      while (current != parent[current]) {
        path.push_back(current);
        current = parent[current];
      }
      path.push_back(current);
      std::reverse(path.begin(), path.end());
    }
  }

  // compute default path ETX (i.e. for every default path, the shortest
  // distance from every node in the network to the destination along the path)
  std::unordered_map<int, ETXMap> default_path_etx;
  for (const int s : ids) {
    for (const int d : ids) {
      for (const int n : ids) {
        double min_etx = std::numeric_limits<double>::max();
        for (const int p : default_paths[s][d])
          min_etx = std::min(min_etx, link_etx[n][p] + path_etx[p][d]);
        default_path_etx[s][d][n] = min_etx;
      }
    }
  }

  //
  // compute relay list from root to every other node in the network
  //

  // helper functions to make relay selection more readable
  auto pair_less = [](const auto& l, const auto& r) {
    return l.second < r.second;
  };
  auto min_etx_to_path = [&](const std::vector<int>& path, const int r) {
    double min = std::numeric_limits<double>::max();
    for (const int p : path)
      min = std::min(min, link_etx[r][p]);
    return min;
  };
  auto near_all = [&](const std::vector<NodeCostPair>& relays, const int n) {
    for (const NodeCostPair& r : relays)
      if (link_etx[r.first][n] > RELAY_ETX_THRESHOLD ||
          link_etx[n][r.first] > RELAY_ETX_THRESHOLD)
        return false;
    return true;
  };
  auto compute_closeness_factor = [&](const int c, const int n) {
    return std::max(CLOSENESS_FACTOR * link_etx[c][n], CLOSENESS_THRESHOLD);
  };

  // compute the full routing table; loosly based on the rules presented in
  // SOAR: https://ieeexplore.ieee.org/document/4912211
  VariableRoutingMap new_routing_map;
  for (const int src : ids) {
    for (const int dest : ids) {
      bool found_root_relays = false;
      const std::vector<int>& path = default_paths[src][dest];
      auto& default_path_i_etx = default_path_etx[src][dest];

      // compute relays for nodes on the default path
      std::unordered_set<int> relay_set(path.begin(), path.end());
      for (size_t i = 0; i < path.size() - 1 && !found_root_relays; i++) {
        const int c = path[i];
        const int n = path[i+1];

        // compute candidate relay nodes for p_curr
        std::vector<NodeCostPair> candidates;
        for (const auto& item : link_etx[c]) {
          const int r = item.first;  // candidate relay

          // candidate relays must satisfy:
          // 1) r moves the packet closer to dest
          // 2) r is within a threshold ETX of p_curr
          // 3) r is within a threshold ETX of a node in path
          if (r != n && default_path_i_etx[r] < default_path_i_etx[c] &&  // 1
              link_etx[c][r] < compute_closeness_factor(c,n) &&           // 2
              min_etx_to_path(path, r) < NEAR_PATH_THRESHOLD)             // 3
            candidates.push_back({r, link_etx[c][r] + default_path_i_etx[r]});
        }
        std::sort(candidates.begin(), candidates.end(), pair_less);

        // choose at most MAX_RELAY_COUNT relays from the candidates that are all
        // within RELAY_ETX_THRESHOLD of one another
        std::vector<NodeCostPair> relays = {{n, default_path_i_etx[n]}};
        for (size_t j = 0; j < candidates.size() && relays.size() < MAX_RELAY_COUNT; j++) {
          const int r = candidates[j].first;
          if (near_all(relays, r))
            relays.push_back({r, default_path_i_etx[r]});
        }

        if (c == root) {
          std::sort(relays.begin(), relays.end(), pair_less);
          std::vector<int> final_relays;
          for (const auto& item : relays) {
            final_relays.push_back(item.first);
          }
          found_root_relays = true;
          new_routing_map[src][dest] = final_relays;
          break;
        } else {
          for (const auto& item : relays)
            relay_set.insert(item.first);
        }
      }

      // only record routing information for flows involving root
      if (relay_set.count(root) == 0 || found_root_relays)
        continue;

      // root must be an auxiliary relay, compute it's relays
      const int c = root;
      std::vector<NodeCostPair> candidates;
      for (const int r : relay_set) {

        // candidate relays must satisfy:
        // 1) r moves the packet closer to dest
        // 2) r is within a threshold ETX of p_curr
        // 3) r is within a threshold ETX of a node in path (already satisfied)
        if (r != c && default_path_i_etx[r] < default_path_i_etx[c] &&  // 1
            link_etx[c][r] < compute_closeness_factor(c,r))             // 2
          candidates.push_back({r, link_etx[c][r] + default_path_i_etx[r]});
      }
      std::sort(candidates.begin(), candidates.end(), pair_less);

      // choose at most MAX_RELAY_COUNT relays from the candidates that are all
      // within RELAY_ETX_THRESHOLD of one another
      std::vector<NodeCostPair> relays;
      for (size_t j = 0; j < candidates.size() && relays.size() < MAX_RELAY_COUNT; j++) {
        const int r = candidates[j].first;
        if (near_all(relays, r))
          relays.push_back({r, default_path_i_etx[r]});
      }

      std::sort(relays.begin(), relays.end(), pair_less);
      std::vector<int> final_relays;
      for (const auto& item : relays)
        final_relays.push_back(item.first);
      new_routing_map[src][dest] = final_relays;
    }
  }

  // convert to fixed relay arrays used in or_protocol_msgs::Header and add
  // direct routing for any src, dest pairs for which root does not participate
  // NOTE this may occur when network state information on each node is out of
  // sync (i.e. node Y thinks node X should participate but node X does not)
  FixedRoutingMap new_fixed_routing_map;
  for (const int src : ids) {
    for (const int dest: ids) {
      if (src == dest)
        continue;
      if (new_routing_map[src][dest].size() == 0) {
        new_fixed_routing_map[src][dest] = {static_cast<uint8_t>(dest), 0, 0, 0};
      } else {
        const std::vector<int>& relays = new_routing_map[src][dest];
        for (size_t i = 0; i < relays.size(); i++)
          new_fixed_routing_map[src][dest][i] = relays[i];
        for (size_t i = relays.size(); i < MAX_RELAY_COUNT; i++)
          new_fixed_routing_map[src][dest][i] = 0;
      }
    }
  }

  std::lock_guard<std::mutex> lock(routing_map_mutex);
  routing_map = new_fixed_routing_map;
}


// NOTE not thread safe! only intended for testing
void NetworkState::set_etx_map(const ETXMap& map)
{
  for (const auto& list : map) {
    int tx_node = list.first;
    node_ids.insert(tx_node);
    for (const auto& item : list.second) {
      or_protocol_msgs::ETXEntry entry;
      entry.node = item.first;
      entry.etx = item.second;
      link_etx_table[tx_node][entry.node] = entry;
    }
  }
}


void NetworkState::process_beacon_queue(const int root)
{
  // TODO handling adding new nodes?

  ETXEntryMap new_link_etx_table;
  std::unordered_map<int, or_protocol_msgs::Point> new_node_positions;
  std::unordered_set<int> new_node_ids;
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    new_link_etx_table = link_etx_table;
    new_node_positions = node_positions;
    new_node_ids = node_ids;
  }

  PacketQueueItemPtr item;
  std::unordered_set<int> beacon_ids;
  while (beacon_queue.pop(item)) {
    or_protocol_msgs::Packet pkt;
    deserialize(pkt, reinterpret_cast<uint8_t*>(item->buffer()), item->size);
    or_protocol_msgs::NetworkStatus status_msg;
    deserialize(status_msg, pkt.data.data(), pkt.data.size());

    beacon_ids.insert(item->header.curr_id);

    for (const or_protocol_msgs::Point& point : status_msg.positions) {
      if (point.seq > new_node_positions[point.node].seq)
        new_node_positions[point.node] = point;
    }

    node_states[item->header.curr_id].update_link_state(item);

    for (const or_protocol_msgs::ETXList& list : status_msg.etx_table) {
      const int tx_node = list.node;
      new_node_ids.insert(tx_node);
      for (const or_protocol_msgs::ETXEntry& entry : list.etx_list) {
        new_node_ids.insert(entry.node);
        if (new_link_etx_table[tx_node][entry.node].seq < entry.seq) {
          new_link_etx_table[tx_node][entry.node] = entry;
        }
      }
    }
  }

  // TODO how to handle nodes that leave the network? if a node becomes
  // disconnected either temporarily or permanently the rest of the network
  // stops receiving beacon frames from them - as a result their ETX values
  // remain fixed resulting in inaccuracies in route setting. Enforce a timeout
  // after which a link's ETX gets set to ETX_MAX?

  // run through nodes that did not receive a beacon this iteration
  for (const int id : new_node_ids)
    if (beacon_ids.count(id) == 0 && id != root)
      node_states[id].update_link_state();

  // update ETX table with most recent ETX values in node_states
  // NOTE the ETX values held in node_states are guaranteed to be the most
  // recent and thus we don't need to check for increasing sequence numbers here
  for (const auto& item : node_states)
    new_link_etx_table[item.first][root] = item.second.get_etx_entry(root);

  std::lock_guard<std::mutex> lock(status_mutex);
  link_etx_table = new_link_etx_table;
  node_positions = new_node_positions;
  node_ids = new_node_ids;
}


or_protocol_msgs::NetworkStatus::Ptr NetworkState::generate_beacon()
{
  ETXEntryMap link_etx;
  std::unordered_map<int, or_protocol_msgs::Point> positions;
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    link_etx = link_etx_table;
    positions = node_positions;
  }

  or_protocol_msgs::NetworkStatus::Ptr msg(new or_protocol_msgs::NetworkStatus());

  for (const auto& point : positions)
    msg->positions.push_back(point.second);

  for (const auto& list : link_etx) {
    or_protocol_msgs::ETXList etx_list;
    etx_list.node = list.first;
    for (const auto& item : list.second)
      etx_list.etx_list.push_back(item.second);
    msg->etx_table.push_back(etx_list);
  }

  return msg;
}


or_protocol_msgs::RoutingTable::Ptr NetworkState::get_routing_table_msg(int root)
{
  FixedRoutingMap routing_map_copy;
  {
    std::lock_guard<std::mutex> lock(status_mutex);
    routing_map_copy = routing_map;
  }

  or_protocol_msgs::RoutingTable::Ptr msg(new or_protocol_msgs::RoutingTable());

  for (const auto& src_entry : routing_map_copy) {
    or_protocol_msgs::RoutingSrcEntry src_entry_msg;
    src_entry_msg.src_id = src_entry.first;
    for (const auto& dest_entry : src_entry.second) {
      or_protocol_msgs::RoutingDestEntry dest_entry_msg;
      dest_entry_msg.dest_id = dest_entry.first;

      or_protocol_msgs::RoutingRule rule;
      rule.relay_id = root;
      rule.relays = dest_entry.second;

      dest_entry_msg.rules.push_back(rule);
      src_entry_msg.entries.push_back(dest_entry_msg);
    }
    msg->entries.push_back(src_entry_msg);
  }

  return msg;
}


}  // namespace or_protocol

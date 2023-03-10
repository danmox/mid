#include <queue>
#include <deque>
#include <unordered_map>

#include <or_protocol/network_state.h>
#include <or_protocol/utils.h>


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


MsgStatus NetworkState::update_queue(const int node_id,
                                     const or_protocol_msgs::Header &header)
{
  return node_states[node_id].update_queue(header);
}


void NetworkState::ack_msg(const int node_id, const uint32_t seq)
{
  node_states[node_id].ack_msg(seq);
}


int NetworkState::priority(const int node_id, const int seq)
{
  return node_states[node_id].priority(seq);
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


RoutingMap NetworkState::find_routes(const int root)
{
  //
  // compute path_etx and default_paths
  //

  std::unordered_set<int> node_ids;
  for (const auto& id_map_pair : link_etx) {
    node_ids.insert(id_map_pair.first);
    for (const auto& id_etx_pair : id_map_pair.second)
      node_ids.insert(id_etx_pair.first);
  }
  node_ids.insert(root);

  // find the shortest path between each pair of nodes in the network
  ETXMap path_etx;
  RoutingMap default_paths;
  for (const int node : node_ids) {

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
    for (const int dest : node_ids)
      path_etx[node][dest] = cost[dest];

    // reconstruct default paths
    // NOTE assumes a connected network!!
    // TODO handle disconnected networks
    for (const int dest : node_ids) {
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
  for (const int s : node_ids) {
    for (const int d : node_ids) {
      for (const int n : node_ids) {
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
  RoutingMap new_routing_map;
  for (const int src : node_ids) {
    for (const int dest : node_ids) {
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

  return new_routing_map;
}


}  // namespace or_protocol

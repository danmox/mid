#include <or_protocol/node_state.h>
#include <or_protocol/utils.h>


namespace or_protocol {


bool NodeState::update_queue(const or_protocol_msgs::Header& header)
{
  if (msg_hist_map.size() == buff_capacity) {
    uint32_t smallest = msg_hist_deque.back();
    msg_hist_deque.pop_back();
    msg_hist_map.erase(smallest);
  }

  // compute node relay priority
  unsigned int priority = relay_priority(header.curr_id, header);

  bool first = false;
  if (msg_hist_map.count(header.seq) == 0) {
    first = true;
    msg_hist_deque.push_front(header.seq);
    msg_hist_map[header.seq] = priority;
  } else if (msg_hist_map[header.seq] > priority) { // smaller #s = higher priority
    msg_hist_map[header.seq] = priority;
  }

  return first;
}


int NodeState::priority(const int seq)
{
  return msg_hist_map[seq];
}


}  // namespace or_protocol

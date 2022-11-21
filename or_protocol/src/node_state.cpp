#include <or_protocol/node_state.h>


namespace or_protocol {


bool NodeState::update_queue(const or_protocol_msgs::Header& header)
{
  if (msg_hist_set.size() == buff_capacity) {
    uint32_t smallest = msg_hist_deque.back();
    msg_hist_deque.pop_back();
    msg_hist_set.erase(smallest);
  }

  bool first = false;
  if (msg_hist_set.count(header.seq) == 0) {
    first = true;
    msg_hist_deque.push_front(header.seq);
    msg_hist_set.insert(header.seq);
  }

  return first;
}


}  // namespace or_protocol

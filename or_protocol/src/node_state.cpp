#include <or_protocol/node_state.h>
#include <or_protocol/utils.h>


namespace or_protocol {


bool NodeState::update_queue(const or_protocol_msgs::Header& header)
{
  if (msg_hist_map.size() > MSG_BUFFER_CAPACITY) {
    SeqAttemptPair& oldest = msg_hist_deque.back();
    msg_hist_deque.pop_back();

    // remove the message from the map if no new attempts have been received
    if (oldest.attempt == msg_hist_map[oldest.seq].attempt)
      msg_hist_map.erase(oldest.seq);
  }

  unsigned int priority = relay_priority(header.curr_id, header);

  bool first = false;
  if (msg_hist_map.count(header.seq) == 0 ||
      header.attempt > msg_hist_map[header.seq].attempt) {
    first = true;
    msg_hist_deque.push_front(SeqAttemptPair{header.seq, header.attempt});
    msg_hist_map[header.seq] = AttemptPriorityPair{header.attempt, priority};
  } else if (msg_hist_map[header.seq].priority > priority) {
    msg_hist_map[header.seq] = AttemptPriorityPair{header.attempt, priority};
  }

  return first;
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


}  // namespace or_protocol

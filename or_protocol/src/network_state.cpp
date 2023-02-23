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


}  // namespace or_protocol

#ifndef OR_PROTOCOL_NODE_STATE_H_
#define OR_PROTOCOL_NODE_STATE_H_


#include <cstdint>
#include <deque>
#include <unordered_set>

#include <or_protocol_msgs/Header.h>


namespace or_protocol {


class NodeState
{
  public:
    bool update_queue(const or_protocol_msgs::Header& header);
    void update_stats(const or_protocol_msgs::Header& header);

  private:
    std::unordered_set<uint32_t> msg_hist_set;
    std::deque<uint32_t> msg_hist_deque;
    const size_t buff_capacity = 200;
};


}  // namespace or_protocol


#endif

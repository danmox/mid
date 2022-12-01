#include <or_protocol/utils.h>


namespace or_protocol {


int relay_priority(int id, const or_protocol_msgs::Header& header)
{
  size_t priority = 0;
  while (priority < header.relays.size() && header.relays[priority] != id)
    priority++;

  return priority;
}


} // namespace or_protocol

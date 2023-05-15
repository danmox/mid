#ifndef OR_PROTOCOL_TYPES_H_
#define OR_PROTOCOL_TYPES_H_


#include <or_protocol_msgs/Header.h>
#include <or_protocol_msgs/Packet.h>


namespace or_protocol {


// TODO switch to std::unique_ptr for better performance?
typedef std::unique_ptr<char[]> buffer_ptr;


typedef std::function<void(buffer_ptr&, int)> buff_recv_func;


struct PacketQueueItem
{
    buffer_ptr buff_ptr;
    size_t size;
    or_protocol_msgs::Header header;
    ros::Time recv_time, send_time;
    bool processed, retransmission;
    int priority, retries;

    PacketQueueItem(buffer_ptr& _buff_ptr,
                    size_t _size,
                    or_protocol_msgs::Header& _header,
                    ros::Time& now) :
      buff_ptr(std::move(_buff_ptr)),
      size(_size),
      header(_header),
      recv_time(now),
      send_time(0),
      processed(false),
      retransmission(false),
      priority(0),
      retries(0)
    {}

    char* buffer() { return buff_ptr.get(); }
};


typedef std::shared_ptr<PacketQueueItem> PacketQueueItemPtr;


struct AppQueueItem
{
    PacketQueueItemPtr queue_ptr;
    or_protocol_msgs::PacketPtr packet_ptr;

    AppQueueItem(const PacketQueueItemPtr& pqip) : queue_ptr(pqip) {}
    AppQueueItem(const or_protocol_msgs::PacketPtr& pp) : packet_ptr(pp) {}
};


typedef std::shared_ptr<AppQueueItem> AppQueueItemPtr;


enum class PacketAction
{
  RECEIVE,
  SEND,
  RELAY,
  RETRY,
  CANCEL_RETRY,
  DROP_SUP,
  DROP_DUP,
  DROP_ECHO,
  DROP_LOWPRI,
  QUEUE,
  ACK,
  DELIVER
};


}  // namespace or_protocol


#endif

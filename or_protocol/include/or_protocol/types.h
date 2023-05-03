#ifndef OR_PROTOCOL_TYPES_H_
#define OR_PROTOCOL_TYPES_H_


#include <or_protocol_msgs/Header.h>
#include <or_protocol_msgs/Packet.h>


namespace or_protocol {


typedef std::function<void(std::shared_ptr<char[]>&, int)> buff_recv_func;
typedef std::function<void(ros::Time, or_protocol_msgs::Packet&, int, int)> msg_recv_func;


// TODO switch to std::unique_ptr for better performance?
typedef std::shared_ptr<char[]> buffer_ptr;


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
      buff_ptr(_buff_ptr),
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

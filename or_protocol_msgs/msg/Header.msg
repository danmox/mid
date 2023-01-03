uint8 STATUS=1
uint8 PAYLOAD=2
uint8 PING_REQ=3
uint8 PING_RES=4
uint8 ACK=5

# the type of data encapsulated in the packet
uint8 msg_type

# packet identification information
uint8 src_id   # the source node IPv4 address digits identified by the subnet mask: 0.0.0.255
uint8 curr_id  # the transmitting node IPv4 address...
uint8 dest_id  # the destination node IPv4 address...
uint32 seq     # the unique (to the source node) packet sequence number

# packet trip information
uint8 hops       # relays traversed to current node
uint8[4] relays  # list of nodes that can act as relays, in order of decreasing priority
bool reliable    # if true, send acknowledgments
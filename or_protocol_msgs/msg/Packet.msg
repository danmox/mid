uint8 STATUS=1
uint8 PAYLOAD=2

# the type of data encapsulated in the packet
uint8 msg_type

# packet identification information
uint8 src_id  # the source node IPv4 address digits identified by the subnet mask: 0.0.0.255
uint8 curr_id # the transmitting node IPv4 address...
uint8 dest_id # the destination node IPv4 address...
uint32 seq    # the unique (to the source node) packet sequence number

# list of nodes that can act as relays, in order of decreasing priority
uint8[] relays

uint8[] data
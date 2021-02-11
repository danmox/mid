# network data flow specification consisting of a required bandwidth and
# confidence (probabilistic guarantee that the flow rate will be achieved)

string tx
string rx
float64 rate
float64 confidence  # Pr[actual_rate > rate] > confidence
LoRa msg format:
Example 1:
===========
"cb 12 0 4 r=2 6 E"

cb: controller to beacon
bc: beacon to controller
bb: beacon to beacon

12: lora msg sequence number

0: source node (0 for controller 1=< for beacons)
4: destination node (0 for controller 1=< for beacons)

r: command type ( "r" range, "s" uwb shout, "i" respond immediately etc.)

=:command params or command replies

2 6: range with 2 and 6

E: msg-end marker

Example 2:
===========
"bc 12 4 0 r=2 912 6 125 E"




# Scara
A revision/update of IVproject's SCARA arm
https://github.com/IVProjects/Engineering_Projects/tree/main

WIP

Currently only Code, Will update SCARA STLs when needed

code improvements:
Steppers now move simultaneously
Better UI in PyQT5
More functionality
  - end effector set vertical, horizontal, lock, free mode
  - WASD controls
  - more controls
  - sends data as a packet
  - can see controls
  - customizeable save/load waypoint file
  - able to see serial data
  - Arduino reconect
  - some error tolerance
  - works when arduino is not connected
  - better buffer handling
  - threading for reading serial data
  - threading for sending serial packets
  - waypoints generally work better and more accurately
  - Z height error handling
(single step Z motor if using 16to1 gearbox for Z)

Planned
  - Motor specification in data packet
  - time wait setting for waypoints
  - seperate code into multiple .py files
  - clean up code(it works)
  - remove chart
  - clearly seperate print and serial terminal outputs
  - have dot start in appropriate place and prevent div 0 crashes
  - have Z axis dialogue box work, or not crash program
  - Z axis controlled by up/down arrows.
  - update controls
  - reinitialize homing sequence from controller
  - settings for control adjustment
  - speed/accel adjustment from software
  - speed/accel integration into waypoints

# DC Control Module

## Message Packet
Addressed to | Set Angle for 1 | Current Angle for 2 |

## Problems
* Not sure why when activating SEND and GET, there are periods where messages are dropped.
	* Turns out that having the same ID will cause a bus conflict
	* Issuing different IDs allow a much higher speed to be achieved (1Mbit/s)
* Not sure why resetting one before the other will cause messages to be dropped for the first but not for the other.

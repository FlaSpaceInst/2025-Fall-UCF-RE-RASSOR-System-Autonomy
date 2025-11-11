Interfaces are used to define data types for communications between nodes in ROS2. This could be through the use of messages, services, or actions.

The messages created:
    Location Status - Used to describe location of anything, rovers or obstacles. Takes an x and y field in floats to denote the coordinates.

The services created:
    Obstacle Array - Sends an array of location coordinates (type location status detailed above) and receives a boolean value in response to confirm whether the information was received correctly by the server. Can be used to send over an array of obstacles located on map

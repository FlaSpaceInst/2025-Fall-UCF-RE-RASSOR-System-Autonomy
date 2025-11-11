#!/usr/bin/env python3
from functools import partial
import rclpy
import json
import os
import uuid
import threading
import subprocess
import time
from rclpy.node import Node
from pymongo import MongoClient #Install pymongo `pip install pymongo`
from bson.objectid import ObjectId
from re_rassor_interfaces.msg import ObstacleLocation
from re_rassor_interfaces.msg import ObstacleArray as ObstacleArrayMsg
from re_rassor_interfaces.srv import LocationStatus as LocationStatusMsg
from re_rassor_interfaces.srv import ObstacleArray as ObstacleArraySrv
from re_rassor_interfaces.srv import LocationStatus as LocationStatusSrv
from re_rassor_interfaces.srv import NewGoal
from re_rassor_interfaces.srv import FinalLocation 
from re_rassor_interfaces.srv import InitialRoverPosition
from re_rassor_interfaces.srv import InitialObstacleArray

# sample run: 'ros2 run re_rassor_mission_control mission_control --ros-args -p goal_x:=10.5 -p goal_y:=20.0 -p rover_id:=rover_001 -p obstacle_request_period:=30.0'

# Testing the communication from other modules using the following nodes 
# ros2 run re_rassor_mission_control final_location
# ros2 run re_rassor_mission_control obstacle_service
# ros2 run re_rassor_mission_control rover_location_service
# ros2 run re_rassor_mission_control map_data




# DONE - Need to create a client that takes in the ObstacleArray.srv sent from Cartography & Routing 
# DONE - Need to create a client that takes in Location data from the rover periodically (X, Y, time, and 'velocity'?)
# DONE - Need to create a service that takes in the final Rover Location before shutting down simulation

# DONE - Need to create an API that stores the new obstacles inside the DB (Should I check to see if the obstacle already exists before trying to store it?)
# DONE - Need to create an API that stores the Rover's location
# DONE - Need to create an API that stores the Final Location from the Rover
# Need to create an API that stores the planned pathing from the Rover

class MissionControlNode(Node):
    def __init__(self):
        super().__init__("mission_control")
        self.mongo_client = None

        # Declare parameters for goal position and rover ID
        self.declare_parameter('goal_x', 4.0)  # Default goal X coordinate
        self.declare_parameter('goal_y', 4.0)  # Default goal Y coordinate
        self.declare_parameter('rover_id', "rover_001")  # Default rover ID
        self.declare_parameter('obstacle_request_period', 2.0)  # Default to 2 seconds

        # Get parameters
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.rover_id = self.get_parameter('rover_id').value
        self.timer_period = self.get_parameter('obstacle_request_period').value
        self.amount_sent_over=0
        self.get_logger().info(f"Goal position: X={self.goal_x}, Y={self.goal_y}")
        self.get_logger().info(f"Rover ID: {self.rover_id}")

        # Services
       
        self.map_data_service = self.create_service(
            InitialObstacleArray, "map_data", self.handle_map_data_request
        )
        self.rover_initial_position_service = self.create_service(
            InitialRoverPosition, "rover_initial_position", self.handle_rover_initial_position_request
        )  
       
        self.final_location_service = self.create_service(FinalLocation, "final_location", self.store_final_location)
        self.new_obstacles_client = self.create_client(ObstacleArraySrv, "obstacle_array")
        self.rover_location_client = self.create_client(LocationStatusSrv, "update_rover_location")
        self.new_waypoints_client = self.create_service(NewGoal, "new_goal_waypoints", self.handle_new_waypoints)
        
        # MongoDB setup and initialization
        self.initial_map_id_, self.obstacle_list_, self.rover_location_ = self.connect_to_mongo()
        
        # Checks if new obstacles have been detected for a given period of time
        self.timer = self.create_timer(self.timer_period, self.request_new_obstacles)
        # Calls for location data every 5 seconds
        self.timer = self.create_timer(5.0, self.request_rover_location)
        
        # Flag to represent whether new waypoints were received 
        self.new_waypoints = False

    def handle_map_data_request(self, request, response):
        """Handle the map data service request."""
        try:
            obstacle_array_msg = ObstacleArrayMsg()
            obstacle_array_msg.obstacles = []

            for obstacle in self.obstacle_list_:
                obstacle_msg = ObstacleLocation()
                obstacle_msg.x = float(obstacle["x"])
                obstacle_msg.y = float(obstacle["y"])
                obstacle_array_msg.obstacles.append(obstacle_msg)

            response.obstacles = obstacle_array_msg.obstacles
            self.get_logger().info(f"Sent map data with {len(obstacle_array_msg.obstacles)} obstacles.")
            self.get_logger().info(str(obstacle_array_msg.obstacles))
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to handle map data request: {e}")
            response.success = False
            return response
        
    def handle_rover_initial_position_request(self, request, response):
        """Handle the rover initial position service request."""
        try:
            response.position_x = float(self.rover_location_["x"])
            response.position_y = float(self.rover_location_["y"])
            response.time = float(self.rover_location_["time"])
            response.goal_x = float(self.goal_x)
            response.goal_y = float(self.goal_y)
            self.amount_sent_over+=1
            self.get_logger().info(
                f"Sent initial rover position: X={response.position_x}, Y={response.position_y}, Time={response.time}, "
                f"Goal_X={response.goal_x}, Goal_Y={response.goal_y}, Number of times ={self.amount_sent_over}"
            )
            
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to handle rover initial position request: {e}")
            response.success = False
            return response

    def connect_to_mongo(self):
        """Connect to MongoDB, retrieve rover and map data, and initialize publishers."""
        try:
            # Connect to MongoDB
            mongo_uri = "mongodb+srv://seniordesignl18:ReRassorDatabase@rerassor.wpcf5.mongodb.net/"
            self.mongo_client = MongoClient(mongo_uri)
            self.get_logger().info("Connected to MongoDB")

            # Retrieve the rover data using the rover_id parameter
            rover_id = self.get_parameter('rover_id').value
            db = self.mongo_client["RE-RASSOR"]
            rover_collection = db["Rovers"]
            map_collection = db["Maps"]

            # Check if the rover exists in the database
            rover_data = rover_collection.find_one({"rover_id": rover_id})
            if not rover_data:
                self.get_logger().warn(f"No rover data found for rover_id={rover_id}")

                # Retrieve the first map from the database
                first_map = map_collection.find_one({})
                if not first_map:
                    self.get_logger().warn("No maps found in the database. Creating a new map...")

                    # Generate a random map_id and create a new map entry
                    new_map_id = f"map_{uuid.uuid4().hex[:8]}" 
                    new_map_data = {
                        "map_id": new_map_id,
                        "obstacles": []
                    }
                    map_collection.insert_one(new_map_data)
                    self.get_logger().info(f"Created new map with map_id={new_map_id}")

                    first_map = new_map_data

                # Create a new rover entry in the database
                new_rover_data = {
                    "rover_id": rover_id,
                    "rover_location": {"x": "0.0", "y": "0.0", "time": "0.0", "velocity": "0.0", "orientation" : "0.0"},
                    "traversal_map": first_map.get("map_id"),
                    "rover_pathing": []
                }
                rover_collection.insert_one(new_rover_data)
                self.get_logger().info(f"Created new rover entry for rover_id={rover_id} with map_id={first_map.get('map_id')}")
                rover_data = new_rover_data 

            self.get_logger().info(f"Fetched rover data: {rover_data}")

            # Extract the rover's initial position
            rover_location = rover_data.get("rover_location")
            if not rover_location:
                self.get_logger().warn(f"No rover location found for rover_id={rover_id}")
                return None, None, None

            # Extract the map_id and fetch the map data
            map_id = rover_data.get("traversal_map")
            if not map_id:
                self.get_logger().warn(f"Rover data for rover_id={rover_id} does not contain a map_id")
                return None, None, None

            map_id, obstacle_list = self.fetch_map_data(map_id)
            return map_id, obstacle_list, rover_location
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MongoDB: {e}")
            self.mongo_client = None
            return None, None, None

    def fetch_map_data(self, map_id):
        """Fetch map data for a specific map_id from the MongoDB 'Maps' collection."""
        if not self.mongo_client:
            self.get_logger().error("MongoDB client is not connected")
            return None, None

        try:
            db = self.mongo_client["RE-RASSOR"]
            collection = db["Maps"]

            # Query for the specific map_id
            query = {"map_id": map_id}  
            map_data = collection.find_one(query) 

            if map_data:
                self.get_logger().info(f"Fetched map record for map_id={map_id}")
                return self.process_map_data([map_data])
            else:
                self.get_logger().warn(f"No map record found for map_id={map_id}")
                return None, None
        except Exception as e:
            self.get_logger().error(f"Failed to fetch data from MongoDB: {e}")
            return []
      
    def process_map_data(self, map_records):
        """Processes the map data fetched from MongoDB."""
        if not map_records:
            print("No map records found.")
            return None, None

        # Assuming only one record is fetched
        map_data = map_records[0]

        # Extract the map_id and obstacles
        map_id = str(map_data.get('map_id'))
        obstacles = map_data.get('obstacles', [])

        # Log or process further
        print(f"Map ID: {map_id}")
        print(f"Obstacles: {obstacles}")

        return map_id, obstacles
    
    def request_new_obstacles(self):
        """Request new obstacles from the Cartography module."""
        if not self.new_obstacles_client.service_is_ready():
            self.get_logger().warn("ObstacleArray service not available, waiting...")
            return

        self.get_logger().info("Requesting new obstacles...")
        request = ObstacleArraySrv.Request() 
        future = self.new_obstacles_client.call_async(request)
        future.add_done_callback(self.process_new_obstacles)

    def process_new_obstacles(self, future):
        """Callback to process the received obstacle array."""
        try:
            response = future.result()
            new_obstacles = response.obstacles
            self.get_logger().info(f"Received {len(new_obstacles)} new obstacles.")

            # Store unique obstacles in the database
            unique_obstacles = self.store_unique_obstacles(new_obstacles)
            self.get_logger().info(f"Stored {len(unique_obstacles)} unique obstacles in the database.")
        except Exception as e:
            self.get_logger().error(f"Failed to receive obstacles: {e}")

    def store_unique_obstacles(self, obstacles):
        """Store unique obstacles in the obstacles array for the current map ID in MongoDB."""
        if not self.mongo_client:
            self.get_logger().error("MongoDB client is not connected.")
            return []

        try:
            db = self.mongo_client["RE-RASSOR"]
            collection = db["Maps"]

            # Fetch the current obstacles for the map
            current_map = collection.find_one({"map_id": self.initial_map_id_})
            if not current_map:
                self.get_logger().error(f"Map with ID {self.initial_map_id_} not found.")
                return []

            current_obstacles = current_map.get("obstacles", [])
            current_obstacles_set = {(float(o["x"]), float(o["y"])) for o in current_obstacles}

            unique_obstacles = []
            for obstacle in obstacles:
                # Create a tuple to match existing obstacles
                new_obstacle_tuple = (float(obstacle.x), float(obstacle.y))
                if new_obstacle_tuple not in current_obstacles_set:
                    unique_obstacles.append({"x": float(obstacle.x), "y": float(obstacle.y)})

            if unique_obstacles:
                # Add unique obstacles to the database
                result = collection.update_one(
                    {"map_id": self.initial_map_id_},
                    {"$push": {"obstacles": {"$each": unique_obstacles}}}
                )

                if result.modified_count > 0:
                    self.get_logger().info(f"Added {len(unique_obstacles)} new obstacles to map {self.initial_map_id_}.")

            return unique_obstacles
        except Exception as e:
            self.get_logger().error(f"Failed to store obstacles in MongoDB: {e}")
            return []

    def request_rover_location(self):
        """Request updated location data from the rover."""
        while not self.rover_location_client.wait_for_service(10):
            self.get_logger().warn("RoverLocation service not available, waiting...")
            return

        self.get_logger().info("Requesting updated rover location...")
        request = LocationStatusSrv.Request()
        future = self.rover_location_client.call_async(request)
        future.add_done_callback(partial(self.process_rover_location))
    
    def process_rover_location(self, future):
        """Callback to process the received rover location."""
        try:
            response = future.result()
            self.get_logger().info("Checing to see what the repsonse for location updates is...")
            self.get_logger().info(str(response))
            updated_location = {
                "x": response.x,
                "y": response.y,
                "time": response.time,
                "velocity": response.velocity,
                "orientation": response.orientation
            }
            self.get_logger().info(f"Received updated rover location: {updated_location}")

            # Store the updated location in the database
            self.store_updated_location(updated_location)
        except Exception as e:
            self.get_logger().error(f"Failed to receive rover location: {e}")

    def store_updated_location(self, updated_location):
        """Store the updated rover location in MongoDB."""
        if not self.mongo_client:
            self.get_logger().error("MongoDB client is not connected.")
            return

        try:
            db = self.mongo_client["RE-RASSOR"]
            collection = db["Rovers"]

            # Update the rover location for the current rover ID
            result = collection.update_one(
                {"rover_id": self.rover_id}, 
                {"$set": {"rover_location": updated_location}} 
            )

            if result.modified_count > 0:
                self.get_logger().info(f"Updated location for rover_id {self.rover_id}: {updated_location}")
            else:
                self.get_logger().warn(f"No updates made. Rover ID {self.rover_id} may not exist.")
        except Exception as e:
            self.get_logger().error(f"Failed to update rover location in MongoDB: {e}")

    def store_final_location(self, request, response):
        """Callback to handle the final location storage."""
        if not self.mongo_client:
            self.get_logger().error("MongoDB client is not connected.")
            response.success = False
            response.message = "MongoDB client is not connected."
            return response

        try:
            db = self.mongo_client["RE-RASSOR"]
            rover_collection = db["Rovers"]

            # Update the rover's location as the final location
            result = rover_collection.update_one(
                {"rover_id": self.rover_id},
                {"$set": {
                    "rover_location": {
                        "x": request.x,
                        "y": request.y,
                        "time": request.time,
                        "velocity": request.velocity,
                        "orientation": request.orientation
                    }
                }}
            )

            if result.modified_count > 0:
                self.get_logger().info(f"Final location updated for rover_id {self.rover_id}: ({request.x}, {request.y}, {request.time}, {request.velocity}, {request.orientation})")
                response.success = True
                response.message = "Final location stored successfully."
            else:
                self.get_logger().warn(f"No updates made. Rover ID {self.rover_id} may not exist.")
                response.success = False
                response.message = f"Rover ID {self.rover_id} may not exist."
        except Exception as e:
            self.get_logger().error(f"Failed to store final location in MongoDB: {e}")
            response.success = False
            response.message = str(e)

        return response
    
    def handle_new_waypoints(self, request, response):
        try:
            if self.new_waypoints:
                response.x = 48.0
                response.y = 48.0
                response.new_waypoints_available = True
                self.get_logger().info(f"Sending new waypoints to Cartography: X={response.x}, Y={response.y}")
            else:
                # Open a new terminal for user input using terminator
                self.get_logger().info("No waypoints available. Prompting user for input...")

                # Use a temp file to store the user's response
                temp_file = "/tmp/waypoint_input.txt"
                
                try:
                    if os.path.exists(temp_file):
                        os.remove(temp_file) 

                    script_path = os.path.join(os.path.dirname(__file__), "user_input_prompt.py")
                    subprocess.Popen(["terminator", "-e", f"python3 {script_path} {temp_file}"])

                    # Wait for user input (polling method)
                    timeout = 60  # Maximum wait time in seconds
                    start_time = time.time()
                    while not os.path.exists(temp_file):
                        if time.time() - start_time > timeout:
                            self.get_logger().error("User input timeout exceeded. No response received.")
                            response.x = 0.0
                            response.y = 0.0
                            response.new_waypoints_available = False
                            return response
                        time.sleep(1) 

                    # Read the user input from the temp file
                    with open(temp_file, "r") as file:
                        data = file.read().strip()
                        x, y = map(float, data.split(","))

                    response.x = x
                    response.y = y
                    response.new_waypoints_available = True
                    self.get_logger().info(f"New waypoints set by user: X={response.x}, Y={response.y}")
                except Exception as e:
                    self.get_logger().error(f"Failed to set new waypoints: {e}")
                    response.x = 0.0
                    response.y = 0.0
                    response.new_waypoints_available = False
        except Exception as e:
            self.get_logger().error(f"Unable to send waypoints: {e}")
            response.x = 0.0
            response.y = 0.0
            response.new_waypoints_available = False

        self.new_waypoints = False
        return response
        
        
def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
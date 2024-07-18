import sys
from time import sleep
from threading import Thread
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from project_interfaces.action import Patrol


NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])

HOVERING_HEIGHT = 15.0
VULNERABLE_SENSOR_IDS = [0, 1]  # IDs of the two most vulnerable sensors

class FleetCoordinator(Node):
    """
    Fleet Coordinator class, used to manage the whole fleet of Balloons and Drones.
    This is where most of the tasks should be submitted by default.
    """
    
    def __init__(self):
        super().__init__('fleet_coordinator')

        self.balloon_action_clients = {}
        self.sensor_positions = {}
        self.balloon_states = {}

        for i in range(NUMBER_OF_BALLOONS):
            self.balloon_action_clients[i] = ActionClient(
                self,
                Patrol,
                f'/Balloon_{i}/patrol'
            )
            self.balloon_states[i] = BalloonState.LANDED

        for i in range(NUMBER_OF_SENSORS):
            self.create_subscription(
                Odometry,
                f'Sensor_{i}/odometry',
                lambda msg, id=i: self.store_sensor_position(id, msg),
                10
            )

    def patrol_targets(self):
        """
        Method used to keep the fleet of Balloons constantly patrolling the set of targets.
        When a patrolling task has been completed, a new one with the same targets is given again.
        """

        def patrol_targets_inner():
            while True:
                for i in range(NUMBER_OF_BALLOONS):
                    # Do not resubmit tasks to already moving balloons
                    if not self.balloon_states[i] is BalloonState.MOVING:
                        self.submit_task(i)
            
        # Start this function in another thread, so that the node can start spinning immediately after
        # this function has finished
        Thread(target=patrol_targets_inner).start()

    def submit_task(self, uav_id: int):
        # Wait for the action server to go online
        while not self.balloon_action_clients[uav_id].wait_for_server(1) or len(self.sensor_positions) < NUMBER_OF_SENSORS:
###            self.get_logger().info(f"NUMBER OF SENSORS {len(self.sensor_positions)}")
            self.get_logger().info("Waiting for action server to come online and sensors to announce their position")
            sleep(3)

        # Set the Balloon to moving state
        self.balloon_states[uav_id] = BalloonState.MOVING
        goal = Patrol.Goal()
        goal.targets = []

        if uav_id == 0:
            # Hover over the center of the leftmost square
            center_leftmost_square = Point(x=-30.0, y=30.0, z=HOVERING_HEIGHT)
            goal.targets.append(center_leftmost_square)
        else:
            # Circulate over all other sensors except the two most vulnerable ones
            non_vulnerable_sensors = [id for id in self.sensor_positions if id not in VULNERABLE_SENSOR_IDS]
            non_vulnerable_sensors.sort()
            for target in range(len(non_vulnerable_sensors)):
                actual_target = (target + uav_id) % len(non_vulnerable_sensors)
                goal.targets.append(self.sensor_positions[non_vulnerable_sensors[actual_target]])
                goal.targets[-1].z = HOVERING_HEIGHT #############################+ (uav_id * 8.0)
                
###        self.get_logger().info(f"Submitting task for Balloon {uav_id}")

        # Submit the task here and add a callback for when the submission is accepted
        patrol_future = self.balloon_action_clients[uav_id].send_goal_async(goal)
        patrol_future.add_done_callback(lambda future, uav_id=uav_id: self.patrol_submitted_callback(uav_id, future))

    def patrol_submitted_callback(self, uav_id, future):
        # Check if the patrol action was accepted
        goal_handle = future.result()
    
        if not goal_handle.accepted:
            # If not, set the balloon back to hovering, and return
###            self.get_logger().info("Task has been refused by the action server")
            self.balloon_states[uav_id] = BalloonState.HOVERING
            return
        
###        self.get_logger().info(f"Task for Balloon {uav_id} has been accepted")
        result_future = goal_handle.get_result_async()

        # Add a callback for when the action is completed
        result_future.add_done_callback(lambda future, uav_id=uav_id: self.patrol_completed_callback(uav_id, future))

    def patrol_completed_callback(self, uav_id, future):
        # Action completed, Balloon can go back to hovering. Note that we don't check if the patrol was correctly completed,
        # you may have to handle such cases
###        self.get_logger().info(f"Patrolling action for Balloon {uav_id} has been completed. Drone is going idle")
        self.balloon_states[uav_id] = BalloonState.HOVERING

    def store_sensor_position(self, id, msg: Odometry):
        self.sensor_positions[id] = msg.pose.pose.position

class BalloonState(Enum):
    LANDED = 1
    HOVERING = 2
    MOVING = 3

def main():
    rclpy.init()

    executor = MultiThreadedExecutor()
    fleet_coordinator = FleetCoordinator()

    executor.add_node(fleet_coordinator)
    fleet_coordinator.patrol_targets()

    executor.spin()

    executor.shutdown()
    fleet_coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

























#import sys
#from time import sleep
#from threading import Thread
#from enum import Enum

#import rclpy
#from rclpy.node import Node
#from rclpy.action import ActionClient
#from rclpy.executors import MultiThreadedExecutor

#from geometry_msgs.msg import Point
#from project_interfaces.action import Patrol
#from nav_msgs.msg import Odometry

#NUMBER_OF_BALLOONS = int(sys.argv[1])
#NUMBER_OF_SENSORS = int(sys.argv[2])
#HOVERING_HEIGHT = 15.0

#class FleetCoordinator(Node):
#    """
#    Fleet Coordinator class, used to manage the whole fleet of Balloons and Drones.
#    This is where most of the tasks should be submitted by default.
#    """
#    
#    def __init__(self):
#        super().__init__('fleet_coordinator')
#        
#        self.sensor_positions = {}      
#        self.balloon_action_clients = {}
#        self.balloon_states = {}
#        self.hovering_positions = calculate_balloon_coordinates(NUMBER_OF_BALLOONS)

#        for i in range(NUMBER_OF_BALLOONS):
#            self.balloon_action_clients[i] = ActionClient(
#                self,
#                Patrol,
#                f'/Balloon_{i}/patrol'
#            )
#            self.balloon_states[i] = BalloonState.LANDED

#        for i in range(NUMBER_OF_SENSORS):
#            self.create_subscription(
#                Odometry,
#                f'Sensor_{i}/odometry',
#                lambda msg, id = i : self.store_sensor_position(id, msg),
#                10
#            )
#        
#        Thread(target=self.submit_initial_tasks).start()

#    def submit_initial_tasks(self):
#        """
#        Method used to initially submit the patrolling tasks for all balloons.
#        """
#        for i in range(NUMBER_OF_BALLOONS):
#        	if not self.balloon_states[i] is BalloonState.MOVING:
#        		self.submit_task(i)

#    def submit_task(self, uav_id: int):
#        # Wait for the action server to go online
#        while not self.balloon_action_clients[uav_id].wait_for_server(1) or len(self.sensor_positions) < NUMBER_OF_SENSORS:
#            self.get_logger().info("Waiting for world server to come online")
#            sleep(3)

#        # Set the Balloon to moving state
#        self.balloon_states[uav_id] = BalloonState.MOVING
#        goal = Patrol.Goal()
#        goal.targets = [self.hovering_positions[uav_id]]

#        self.get_logger().info(f"Submitting task for Balloon {uav_id} to hover at {self.hovering_positions[uav_id]}")

#        # Submit the task here and add a callback for when the submission is accepted
#        patrol_future = self.balloon_action_clients[uav_id].send_goal_async(goal)
#        patrol_future.add_done_callback(lambda future, uav_id=uav_id: self.patrol_submitted_callback(uav_id, future))

#    def patrol_submitted_callback(self, uav_id, future):
#        # Check if the patrol action was accepted
#        goal_handle = future.result()

#        if not goal_handle.accepted:
#            # If not, set the balloon back to hovering, and return
#            self.get_logger().info("Task has been refused by the action server")
#            self.balloon_states[uav_id] = BalloonState.HOVERING
#            return

#        result_future = goal_handle.get_result_async()

#        # Add a callback for when the action is completed
#        result_future.add_done_callback(lambda future, uav_id=uav_id: self.patrol_completed_callback(uav_id, future))

#    def patrol_completed_callback(self, uav_id, future):
#        # Action completed, Balloon can go back to hovering. Note that we don't check if the patrol was correctly completed,
#        # you may have to handle such cases
#        self.get_logger().info(f"Patrolling action for Balloon {uav_id} has been completed. Balloon is going idle")
#        self.balloon_states[uav_id] = BalloonState.HOVERING

#    def store_sensor_position(self, id, msg : Odometry):
#        self.sensor_positions[id] = msg.pose.pose.position

#class BalloonState(Enum):
#    LANDED = 1
#    HOVERING = 2
#    MOVING = 3

#def calculate_balloon_coordinates(num_balloons, x_range=(-40, 40), y_range=(-40, 40)):
#        x_min, x_max = x_range
#        y_min, y_max = y_range

#        # Divide balloons into two rows
#        half = (num_balloons + 1) // 2  # Ceiling of half
#        row1_balloons = half
#        row2_balloons = num_balloons - half

#        # Calculate x coordinates for each row
#        def calculate_x_coords(num_balloons, x_min, x_max):
#            segment_width = (x_max - x_min) / num_balloons
#            x_coords = [x_min + segment_width * (i + 0.5) for i in range(num_balloons)]
#            return x_coords

#        x_coords_row1 = calculate_x_coords(row1_balloons, x_min, x_max)
#        x_coords_row2 = calculate_x_coords(row2_balloons, x_min, x_max)
#        x_coords = x_coords_row1 + x_coords_row2

#        # Y coordinates for the two rows
#        y_coords_row1 = [y_max / 2] * row1_balloons
#        y_coords_row2 = [y_min / 2] * row2_balloons
#        y_coords = y_coords_row1 + y_coords_row2

#        # Combine coordinates
#        coordinates = [Point(x=x, y=y, z=HOVERING_HEIGHT+8*i) for i, (x, y) in enumerate(zip(x_coords, y_coords))]

#        return coordinates

#def main():
#    rclpy.init()

#    executor = MultiThreadedExecutor()
#    fleet_coordinator = FleetCoordinator()

#    executor.add_node(fleet_coordinator)
#    executor.spin()

#    executor.shutdown()
#    fleet_coordinator.destroy_node()
#    rclpy.shutdown()
#    
    
    
#import sys
#from time import sleep
#from threading import Thread
#from enum import Enum

#import rclpy
#from rclpy.node import Node
#from rclpy.action import ActionClient
#from rclpy.executors import MultiThreadedExecutor

#from geometry_msgs.msg import Point
#from nav_msgs.msg import Odometry
#from project_interfaces.action import Patrol


#NUMBER_OF_BALLOONS = int(sys.argv[1])
#NUMBER_OF_SENSORS = int(sys.argv[2])

#HOVERING_HEIGHT = 15.0


#class FleetCoordinator(Node):

#    """
#    Fleet Coordinator class, used the manage the whole fleet of Balloons and Drones.
#    This is where most of the task should be submitted by default.
#    """
#    
#    def __init__(self):

#        super().__init__('fleet_coordinator')

#        self.balloon_action_clients = {}
#        self.sensor_positions = {}      
#        self.balloon_states = {} 

#        for i in range(NUMBER_OF_BALLOONS):

#            self.balloon_action_clients[i] = ActionClient(
#                    self,
#                    Patrol,
#                    f'/Balloon_{i}/patrol'
#                )
#            
#            self.balloon_states[i] = BalloonState.LANDED
#            


#        for i in range(NUMBER_OF_SENSORS):
#            self.create_subscription(
#                Odometry,
#                f'ActiveSensor_{i}/odometry',
#                lambda msg, id = i : self.store_sensor_position(id, msg),
#                10
#            )




#    def patrol_targets(self):

#        """
#        Method used to keep the fleet of Balloons constantly patrolling the set of targets.
#        When a patrolling task has been completed, a new one with the same targets is given again.
#        """

#        def patrol_targets_inner():

#            while True:
#                for i in range(NUMBER_OF_BALLOONS):
#                    # Do not resubmit tasks to already moving balloons
#                    if not self.balloon_states[i] is BalloonState.MOVING:
#                        self.submit_task(i)
#            
#        # Start this function in another thread, so that the node can start spinning immediately after
#        # this function has finished
#        Thread(target=patrol_targets_inner).start()


#    def submit_task(self, uav_id : int):

#        # Wait for the action server to go online
#        while not self.balloon_action_clients[uav_id].wait_for_server(1) or len(self.sensor_positions) < NUMBER_OF_SENSORS:
#            self.get_logger().info(f"NUMBER OF SENSORS {len(self.sensor_positions)}")
#            self.get_logger().info("Waiting for action server to come online and sensors to announce their position")
#            sleep(3)

#        # Set the Balloon to moving state
#        self.balloon_states[uav_id] = BalloonState.MOVING
#        goal = Patrol.Goal()
#        goal.targets = []

#        # Just iterate through all the sensors in a round-robin fashion
#        for target in range(len(self.sensor_positions)):

#            # Start from the sensor with the id equal to the balloon id
#            actual_target = (target + uav_id) % len(self.sensor_positions)
#            goal.targets.append(self.sensor_positions[actual_target])

#            # Set the height to a predefined value, as the sensors will be on the ground, we want to hover them
#            goal.targets[-1].z = HOVERING_HEIGHT + (uav_id * 8.0)



#        self.get_logger().info(f"Submitting task for Balloon {uav_id}")

#        # Submit the task here and add a callback for when the submission is accepted
#        patrol_future = self.balloon_action_clients[uav_id].send_goal_async(goal)
#        patrol_future.add_done_callback(lambda future, uav_id = uav_id : self.patrol_submitted_callback(uav_id, future))


#    def patrol_submitted_callback(self, uav_id, future):

#        # Check if the patrol action was accepted
#        goal_handle = future.result()
#    
#        if not goal_handle.accepted:
#            # If not, set the balloon back to hovering, and return
#            self.get_logger().info("Task has been refused by the action server")
#            self.balloon_states[uav_id] = BalloonState.HOVERING
#            return
#        
#        result_future = goal_handle.get_result_async()

#        # Add a callback for when the action is completed
#        result_future.add_done_callback(lambda future, uav_id = uav_id : self.patrol_completed_callback(uav_id, future))


#    def patrol_completed_callback(self, uav_id, future):

#        # Action completed, Balloon can go back to hovering. Note that we don't check if the patrol was correctly completed,
#        # you may have to handle such cases
#        self.get_logger().info(f"Patrolling action for Balloon {uav_id} has been completed. Drone is going idle")
#        self.balloon_states[uav_id] = BalloonState.HOVERING


#    def store_sensor_position(self, id, msg : Odometry):
#        self.sensor_positions[id] = msg.pose.pose.position



#class BalloonState(Enum):
#    LANDED = 1
#    HOVERING = 2
#    MOVING = 3


#def main():

#    rclpy.init()

#    executor = MultiThreadedExecutor()
#    fleet_coordinator = FleetCoordinator()

#    executor.add_node(fleet_coordinator)
#    fleet_coordinator.patrol_targets()

#    executor.spin()

#    executor.shutdown()
#    fleet_coordinator.destroy_node()
#    rclpy.shutdown()

    





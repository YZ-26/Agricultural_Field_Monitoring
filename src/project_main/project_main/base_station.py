import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rosgraph_msgs.msg import Clock

from project_interfaces.srv import DataRequest  # Adjust the import path based on your package structure
from sim_utils import EventScheduler

WORLD_NAME = "iot_project_world"
REQUEST_INTERVAL = 30.0  # Time in seconds between requests for each sensor
NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])

class BaseStation(Node):

    def __init__(self):
        super().__init__('base_station2')

        self.sensor_index = 0
        self.current_simulation_time = 0
        self.success_received = False
        self.initial_request = True
        self.delivery_times = []  # Cache to store delivery times
        self.request_count = 0  # Counter to track number of requests

        # Create a service client for each balloon
        self.balloon_service_clients = []
        for i in range(NUMBER_OF_BALLOONS):
            client = self.create_client(DataRequest, f'/Balloon_{i}/data_request')
            self.balloon_service_clients.append(client)

        self.clock_subscriber = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.update_simulation_time,
            10
        )

        self.data_responses = {}
        self.pending_requests = 0
        self.start_time = None
        self.event_scheduler = EventScheduler()
        self.event_scheduler.schedule_event(REQUEST_INTERVAL, self.request_sensor_data, repeat=True)

    def update_simulation_time(self, clock_msg: Clock):
        self.current_simulation_time = clock_msg.clock.sec + clock_msg.clock.nanosec * 10**(-9)
        self.event_scheduler.routine(clock_msg)  # Execute scheduled events based on the new simulation time

    def request_sensor_data(self):
        if self.success_received:
            self.sensor_index = (self.sensor_index + 1) % NUMBER_OF_SENSORS
            self.success_received = False
            self.initial_request = True

        if self.initial_request:
            self.get_logger().info(f"Requesting data for sensor {self.sensor_index} at time {self.current_simulation_time}")
            self.start_time = self.current_simulation_time
            self.initial_request = False
        else:
            self.get_logger().info(f"Retrying data request for sensor {self.sensor_index} at time {self.current_simulation_time}")

        self.pending_requests = len(self.balloon_service_clients)

        for i, client in enumerate(self.balloon_service_clients):
            if client.service_is_ready() and not self.success_received:
                request = DataRequest.Request()
                request.sensor_id = self.sensor_index
                future = client.call_async(request)
                future.add_done_callback(lambda future, balloon_index=i: self.handle_response(future, balloon_index))
            else:
                self.get_logger().warning(f"Service {client.srv_name} is not ready or request already successful")
                self.pending_requests -= 1  # Reduce count for unsuccessful attempts

    def handle_response(self, future, balloon_index):
        response = future.result()
        if response.success:
            self.success_received = True
            sensor_data = response.data
            end_time = self.current_simulation_time
            self.data_responses[sensor_data] = end_time
            self.get_logger().info(f"Received data from Balloon {balloon_index}: {sensor_data} at time {end_time}")
            self.measure_delivery_time(sensor_data)
#        else:
#            self.get_logger().info("\n")
#            self.get_logger().warning(f"Failed to receive data from Balloon {balloon_index}")

        self.pending_requests -= 1
        if self.pending_requests == 0 and not self.success_received:
            self.get_logger().info("No successful responses received, retrying...")
            self.event_scheduler.schedule_event(5.0, self.request_sensor_data, repeat=False)

    def measure_delivery_time(self, sensor_data):
        if sensor_data in self.data_responses:
            end_time = self.data_responses[sensor_data]
            delivery_time = end_time - self.start_time
            self.delivery_times.append(delivery_time)
            self.request_count += 1

            if self.sensor_index == NUMBER_OF_SENSORS-1:
                self.log_delivery_times()

    def log_delivery_times(self):    
        average_delivery_time = sum(self.delivery_times) / len(self.delivery_times)
        self.get_logger().info(f"Average delivery time for {NUMBER_OF_SENSORS} sensors: {average_delivery_time} seconds\n...\n...\n...")

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()

    base_station = BaseStation()
    executor.add_node(base_station)

    executor.spin()

    base_station.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


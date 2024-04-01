from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from backend_server.logic.map import MapData


class MapPublisher(Node):
    newMapAvailable = False

    def __init__(self):
        super().__init__('map_publisher')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, occupancy_grid: OccupancyGrid):
        base_64_map_data = self.convertDataToBase64Str(occupancy_grid)
        self.base_64_map_img = f'data:image/bmp;base64,{base_64_map_data}'
        self.newMapAvailable = True
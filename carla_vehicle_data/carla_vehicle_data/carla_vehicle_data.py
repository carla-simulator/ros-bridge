import rclpy
from rclpy.node import Node
import carla
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaActorList, CarlaActorInfo
from derived_object_msgs.msg import ObjectArray

class CarlaVehicleDataPublisher(Node):
    def __init__(self):
        super().__init__('carla_vehicle_data_publisher')
        self.vehicle_roles = {}

        # Initialize CARLA client
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)

        # Publisher for vehicle list
        self.publisher = self.create_publisher(CarlaActorList, '/carla/vehicles', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Subscriber for vehicles (including 'hero')
        self.objects_sub = self.create_subscription(ObjectArray, '/carla/objects', self.objects_callback, 10)

    def timer_callback(self):
        world = self.client.get_world()
        vehicles = world.get_actors().filter('vehicle.*')

        actor_list_msg = CarlaActorList()
        for vehicle in vehicles:
            actor_info = CarlaActorInfo()
            actor_info.id = vehicle.id
            actor_info.type = vehicle.type_id
            actor_info.rolename = vehicle.attributes.get('role_name', 'unknown')
            actor_list_msg.actors.append(actor_info)

            # Update vehicle_roles for object callback use
            self.vehicle_roles[vehicle.id] = actor_info.rolename

        self.publisher.publish(actor_list_msg)

    def objects_callback(self, msg):
        for object in msg.objects:
            role_name = self.vehicle_roles.get(object.id, 'unknown')
            if role_name in ['hero', 'FollowCar', 'LeadCar']:
                x = object.pose.position.x
                y = object.pose.position.y
                z = object.pose.position.z
                accel_y = object.accel.linear.y
                vehicle_data = f'[{role_name}] [x={x}, y={y}, z={z}, accel_y={accel_y}]'
                self.get_logger().info(vehicle_data)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaVehicleDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

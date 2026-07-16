import rclpy

from carla_map_visualizer.carla_map_visualizer import CarlaMapVisualizer

def main(args=None):
    rclpy.init(args=args)

    # create nodes
    carla_map_visualizer = CarlaMapVisualizer()

    # create executor and add nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    if(not executor.add_node(carla_map_visualizer)):
        carla_map_visualizer.get_logger().error(
            "Can't add carla_map_visualizer to executor")

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt. Shutting down.")

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

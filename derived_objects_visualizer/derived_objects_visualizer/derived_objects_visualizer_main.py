import rclpy

from derived_objects_visualizer.derived_objects_visualizer import DerivedObjectsVisualizer


def main(args=None):
    rclpy.init(args=args)

    # create nodes
    derived_objects_visualizer = DerivedObjectsVisualizer()

    # create executor and add nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    if(not executor.add_node(derived_objects_visualizer)):
        derived_objects_visualizer.get_logger().error(
            "Can't add derived_objects_visualizer to executor")

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt. Shutting down.")

    derived_objects_visualizer.before_shutdown()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

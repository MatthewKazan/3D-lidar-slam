import multiprocessing
import os
import queue
import threading

import rclpy
from slam.point_cloud_publisher import PointCloudPublisher
from slam.point_cloud_subscriber import PointClouds2Subscriber
from slam.process_point_clouds import get_processor


class ProcessPointCloudsThread2:
    """
    Class to abstract multiprocessing for point cloud processing.
    """

    def __init__(self, algorithm: str, config_path: str, input_queue: queue.Queue,
                 stop_event: multiprocessing.Event, reset_event: multiprocessing.Event):

        self.input_queue = input_queue
        self.stop_event = stop_event
        self.reset_event = reset_event
        self.algorithm = algorithm
        self.config_path = config_path

        self.processor_thread = multiprocessing.Process(target=self.start_processing)

    def start_processing(self) -> None:
        """
        Start the point cloud processing thread. Should be run in a separate process.
        """
        rclpy.init()
        publisher_node = PointCloudPublisher()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher_node)
        processor = get_processor(
            algorithm=self.algorithm,
            publisher_node=publisher_node,
            config_path=self.config_path,
            queue=self.input_queue,
            stop_event=self.stop_event,
            reset_event=self.reset_event
        )
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        try:
            processor.run()
        except KeyboardInterrupt:
            print("Stopped by user")
        finally:
            # publisher_node.get_logger().info("destroying publisher node")
            publisher_node.destroy_node()
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()

    def start(self) -> None:
        """
        Start the point cloud processing thread.
        """
        self.processor_thread.start()

    def stop(self) -> None:
        """
        Stop the point cloud processing thread.
        """
        self.stop_event.set()
        self.processor_thread.join(timeout=3)

        if self.processor_thread.is_alive():
            self.processor_thread.terminate()  # Force terminate if it doesn't stop
            self.processor_thread.join()



def main():
    multiprocessing.set_start_method("spawn", force=True)  # Fix multiprocessing issues on MacOS
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

    # Disable unnecessary DDS buffering
    os.environ[
        "CYCLONEDDS_URI"] = ""
    # Force UDP transport (reduces overhead)
    os.environ["RMW_FASTRTPS_USE_UDP"] = "1"

    rclpy.init()

    # Create queues for data transfer
    input_queue = multiprocessing.Queue()
    reset_event = multiprocessing.Event()
    stop_event = multiprocessing.Event()

    # Create nodes
    subscriber_node = PointClouds2Subscriber(input_queue, reset_event)

    algorithm = subscriber_node.declare_parameter('algorithm', 'icp').value.lower()
    camera_config_name = subscriber_node.declare_parameter('config',
                                                'lidar_config.yaml').value.lower()

    # Start processor in a separate thread
    processor_thread = ProcessPointCloudsThread2(
        algorithm=algorithm,
        config_path=camera_config_name,
        input_queue=input_queue,
        stop_event=stop_event,
        reset_event=reset_event
    )
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(subscriber_node)
    try:
        processor_thread.start()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        processor_thread.stop()
        # executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

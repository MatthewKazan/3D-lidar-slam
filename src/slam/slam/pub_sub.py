import multiprocessing
import os
import queue
import threading

import rclpy
from slam.point_cloud_publisher import PointCloudPublisher
from slam.point_cloud_subscriber import PointClouds2Subscriber
from scripts.process_point_clouds import get_processor

from scripts.paths import PATH_TO_CONFIG


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
    # No idea if the following lines are necessary but
    # websockets has been finicky so im not touching it
    multiprocessing.set_start_method("spawn", force=True)  # Fix multiprocessing issues on MacOS
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = ""
    os.environ["RMW_FASTRTPS_USE_UDP"] = "1"
    # End weirdness

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
    config_path = os.path.join(
        PATH_TO_CONFIG,
        camera_config_name
    )
    # Start processor in a separate thread
    processor_thread = ProcessPointCloudsThread2(
        algorithm=algorithm,
        config_path=str(config_path),
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

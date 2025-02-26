import multiprocessing
import os
import queue
import threading

import rclpy
from slam.output_handler import PointCloudPublisher
from slam.input_handler import PointClouds2Subscriber

from scripts.paths import PATH_TO_CONFIG

from scripts.data_transfer import DataTransfer
from scripts.process_pc_manager import ProcessPointCloudsHandler
from slam.simple_service_handler import ResetServiceMapping, \
    ServiceMapping, SimpleServiceHandler




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
    data_transfer = DataTransfer()
    reset_event = multiprocessing.Event()
    stop_event = multiprocessing.Event()


    # Create nodes
    subscriber_node = PointClouds2Subscriber(data_transfer)
    publisher_node = PointCloudPublisher(data_transfer)

    service_mappings = [
        ResetServiceMapping([subscriber_node, publisher_node, data_transfer], reset_event),
        ServiceMapping("/save_global_map", publisher_node.save_map_callback),
        ServiceMapping("/toggle_save_inputs", subscriber_node.save_inputs_callback),
    ]
    service_handler = SimpleServiceHandler(service_mappings)


    algorithm = subscriber_node.declare_parameter('algorithm', 'icp').value.lower()
    camera_config_name = subscriber_node.declare_parameter('config',
                                                'lidar_config.yaml').value.lower()
    config_path = os.path.join(
        PATH_TO_CONFIG,
        camera_config_name
    )
    # Start processor in a separate thread
    processor_handler = ProcessPointCloudsHandler(
        algorithm=str(algorithm),
        config_path=str(config_path),
        data_transfer=data_transfer,
        stop_event=stop_event,
        reset_event=reset_event
    )
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(subscriber_node)
    executor.add_node(publisher_node)
    executor.add_node(service_handler)
    try:
        processor_handler.start()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        processor_handler.stop()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

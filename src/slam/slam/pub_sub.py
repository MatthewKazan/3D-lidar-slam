import multiprocessing
import os

import rclpy
from slam.output_data_handler import PointCloudPublisher
from slam.input_data_handler import PointClouds2Subscriber

from scripts.paths import PATH_TO_CONFIG

from scripts.data_transfer import DataTransfer
from scripts.process_pc_manager import ProcessPointCloudsHandlerNode
from slam.simple_service_handler import ResetServiceMapping, \
    ServiceMapping, SimpleServiceHandler
from std_srvs.srv import Trigger
from  custom_interfaces.srv import SetAlgorithm


def main():
    # No idea if the following lines are necessary but
    # websockets has been finicky so im not touching it
    multiprocessing.set_start_method("spawn", force=True)  # Fix multiprocessing issues on MacOS
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["CYCLONEDDS_URI"] = ""
    os.environ["RMW_FASTRTPS_USE_UDP"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"


    # End weirdness

    rclpy.init()

    # Create queues for data transfer
    data_transfer = DataTransfer()
    reset_event = multiprocessing.Event()
    stop_event = multiprocessing.Event()


    # Create nodes
    subscriber_node = PointClouds2Subscriber(data_transfer)
    publisher_node = PointCloudPublisher(data_transfer)

    algorithm = subscriber_node.declare_parameter('algorithm', 'icp').value.lower()
    camera_config_name = subscriber_node.declare_parameter('config',
                                                'config.yaml').value.lower()
    config_path = os.path.join(
        PATH_TO_CONFIG,
        camera_config_name
    )
    # Start processor in a separate thread
    processor_handler = ProcessPointCloudsHandlerNode(
        algorithm=str(algorithm),
        config_path=str(config_path),
        data_transfer=data_transfer,
        stop_event=stop_event,
        reset_event=reset_event
    )

    service_mappings = [
        ResetServiceMapping([subscriber_node, publisher_node, data_transfer], reset_event),
        ServiceMapping("/save_global_map", Trigger, publisher_node.save_map_callback),
        ServiceMapping("/toggle_save_inputs", Trigger, subscriber_node.save_inputs_callback),
        ServiceMapping("/set_algorithm", SetAlgorithm, processor_handler.set_algorithm),
    ]
    service_handler = SimpleServiceHandler(service_mappings)

    nodes = [subscriber_node, publisher_node, processor_handler, service_handler]

    executor = rclpy.executors.MultiThreadedExecutor()
    for node in nodes:
        node.get_logger().info(f"Adding {node.get_name()} to executor")
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for node in nodes:
                node.get_logger().info(f"Shutting down {node.get_name()}")
                node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

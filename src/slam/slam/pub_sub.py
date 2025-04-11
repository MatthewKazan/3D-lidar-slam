import multiprocessing
import os
import sys
import threading
import time
import traceback

import rclpy
import rclpy.logging
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from slam.output_data_handler import PointCloudPublisher
from slam.input_data_handler import PointClouds2Subscriber

from scripts.paths import PATH_TO_CONFIG

from scripts.data_transfer import DataTransfer
from scripts.process_pc_manager import ProcessPointCloudsHandlerNode
from slam.simple_service_handler import ResetHandler, \
    ServiceMapping, SimpleServiceHandler
from std_srvs.srv import Trigger
from custom_interfaces.srv import SetAlgorithm
from custom_interfaces.srv import GetAlgorithmsList

from slam.simple_service_handler import get_algorithms_list_callback

from slam.simple_service_handler import SetAlgorithmServiceMapping


def spin_executor(executor):
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()


def run_subscriber_process(data_transfer):
    """
    This function will run in a separate process.
    It initializes rclpy, creates the subscriber node, and spins an executor.
    """
    rclpy.init(args=None)  # Initialize ROS in this process
    # Create subscriber node using the shared data_transfer object.
    subscriber_node = PointClouds2Subscriber(data_transfer)
    reset_handler = ResetHandler(
        [subscriber_node],
        node_name='subscriber_reset_handler',
    )
    service_mappings = [
        ServiceMapping("/toggle_save_inputs", Trigger,
                       subscriber_node.save_inputs_callback),
    ]
    service_handler = SimpleServiceHandler(service_mappings, "subscriber_service_handler")
    nodes = [service_handler, reset_handler, subscriber_node]
    executor = MultiThreadedExecutor(8)
    for node in nodes:
        node.get_logger().info(f"Adding {node.get_name()} to executor")
        executor.add_node(node)
    try:
        executor.spin()  # Spin until shutdown
    except KeyboardInterrupt:
        pass
    finally:
        data_transfer.queue_shutdown()
        data_transfer.stop_event.set()
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
            rclpy.logging.get_logger("subscriber_process").info(
                f"Ended node {type(node)}")
        if rclpy.ok():
            rclpy.logging.get_logger("subscriber_process").info(
                "Shut down subscriber process...")
            rclpy.shutdown()
        sys.exit(0)

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

    # Create nodes
    # subscriber_node = PointClouds2Subscriber(data_transfer)
    publisher_node = PointCloudPublisher(data_transfer)

    algorithm = publisher_node.declare_parameter('algorithm', 'icp').value.lower()
    camera_config_name = publisher_node.declare_parameter('config',
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
        # stop_event=data_transfer.stop_event,
        # reset_event=data_transfer.reset_event
    )
    reset_handler = ResetHandler([publisher_node, data_transfer, processor_handler])
    service_mappings = [
        ServiceMapping("/save_global_map", Trigger,
                       publisher_node.save_map_callback),
        ServiceMapping("/set_algorithm", SetAlgorithm,
                       processor_handler.set_algorithm),
        ServiceMapping("/get_algorithms_list", GetAlgorithmsList,
                       get_algorithms_list_callback),
        SetAlgorithmServiceMapping(callback=processor_handler.set_algorithm)
    ]
    service_handler = SimpleServiceHandler(service_mappings)

    nodes = [service_handler, reset_handler, processor_handler, publisher_node]

    executor = rclpy.executors.MultiThreadedExecutor(len(nodes) + 3)
    for node in nodes:
        node.get_logger().info(f"Adding {node.get_name()} to executor")
        executor.add_node(node)

    # executor_sub_pub = MultiThreadedExecutor(8)
    # executor_sub_pub.add_node(subscriber_node)
    # executor_sub_pub.add_node(service_handler)
    #
    # # And another executor for processor and service nodes (heavy processing)
    # executor_proc_srv = MultiThreadedExecutor(8)
    # executor_proc_srv.add_node(processor_handler)
    # executor_proc_srv.add_node(publisher_node)

    # Spin each executor in its own thread.
    # thread_sub_pub = threading.Thread(target=spin_executor, args=(executor_sub_pub,), daemon=True)
    # thread_proc_srv = threading.Thread(target=spin_executor, args=(executor_proc_srv,), daemon=True)
    #
    # thread_sub_pub.start()
    # thread_proc_srv.start()
    subscriber_proc = multiprocessing.Process(
        target=run_subscriber_process, args=(data_transfer,), daemon=True
    )
    subscriber_proc.start()


    try:
        executor.spin()
        # while rclpy.ok():
        #     time.sleep(1)
    except KeyboardInterrupt:
        publisher_node.get_logger().info("Keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"Executor crashed: {e}")
        traceback.print_exc()
    finally:
        data_transfer.stop_event.set()
        data_transfer.queue_shutdown()
        subscriber_proc.join(5)

        executor.shutdown()
        # executor_sub_pub.shutdown()
        # executor_proc_srv.shutdown()

        for node in nodes:
            node.destroy_node()
            rclpy.logging.get_logger("processing_manager").info(
                f"Ended node {type(node)}")

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

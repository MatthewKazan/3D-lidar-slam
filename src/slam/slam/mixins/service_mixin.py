from typing import List, Callable, Type, Generic, TypeVar, Any

import rclpy
import std_msgs.msg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Empty
from custom_interfaces.srv import GetAlgorithmsList
from custom_interfaces.srv import SetAlgorithm

from scripts.algorithm_enum import AlgorithmType

ServiceT = TypeVar("ServiceT")

class ServiceMapping(Generic[ServiceT]):
    """
    Map a service name to a specific instance of its callback functions.
    """
    def __init__(self, service_name: str, service_type: Type[ServiceT], callback: Callable[[Any, Any], Any]):
        self.service_name = service_name
        self.callback_fn = callback
        self.service_type = service_type

    def callback(self, request, response) -> Any:
        return self.callback_fn(request, response)


class SimpleServiceMixin:
    """
    Class to receive and delegate simple service requests
    """

    def __init_simple_service_mixin__(self, services: List[ServiceMapping], node_name: str = 'service_handler'):
        """
        Initialize the SimpleServiceHandler.
        :param services: A list of mappings for service names to a specific instance of its callback functions.
        """
        self.service_callback_map = {}
        for service_callback in services:
            self.create_service(service_callback.service_type, service_callback.service_name, service_callback.callback)


class SetAlgorithmServiceMapping(ServiceMapping):
    """
    Map a set_algorithm to a specific instance of its callback functions.
    """
    def __init__(self, callback: Callable[[Any, Any], Any]):
        super().__init__("/set_algorithm", SetAlgorithm, self.callback)
        self.callback_fn = callback


    def callback(self, request, response) -> SetAlgorithm.Response:
        """
        Callback function to handle requests to set the algorithm.

        :param request: The request object.
        :param response: The response object.

        :return: The response object with success status and message.
        """
        node = Node('one_off_publisher')
        publisher = node.create_publisher(Empty, '/reset', 10)
        msg = Empty()
        publisher.publish(msg)
        node.get_logger().info("Published reset.")
        # Give it a short time to process the publish
        rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
        response = self.callback_fn(request, response)
        return response


def get_algorithms_list_callback(request, response) -> GetAlgorithmsList.Response:
    """
    Callback function to handle requests for the list of algorithms.

    :param request: The request object.
    :param response: The response object.

    :return: The response object with the list of algorithms.
    """
    response.algorithms = [e.value for e in AlgorithmType]
    return response
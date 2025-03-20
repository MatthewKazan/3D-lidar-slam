import multiprocessing
from typing import List, Callable, Type, Generic, TypeVar, Any

from rclpy.node import Node
from std_srvs.srv import Trigger
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

class ResetServiceMapping(ServiceMapping):
    """
    Map a reset to a specific instance of its callback functions.
    """
    def __init__(self, objs_to_reset, event: multiprocessing.Event):
        super().__init__("/reset", Trigger, self.callback)
        self.objs_to_reset = objs_to_reset
        self.event = event

    def reset(self):
        """
        Reset the objects associated with this service mapping and set the event
        to signal that a reset has occurred to other processes or threads.
        """
        self.event.set()
        for obj in self.objs_to_reset:
            obj.reset()


    def callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Callback function to handle reset requests.

        :param request: The request object.
        :param response: The response object.

        :return: The response object with success status and message.
        """
        self.reset()

        response.success = True
        response.message = "Reset successful"
        return response


class SimpleServiceHandler(Node):
    """
    Class to receive and delegate simple service requests
    """

    def __init__(self, services: List[ServiceMapping]):
        """
        Initialize the SimpleServiceHandler.
        :param services: A list of mappings for service names to a specific instance of its callback functions.
        """
        super().__init__('simple_service_handler')
        self.service_callback_map = {}
        for service_callback in services:
            self.create_service(service_callback.service_type, service_callback.service_name, service_callback.callback)


class SetAlgorithmServiceMapping(ServiceMapping):
    """
    Map a set_algorithm to a specific instance of its callback functions.
    """
    def __init__(self, reset_service_mapping: ResetServiceMapping, callback: Callable[[Any, Any], Any]):
        super().__init__("/set_algorithm", SetAlgorithm, self.callback)
        self.reset_service_mapping = reset_service_mapping
        self.callback_fn = callback

    def callback(self, request, response) -> SetAlgorithm.Response:
        """
        Callback function to handle requests to set the algorithm.

        :param request: The request object.
        :param response: The response object.

        :return: The response object with success status and message.
        """
        self.reset_service_mapping.reset()
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
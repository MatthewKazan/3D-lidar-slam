import multiprocessing
import time
from typing import List, Callable

from rclpy.node import Node
from std_srvs.srv import Trigger

class ServiceMapping:
    """
    Map a service name to a specific instance of its callback functions.
    """
    def __init__(self, service_name: str, callback: Callable[[Trigger.Request, Trigger.Response], Trigger.Response]):
        self.service_name = service_name
        self.callback_fn = callback

    def callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self.callback_fn(request, response)

class ResetServiceMapping(ServiceMapping):
    """
    Map a reset to a specific instance of its callback functions.
    """
    def __init__(self, objs_to_reset, event: multiprocessing.Event):
        super().__init__("/reset", self.callback)
        self.objs_to_reset = objs_to_reset
        self.event = event

    def callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Callback function to handle reset requests.
        :param request: The request object.
        :param response: The response object.
        """
        self.event.set()
        for obj in self.objs_to_reset:
            obj.reset()

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
            self.create_service(Trigger, service_callback.service_name, service_callback.callback)


    def service_callback(self, request, response, service_name):
        """
        Callback function to handle service requests.
        :param request: The request object.
        :param response: The response object.
        """

        if service_name in self.service_callback_map:
            callback = self.service_callback_map[service_name]
            response = callback(request, response)
        else:
            self.get_logger().error(f"Service {service_name} not found.")
            response.success = False
            response.message = f"Service {service_name} not found."

        return response
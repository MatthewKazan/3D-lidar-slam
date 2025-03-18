import multiprocessing
from typing import List, Callable, Type, Generic, TypeVar, Any

from rclpy.node import Node
from std_srvs.srv import Trigger

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

    def callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        Callback function to handle reset requests.

        :param request: The request object.
        :param response: The response object.

        :return: The response object with success status and message.
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
            self.create_service(service_callback.service_type, service_callback.service_name, service_callback.callback)

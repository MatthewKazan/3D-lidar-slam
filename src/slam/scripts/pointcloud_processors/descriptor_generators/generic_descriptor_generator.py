from abc import ABC, abstractmethod

class GenericDescriptorGenerator(ABC):
    """
    A generic descriptor generator that can be used to generate descriptors for point clouds.
    This class is not meant to be used directly, but rather as a base class for other descriptor generators.
    """

    def __init__(self, logger):
        """
        Initialize the descriptor generator with a logger.

        Args:
            logger: A logger instance for logging messages.
        """
        self.logger = logger

    @abstractmethod
    def generate_descriptor(self, point_cloud):
        """
        Generate descriptor for the given point cloud.

        Args:
            point_cloud: The input point cloud.

        Returns:
            A list of descriptors for the point cloud.
        """
        raise NotImplementedError("This method should be overridden by subclasses.")
import numpy as np

from scripts.pointcloud_processors.descriptor_generators.generic_descriptor_generator import \
    GenericDescriptorGenerator


class ScanContext(GenericDescriptorGenerator):
    def __init__(self):
        """
        Initialize the ScanContext descriptor generator.
        """
        super().__init__(logger=None)

    def generate_descriptor(self, point_cloud):
        """
        Generate descriptors for the given point cloud.

        :param point_cloud: Input point cloud of shape (N, 3) with columns [x, y, z].

        :return: List of descriptors for the point cloud.
        """
        # Generate a single descriptor for the point cloud.
        descriptor = self.compute_scan_context_descriptor(point_cloud)
        return descriptor

    def compute_scan_context_descriptor(self, scan, num_angle_bins=60, num_radius_bins=20,
        max_range=80.0):
        """
        Computes a Scan Context descriptor for a point cloud.

        :param scan: Input point cloud of shape (N, 3) with columns [x, y, z].
        :param num_angle_bins: Number of bins for the azimuth angle.
        :param num_radius_bins: Number of bins for the radial distance.
        :param max_range: Maximum range to consider for binning.

        :return: Flattened descriptor vector.
        """
        # Initialize the descriptor matrix with a very small value.
        descriptor = np.full((num_radius_bins, num_angle_bins), -np.inf)

        # Compute polar coordinates (rho, theta) for each point.
        xs = scan[:, 0]
        ys = scan[:, 1]
        zs = scan[:, 2]
        rho = np.sqrt(xs ** 2 + ys ** 2)
        theta = np.arctan2(ys, xs)  # range [-pi, pi]

        # Only consider points within the max_range.
        valid = rho < max_range
        rho = rho[valid]
        theta = theta[valid]
        zs = zs[valid]

        # Map angles from [-pi, pi] to [0, 2*pi]
        theta = theta + np.pi

        # Determine bin indices.
        angle_bin_indices = np.floor(theta / (2 * np.pi) * num_angle_bins).astype(
            np.int32)
        radius_bin_indices = np.floor(rho / max_range * num_radius_bins).astype(
            np.int32)

        # Clamp indices to valid range.
        angle_bin_indices = np.clip(angle_bin_indices, 0, num_angle_bins - 1)
        radius_bin_indices = np.clip(radius_bin_indices, 0, num_radius_bins - 1)

        # Populate the descriptor matrix: use maximum z in each bin.
        for r_bin, a_bin, z in zip(radius_bin_indices, angle_bin_indices, zs):
            # Update the bin if this z is higher than the current stored value.
            if z > descriptor[r_bin, a_bin]:
                descriptor[r_bin, a_bin] = z

        # Replace -inf values with 0 (bins that received no points).
        descriptor[descriptor == -np.inf] = 0

        # Optionally, flatten and L2-normalize the descriptor.
        flat_descriptor = descriptor.flatten()
        norm = np.linalg.norm(flat_descriptor) + 1e-8
        flat_descriptor /= norm

        return flat_descriptor

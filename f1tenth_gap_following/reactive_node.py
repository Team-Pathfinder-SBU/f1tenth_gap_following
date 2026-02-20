import rclpy
from rclpy.node import Node

import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class ReactiveFollowGap(Node):
    """
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """

    def __init__(self):
        super().__init__("reactive_node")
        # Topics & Subs, Pubs
        lidarscan_topic = "/scan"
        drive_topic = "/drive"

        self.angle_window = 5  # number of samples for mean filtering
        # self.max_range = 3.0    # maximum range to consider (meters)
        self.radius = (
            0.25  # radius of the bubble around closest point to clear (meters)
        )
        self.safety_distance = 0.1  # safety distance threshold

        # Create ROS subscriber and publisher
        self.scan_subscriber = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, "/drive", 10
        )

    def preprocess_lidar(self, ranges):
        """Preprocess the LiDAR scan array. Expert implementation includes:
        1.Setting each value to the mean over some window (Window size is set to 5)
        2.Rejecting high values (eg. > 3m)
        """

        # Convert to numpy array if it's not already
        ranges = np.array(ranges)

        # Perform slicing to get the lidar data from right 90˚ angle (180) to the left 90˚ angle (900)
        proc_ranges = ranges[180:901]

        # Replace infinite, NaN, and non-positive values with max_range
        max_range = 3.0
        proc_ranges = np.where(
            (~np.isfinite(proc_ranges)) | (proc_ranges <= 0.0), max_range, proc_ranges
        )

        # Apply mean filtering over a window
        kernel_size = self.angle_window
        if kernel_size > 0:
            # Use convolution for mean filtering
            kernel = np.ones(kernel_size) / kernel_size
            proc_ranges = np.convolve(proc_ranges, kernel, mode="same")

        # Reject high values (set to max_range of 3.0 meters)
        proc_ranges = np.clip(proc_ranges, 0, 3.0)

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """Return the start index & end index of the max gap in free_space_ranges"""
        # Convert to list if it's a numpy array
        if isinstance(free_space_ranges, np.ndarray):
            free_space_ranges = free_space_ranges.tolist()

        # Append a zero at the end of free_space_ranges to ensure the last gap is counted
        # The loop below assume a gap always ends with a zero
        free_space_ranges.append(0)

        # Largest found gap size and its starting index
        max_size = 0
        max_start = 0

        # Current gap size and its starting index
        curr_size = 0
        curr_start = 0

        # Loop through the free_space_ranges
        for i in range(len(free_space_ranges)):
            # Check if the current distance is greater than 0
            if free_space_ranges[i] > 0.0:
                # If it is, we are in a gap. If this is the start of a new gap, record the starting index.
                if curr_size == 0:
                    curr_start = i

                # Increment the current gap size as we are still in a gap
                curr_size += 1
            else:
                # If not, we are either encounter an obstsacle or have reached the end of a gap.
                # Check if the current gap size is larger than the maximum found so far.
                if curr_size > max_size:
                    # If it is, update the maximum gap size and starting index.
                    max_size = curr_size
                    max_start = curr_start

                # Reset the current gap size to 0 as we are no longer in a gap
                curr_size = 0

        # Uncomment if the list is not appended with a zero at the end before the loop
        # # Final check (in case the list ends with positives)
        # if curr_size > max_size:
        #     max_size = curr_size
        #     max_start = curr_start

        # Return the starting and ending index of the maximum gap found
        return max_start, max_start + max_size - 1

    def draw_bubble(self, ranges, min_index):
        left_exceed, right_exceed = False, False
        n = 1

        # Check neighbor points on left of the closest point until points are outside the radius
        while True:
            # Obtain the distance from the n-th nearest point and the point next to it
            D_t = ranges[min_index]

            # Check bounds before accessing neighbors
            if min_index + n >= len(ranges):
                left_exceed = True
            else:
                D_n_left = ranges[min_index + n]

                # Angle between the two beams (radians)
                theta = math.radians(0.25 * n)

                # Draw a perpendicular line from the n-th nearest point to the neighbor beam
                # Use trigonometry to calculate the distance from the n-th neighbor point to the nearest point
                s1 = math.sin(theta) * D_t

                s2_left = D_n_left - math.cos(theta) * D_t
                D_t2n_left = math.sqrt(s1**2 + s2_left**2)

                # Check if the distance from then-th neighbor point to the nearest point is within the radius of the bubble
                if D_t2n_left < self.radius:
                    # If it is, set the value of the neighbor point to 0 (eliminate it)
                    ranges[min_index + n] = 0.0
                else:
                    left_exceed = True

            if min_index - n < 0:
                right_exceed = True
            else:
                D_n_right = ranges[min_index - n]

                # Angle between the two beams (radians)
                theta = math.radians(0.25 * n)

                # Draw a perpendicular line from the n-th nearest point to the neighbor beam
                # Use trigonometry to calculate the distance from the n-th neighbor point to the nearest point
                s1 = math.sin(theta) * D_t

                s2_right = D_n_right - math.cos(theta) * D_t
                D_t2n_right = math.sqrt(s1**2 + s2_right**2)

                if D_t2n_right < self.radius:
                    # If it is, set the value of the neighbor point to 0 (eliminate it)
                    ranges[min_index - n] = 0.0
                else:
                    right_exceed = True

            n += 1

            # The ranges data are convoluted and smoothed,
            # so don't have to worry if the points next to the closest point exceed the radius
            # but the points further away are within the radius.
            if left_exceed and right_exceed:
                break

        return ranges

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
            Naive: Choose the furthest point within ranges and go there
        """
        # Find the furthest point in the gap

        # Extract the gap segment from the ranges and find the index of the maximum value in that segment
        max_range_idx = np.argmax(ranges[start_i : end_i + 1])

        # Add the starting offset to get the best point index in the sliced ranges array
        best_idx = start_i + max_range_idx

        return best_idx

    def scan_callback(self, msg):
        """Process the LaserScan message"""
        ranges = msg.ranges

        proc_ranges = self.preprocess_lidar(ranges)

        # TODO:
        # Find closest point to LiDAR
        min_distance = np.min(proc_ranges)
        min_index = np.argmin(proc_ranges)

        # Emergency brake if too close
        if min_distance < self.safety_distance:
            # Emergency stop or turn if too close
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            self.drive_publisher.publish(drive_msg)
            return

        # Eliminate all points inside 'bubble' (set them to zero)
        # free_space_ranges = proc_ranges.copy()
        free_space_ranges = self.draw_bubble(proc_ranges, min_index)

        # Find max length gap
        start_i, end_i = self.find_max_gap(free_space_ranges)

        # Validate gap indices
        if end_i < start_i or start_i < 0 or end_i >= len(free_space_ranges):
            # No valid gap found, stop the car
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            self.drive_publisher.publish(drive_msg)
            return

        # Find the best point in the gap
        # add offset to get index in original ranges array
        best_idx = self.find_best_point(start_i, end_i, free_space_ranges) + 180
        best_angle = best_idx * msg.angle_increment - msg.angle_min

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = best_angle

        # Adjust speed based on steering angle
        steering_deg = abs(best_angle)

        if steering_deg >= 20.0:
            velocity = 0.5
        elif steering_deg >= 10.0:
            velocity = 1.0
        else:
            velocity = 1.5

        drive_msg.drive.speed = velocity

        # Log the steering angle and speed for debugging every 1 second
        self.get_logger().info(
            f"Steering: {steering_deg:.1f}°, Speed: {velocity:.3f} m/s",
            throttle_duration_sec=1.0,
        )

        # One-time print of the raw ranges with indices
        # if not getattr(self, "_printed_ranges", False):
        #     self._printed_ranges = True
        #     r_full = np.array(ranges)
        #     lines = [f"{i}: {v:.3f}" for i, v in enumerate(r_full)]
        #     self.get_logger().info("First scan ranges:\n" + "\n".join(lines), once=True)

        self.drive_publisher.publish(drive_msg)


def main(args=None):
    # ScanPlotter().run()

    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import numpy as np
import matplotlib.pyplot as plt

class ProxSensor:
    """
    Simulates a proximity sensor that estimates wheel velocity with some noise.
    """
    def __init__(self, offset_x, noise_std=0.03):  # Reduced noise
        self.offset_x = offset_x  # Position relative to the wheel center
        self.reading = 0  # Initial sensor reading
        self.noise_std = noise_std  # Standard deviation of noise

    def scan(self, wheel_velocity):
        """
        Measure wheel velocity with added Gaussian noise.
        """
        self.reading = wheel_velocity + np.random.normal(0, self.noise_std)


class TrailerModel:
    """
    Models a trailer attached to a moving vehicle, updating its position based on the main vehicle's movement.
    """
    def __init__(self, trailer_offset=1.5):
        self.trailer_offset = trailer_offset  # Distance between vehicle and trailer
        self.x_trailer = 0
        self.y_trailer = -trailer_offset
        self.theta_trailer = 0  # Initial orientation

    def update_trailer_pose(self, x, y, theta):
        """
        Update the trailer's pose based on the robot's position and orientation.
        """
        alpha = np.arctan2(y - self.y_trailer, x - self.x_trailer)
        self.theta_trailer = alpha  # Adjust the orientation
        self.x_trailer = x - self.trailer_offset * np.cos(self.theta_trailer)
        self.y_trailer = y - self.trailer_offset * np.sin(self.theta_trailer)
        return self.x_trailer, self.y_trailer, self.theta_trailer


class Robot:
    """
    Simulates a robot with two proximity sensors and a trailer.
    It moves forward, estimates distance traveled, and updates its pose.
    """
    def __init__(self, x=0, y=0, theta=0, step_size=1, sensor_offset=0.5, wheel_base=1.0):
        self.x = x  # Robot's X position
        self.y = y  # Robot's Y position
        self.theta = theta  # Orientation in radians
        self.step_size = step_size  # Movement per step
        self.wheel_base = wheel_base  # Distance between wheels
        self.left_sensor = ProxSensor(-sensor_offset)
        self.right_sensor = ProxSensor(sensor_offset)
        self.trailer = TrailerModel()
        self.total_distance = 0  # Estimated total distance traveled
        self.true_distance = 0  # Actual distance traveled (ground truth)

    def sense(self):
        """
        Simulate sensor readings based on wheel velocity.
        """
        wheel_velocity = self.step_size
        self.left_sensor.scan(wheel_velocity)
        self.right_sensor.scan(wheel_velocity)

    def estimate_distance(self):
        """
        Estimate distance traveled based on sensor readings.
        """
        left_dist = self.left_sensor.reading
        right_dist = self.right_sensor.reading
        avg_dist = (left_dist + right_dist) / 2
        self.total_distance += avg_dist
        return avg_dist

    def estimate_pose(self):
        """
        Estimate the robot's new position and orientation using a simple motion model.
        """
        left_dist = self.left_sensor.reading
        right_dist = self.right_sensor.reading
        delta_d = (left_dist + right_dist) / 2  # Average distance traveled
        delta_theta = np.arctan((right_dist - left_dist) / self.wheel_base)  # Orientation change

        self.theta += delta_theta
        self.x += delta_d * np.cos(self.theta)
        self.y += delta_d * np.sin(self.theta)

        # Update trailer position
        self.trailer.update_trailer_pose(self.x, self.y, self.theta)

    def move(self):
        """
        Execute one complete motion cycle (sense → estimate distance → estimate pose).
        """
        self.sense()
        self.estimate_distance()
        self.estimate_pose()
        self.true_distance += self.step_size  # Ground truth distance
        print(f"Step: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}, "
              f"Estimated Distance={self.total_distance:.2f}, True Distance={self.true_distance:.2f}")


# Initialize robot and storage for results
robot = Robot()
trajectory = [(robot.x, robot.y)]
trailer_trajectory = [(robot.trailer.x_trailer, robot.trailer.y_trailer, robot.trailer.theta_trailer)]
distance_traveled = []
true_distances = []

# Run simulation for 100 steps
for _ in range(100):
    robot.move()
    trajectory.append((robot.x, robot.y))
    trailer_trajectory.append((robot.trailer.x_trailer, robot.trailer.y_trailer, robot.trailer.theta_trailer))
    distance_traveled.append(robot.total_distance)
    true_distances.append(robot.true_distance)

# Compute error metrics
errors = np.array(distance_traveled) - np.array(true_distances)
mae = np.mean(np.abs(errors))
rmse = np.sqrt(np.mean(errors**2))
print(f"Mean Absolute Error (MAE): {mae:.4f}")
print(f"Root Mean Square Error (RMSE): {rmse:.4f}")

# Plot robot and trailer trajectories
trajectory = np.array(trajectory)
trailer_trajectory = np.array(trailer_trajectory)
plt.figure(figsize=(10, 5))

# Subplot 1: Path visualization
plt.subplot(1, 2, 1)
plt.plot(trajectory[:, 0], trajectory[:, 1], marker='o', label='Robot Path')
plt.plot(trailer_trajectory[:, 0], trailer_trajectory[:, 1], marker='s', linestyle='dashed', label='Trailer Path')
plt.quiver(trailer_trajectory[:-1, 0], trailer_trajectory[:-1, 1], 
           np.cos(trailer_trajectory[:-1, 2] - 0.1), np.sin(trailer_trajectory[:-1, 2] - 0.1), 
           scale=5, color='r', label='Trailer Orientation')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot and Trailer Path')
plt.legend()
plt.grid()

# Subplot 2: Distance estimation error
plt.subplot(1, 2, 2)
plt.plot(range(len(distance_traveled)), distance_traveled, label='Estimated Distance')
plt.plot(range(len(true_distances)), true_distances, label='True Distance', linestyle='dashed')
plt.xlabel('Time Step')
plt.ylabel('Distance Traveled')
plt.title('Distance Estimation Error')
plt.legend()
plt.grid()

plt.show()

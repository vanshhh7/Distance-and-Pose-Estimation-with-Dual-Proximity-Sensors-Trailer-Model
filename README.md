# Distance-and-Pose-Estimation-with-Dual-Proximity-Sensors-Trailer-Model


# Introduction

This project simulates a mobile robot equipped with two proximity sensors and a trailer to estimate its distance traveled and pose. The system models sensor noise, updates trailer position dynamically, and evaluates the accuracy of distance estimation using error metrics such as Mean Absolute Error (MAE) and Root Mean Square Error (RMSE). The simulation runs for a fixed number of steps, visualizing the robot's path, trailer movement, and estimation errors.

# Features

**Proximity Sensor Simulation:** Two virtual proximity sensors measure wheel velocity with noise.

**Trailer Model:** A dynamically updated trailer follows the robot's movement.

**Pose Estimation:** The robot estimates its position and orientation using a simple motion model.

**Distance Estimation:** The system compares estimated and actual distances traveled.

**Error Metrics:** Computes MAE and RMSE for accuracy evaluation.

**Plot Visualizations:** Displays robot and trailer paths, along with distance estimation errors.

# Dependencies

Ensure you have the following Python libraries installed before running the simulation:

    pip install numpy matplotlib

# How It Works

**The project consists of three primary components:**

# 1. Proximity Sensor Simulation

Each wheel is equipped with a proximity sensor that measures its velocity. The measurement includes a small Gaussian noise to simulate real-world sensor inaccuracies.

class ProxSensor:

    def __init__(self, offset_x, noise_std=0.03):
        self.offset_x = offset_x  # Sensor position relative to the wheel center
        self.reading = 0
        self.noise_std = noise_std  # Standard deviation of noise
        
    def scan(self, wheel_velocity):
        self.reading = wheel_velocity + np.random.normal(0, self.noise_std)

# 2. Trailer Model
The trailer is positioned at a fixed offset from the robot and updates its orientation based on the robot's movement.

 
    class TrailerModel:
      def __init__(self, trailer_offset=1.5):
        self.trailer_offset = trailer_offset
        self.x_trailer = 0
        self.y_trailer = -trailer_offset
        self.theta_trailer = 0


# 3. Robot Motion and Pose Estimation

The robot moves forward, estimates its new position, and updates the trailer's pose. The distance traveled is estimated from sensor readings.


    class Robot:
      def __init__(self, x=0, y=0, theta=0, step_size=1, sensor_offset=0.5, wheel_base=1.0):
          self.x = x
          self.y = y
          self.theta = theta
          self.step_size = step_size
          self.wheel_base = wheel_base
          self.left_sensor = ProxSensor(-sensor_offset)
          self.right_sensor = ProxSensor(sensor_offset)
          self.trailer = TrailerModel()

# Running the Simulation

To execute the simulation, run the Python script:

    python simulation.py

# The simulation will:

Move the robot forward for 100 steps.

Estimate its distance and pose.

Update the trailer position.

Compute estimation errors.

Generate trajectory and error plots.

# Plots and Visualization

**The script produces two plots:**

**Robot and Trailer Path**
**Shows the movement of the robot and trailer.**
**Red arrows indicate trailer orientation.**

![Screenshot 2025-02-27 224000](https://github.com/user-attachments/assets/7ad30051-d346-47d2-a291-081ce7599027)![Screenshot 2025-02-27 225225](https://github.com/user-attachments/assets/844abb1b-5c9b-4f0d-b590-37e672e3e260) ![Screenshot 2025-02-27 2304100](https://github.com/user-attachments/assets/75b63748-9c8b-490b-aa1d-1543dc7ba425)




**Distance Estimation Error**
**Compares estimated vs. true distance traveled.**
**Shows estimation errors over time.**
![Screenshot 2025-02-27 230400](https://github.com/user-attachments/assets/cf89b1c8-7c27-4cfc-824f-a7b8c3e1a9fb)

# Conclusion:

This simulation provides insights into distance estimation using proximity sensors and highlights the impact of sensor noise on accuracy.

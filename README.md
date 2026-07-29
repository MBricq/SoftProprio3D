# Real-Time 3D Proprioception for Soft Robots Using a Single Capacitive Bend Sensor

Official codebase and live visualization demo for the paper presented at BioRob 2026.

## Overview

This repository provides the real-time 3D reconstruction pipeline for an under-actuated, cable-driven soft robot. By embedding a single commercial two-axis capacitive bend sensor into the actuator's core, the system achieves intrinsic proprioception without relying on external optical trackers or computationally heavy neural networks. The pose is mapped analytically using a Constant Curvature model, enabling true real-time execution.  

## Demo

By running this code, you can visualize the live 3D reconstruction of the soft robot's pose based on the sensor readings in a window such as the one shown below. The visualization includes the soft robot's segments, the base, and the tip, along with real-time updates of the sensor data.

<video width="100%" controls>
  <source src="resources/live_demo.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

A video of the complete testbench is available on: [SoftProprio3D Live Demo](https://mbricq.github.io/projects/biorob-2026/).

## Hardware Architecture

* **Soft Actuator**: A spiral-inspired geometry printed in 95A TPU, consisting of a fixed base and five movable segments.
* **Proprioceptive Sensor**: Nitto Two-Axis capacitive bend sensor with a measurement range of $\pm90^\circ$ and $0.18^\circ$ repeatability, mounted coaxially with the actuator's endpoints.
* **Acquisition**: Sensor data is read via the BLE kit provided by Nitto, which transmits the readings to a PC for real-time processing and visualization.

## Quick Start

1. Clone the repository:

   ```bash
   git clone https://github.com/MBricq/SoftProprio3D.git
   cd SoftProprio3D
   ```

2. Install the required dependencies:

   ```bash
    pip install -r requirements.txt
    ```

3. Run the live visualization script:

   ```bash
   python live_plot.py
   ```

## Key Results & Performance

The sensor-based reconstruction was validated against a multi-camera optical ground truth system, demonstrating significant improvements over open-loop, motor-encoder estimations.

* **Sub-Millimeter Precision**: Achieves a global pose RMSE of 1.20mm and a mean tip error of 2.48mm across the full 3D workspace.
* **Angular Accuracy**: Exhibits a mean directional error of 0.35°, making it highly viable for applications requiring precise joint angle measurements like exoskeletons.
* **Robustness to Nonlinearities**: Maintains high tracking fidelity (R² ≥ 0.94) even when mechanical disturbances, such as severe cable slacking, are deliberately introduced.
* **Ultra-Low Latency**: The analytical reconstruction algorithm executes in 0.0241 ± 0.0199 ms on a Raspberry Pi 4 Model B, and under 2 ms on an embedded Arduino Nano 33 IoT.
* **Temporal Stability**: Highly resistant to sensor drift, requiring approximately 87 minutes of continuous operation to accumulate 1mm of global pose error.

## Citation

Once the paper is published, citation details will be provided here. Please check back later for updates.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

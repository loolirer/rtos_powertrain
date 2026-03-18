# Real-Time Powertrain for Raspberry Pi Pico 2W

## Overview
This project implements a real-time motor control system for the Raspberry Pi Pico 2W. It uses FreeRTOS to manage tasks and hardware timers for precise motor control. The system is designed to control multiple motors, measure their speed using encoders, and adjust their performance based on PID control algorithms. Additionally, the project includes Python scripts for remote control and telemetry.

## Motivation
The goal of this project is to create a robust and efficient motor control system for robotics and automation applications. By leveraging the Raspberry Pi Pico's hardware capabilities and FreeRTOS, the system achieves real-time performance while maintaining modularity and scalability.

## Features
- Real-time motor control using PID algorithms.
- Hardware timer-based control loop for precise timing.
- FreeRTOS for task management for network robustness.
- Python scripts for remote control and telemetry.
- Modular design for easy customization and scalability.

## Build Pipeline in VS Code
To build and deploy the project, follow these steps:

1. **Install the Pico SDK and VS Code Extension**:
   - Install the Raspberry Pi Pico on your VS Code.

2. **Configure the Project**:
   - Open the project folder in VS Code.
   - Press `Ctrl + Shift + P` and select `CMake: Configure` (make sure the CMake extension is installed).

3. **Build the Project**:
   - Use the Pico extension to compile the project.
   - Flash the compiled firmware to the Raspberry Pi Pico.

## Running the Python Scripts
The Python scripts in the `python` folder are used for remote control and telemetry. Follow these steps to run them:

1. **Install Python Requirements**:
   - Install the required Python packages:
     ```bash
     pip install -r python/requirements.txt
     ```

2. **Run the Scripts**:
   - To run the controller script:
     ```bash
     python python/controller.py
     ```
   - To run the python/listener script:
     ```bash
     python listener.py
     ```

    Run them on separate terminals to use them together.

## Folder Structure
```
rtos_powertrain/
├── src/                # C source files
├── include/            # Header files
├── python/             # Python scripts for remote control and telemetry
├── README.md           # Project documentation
```

## Explanatory Video 

Watch the video explaining the project [here](https://drive.google.com/drive/folders/1yYwT7XDRWtrHN2iQOrlPad8jU_huYdKD?usp=sharing) This link also hosts the presentation file as additional documentation.

---
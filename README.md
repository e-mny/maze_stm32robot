# Image Capturing Robot

## Overview 📙

This project involves the development of a robot using C programming and the STM32F407VETX microcontroller board. 

The robot is designed to navigate and detect the printed number on a particular side of each block.

 <! -- Insert MDP Video / GIF here -->

## Features 💯

- Trained on YOLOV7 Object Detection model
- Obstacle detection and avoidance
- Efficient pathfinding algorithm

## Hardware Components 🤖

- **STM32F407VETX Board**: The microcontroller board used for processing and control.
- **Ultrasonic Sensors**: Used for obstacle detection.
- **Motors and Motor Drivers**: For robot movement.
- **Raspberry Pi**: Middleware to communicate to all peripherals.

## Software Components 💻

- **Programming Language**: C
- **Development Environment**: STM32CubeIDE or any compatible IDE
- **Libraries**: HAL (Hardware Abstraction Layer) libraries for STM32

## Installation and Setup 🛠️

### Prerequisites 

- STM32CubeIDE or a compatible IDE installed on your computer.
- STM32CubeMX for initializing the STM32F407VETX board.
- Basic knowledge of C programming and embedded systems.

### Steps 📝

1. **Clone the Repository**:
    ```sh
    git clone https://github.com/e-mny/maze_stm32robot.git
    ```

2. **Open the Project**:
    - Open STM32CubeIDE.
    - Import the project by selecting `File` -> `Import` -> `General` -> `Existing Projects into Workspace`.
    - Browse to the cloned repository and select it.

3. **Configure the Project**:
    - Open STM32CubeMX and configure the peripherals (GPIOs, Timers, etc.) as required.
    - Generate the initialization code and save it.

4. **Build and Upload**:
    - Build the project in STM32CubeIDE.
    - Connect the STM32F407VETX board to your computer via USB.
    - Upload the firmware to the board.


## Acknowledgments 🫡

- Special thanks to all team members who contributed to this project! Many sleepless nights were scarificed...


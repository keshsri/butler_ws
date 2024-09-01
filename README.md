# Multiple Order Delivery Robot

## Overview

This project implements a ROS2-based robot designed to handle the task of receiving multiple food delivery orders, collecting the orders from the kitchen, and delivering them to various tables. The robot is capable of handling cancellations, skipping tables with canceled orders, and following a specified sequence of operations, including returning to the kitchen after completing all deliveries before heading back to the home position.

## Features

- **Order Management**: The robot accumulates multiple orders within a specified time window before moving to the kitchen to collect them.
- **Cancellation Handling**: If an order is canceled (e.g., for a specific table), the robot will skip that table during delivery.
- **Sequential Delivery**: The robot delivers to each table sequentially, skipping any canceled orders.
- **Return to Kitchen**: After delivering to the final table, the robot returns to the kitchen before moving back to the home position.
- **State Management**: The robot's operations are managed through a well-defined state machine, including states such as `HOME`, `TO_KITCHEN`, `TO_TABLE`, `RETURN_TO_KITCHEN`, and `RETURN_HOME`.

### Prerequisites

- ROS 2 
- Python 3
- Colcon build system

### Repository Structure

butler_ws/
│
├── src/
│   └── butler_controller/
│       ├── __init__.py
│       ├── multiple_order_with_cancellation.py
│       ├── other_scripts.py
│       └── ...
├── install/
├── build/
├── README.md
└── setup.py



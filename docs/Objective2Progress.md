# Order with Timeout Robot Documentation

## Overview

**Objective**: This node is designed to handle the robot's task of receiving an order with a table number, moving from its home position to the kitchen, delivering the food to the specified table, and then returning to the home position. The node includes a timeout mechanism that returns the robot to the home position if no one attends to it (in the kitchen or at the table) within a specified period.

## System Architecture

- **Node Name**: `order_with_timeout_robot`
- **States**:
  - **HOME**: The robot is at the home position, waiting for an order.
  - **TO_KITCHEN**: The robot is moving from the home position to the kitchen.
  - **WAITING_FOR_KITCHEN_CONFIRMATION**: The robot is waiting for confirmation from the kitchen before proceeding to the table.
  - **TO_TABLE**: The robot is moving from the kitchen to the specified table.
  - **WAITING_FOR_DELIVERY_CONFIRMATION**: The robot is waiting for confirmation from the customer after delivering the food.
  - **RETURN_HOME**: The robot is returning to the home position after the delivery is confirmed or after a timeout.

- **Timeouts**:
  - **Kitchen Confirmation Timeout**: If the robot does not receive confirmation from the kitchen within a set time (e.g., 30 seconds), it will return to the home position.
  - **Delivery Confirmation Timeout**: Similarly, if the robot does not receive confirmation from the customer within a set time, it will return to the home position.

- **ROS Topics**:
  - **/order_received**: Receives the table number for the order.
  - **/kitchen_confirmation**: Receives confirmation from the kitchen.
  - **/delivery_confirmation**: Receives confirmation from the customer after delivery.

## Implementation Details

**State Machine Logic**:
- The robot starts in the `HOME` state.
- Upon receiving an order via the `/order_received` topic, it transitions to the `TO_KITCHEN` state.
- After simulating the movement to the kitchen, the robot waits in the `WAITING_FOR_KITCHEN_CONFIRMATION` state until it receives confirmation from the kitchen or times out.
- If the kitchen confirms, the robot transitions to the `TO_TABLE` state and simulates movement to the specified table. If the timeout occurs, the robot returns to the `HOME` state.
- The robot then waits in the `WAITING_FOR_DELIVERY_CONFIRMATION` state until the customer confirms delivery or the timeout occurs.
- After receiving the delivery confirmation, the robot transitions to the `RETURN_HOME` state and returns to its home position.
- Once the robot returns home, it transitions back to the `HOME` state, ready to receive the next order.

**Timeout Mechanism**:
- The robot uses a timer to manage timeouts during the waiting periods. If the timer expires without receiving the necessary confirmation, the robot returns home.


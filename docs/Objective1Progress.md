# Order to Delivery Robot Documentation

## Overview

**Objective**: This node is designed to handle the robot's task of receiving an order with a table number, moving from its home position to the kitchen, delivering the food to the specified table, and then returning to the home position.

## System Architecture

- **Node Name**: `order_to_delivery_robot`
- **States**:
  - **HOME**: The robot is at the home position, waiting for an order.
  - **TO_KITCHEN**: The robot is moving from the home position to the kitchen.
  - **TO_TABLE**: The robot is moving from the kitchen to the specified table.
  - **RETURN_HOME**: The robot is returning to the home position after delivering the food.
- **ROS Topics**:
  - **/order_received**: Receives the table number for the order.
- **ROS Services**: Not used in this node.

## Implementation Details

**State Machine Logic**:
- The robot starts in the `HOME` state.
- Upon receiving an order via the `/order_received` topic, it transitions to the `TO_KITCHEN` state.
- After simulating the movement to the kitchen, it automatically transitions to the `TO_TABLE` state.
- After delivering the food to the table, it transitions to the `RETURN_HOME` state.
- Once the robot returns home, it transitions back to the `HOME` state, ready to receive the next order.

**Single Execution**:
- The node processes each order only once. If the robot is not in the `HOME` state, any incoming orders are ignored until the robot returns home.


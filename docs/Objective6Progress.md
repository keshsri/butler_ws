# Multiple Order with Skip Robot Documentation

## Overview

**Objective**: This node is designed to handle the robot's task of receiving multiple orders with different table numbers, moving from its home position to the kitchen to collect the orders, and then delivering them to multiple tables sequentially. If no confirmation is received at a table, the robot skips to the next table. After completing all deliveries, the robot returns to the kitchen before heading back to the home position.

## System Architecture

- **Node Name**: `multiple_order_with_skip_robot`
- **States**:
  - **HOME**: The robot is at the home position, waiting for orders.
  - **TO_KITCHEN**: The robot is moving from the home position to the kitchen to collect the orders.
  - **TO_TABLE**: The robot is moving from the kitchen to each specified table to deliver the orders.
  - **RETURN_TO_KITCHEN**: The robot returns to the kitchen after completing all deliveries.
  - **RETURN_HOME**: The robot is returning to the home position after completing all deliveries and visiting the kitchen.

- **Order Accumulation**:
  - The robot accumulates multiple orders while in the `HOME` state. A timer is used to define a time window (e.g., 10 seconds) during which the robot can accept multiple orders before proceeding to the kitchen.

- **Skipping Unconfirmed Deliveries**:
  - If no confirmation is received at a table, the robot automatically skips to the next table.

- **ROS Topics**:
  - **/order_received**: Receives the table number for each order.
  - **/kitchen_confirmation**: Receives confirmation from the kitchen after collecting the orders.
  - **/table_delivery_confirmation**: Receives confirmation after delivering to each table.

## Implementation Details

**State Machine Logic**:
- The robot starts in the `HOME` state and accumulates orders as they are received.
- A timer (`order_acceptance_timer`) starts when the first order is received, allowing the robot to accept additional orders for a fixed duration (e.g., 10 seconds).
- After the timer expires, the robot transitions to the `TO_KITCHEN` state to collect the orders.
- Once the orders are collected, the robot transitions to the `TO_TABLE` state and delivers the orders to each table sequentially.
- If no confirmation is received at a table, the robot skips to the next table.
- After delivering to the final table, the robot transitions to the `RETURN_TO_KITCHEN` state, visits the kitchen, and then transitions to the `RETURN_HOME` state to move back to the home position.
- The robot clears the list of orders after returning home, making it ready for the next set of orders.

**Delays**:
- A 30-second delay is applied between state transitions to simulate more realistic behavior and allow for easier observation during testing.


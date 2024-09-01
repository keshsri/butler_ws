# Order with Cancellation and Delay Robot Documentation

## Overview

**Objective**: This node is designed to handle the robot's task of receiving an order with a table number, moving from its home position to start the task, and handling cancellations. If the order is canceled while going to the table, the robot will return to the kitchen and then go to the home position. If canceled while going to the kitchen, the robot will return directly to the home position. The node introduces a delay between each state transition to simulate more realistic robot behavior.

## System Architecture

- **Node Name**: `order_with_cancellation_robot`
- **States**:
  - **HOME**: The robot is at the home position, waiting for an order.
  - **TO_KITCHEN**: The robot is moving from the home position to the kitchen.
  - **TO_TABLE**: The robot is moving from the kitchen to the specified table.
  - **RETURN_TO_KITCHEN**: If the order is canceled while moving to the table, the robot will return to the kitchen first.
  - **RETURN_HOME**: The robot is returning to the home position after the delivery is confirmed or after a cancellation.
  - **CANCELED_RETURN_HOME**: The robot is returning directly to the home position after an order is canceled.

- **State Transition Delay**:
  - A delay of 5 seconds is introduced between state transitions to simulate more realistic behavior.

- **ROS Topics**:
  - **/order_received**: Receives the table number for the order.
  - **/order_canceled**: Receives a signal to cancel the current order.
  - **/kitchen_confirmation**: Receives confirmation from the kitchen.

## Implementation Details

**State Machine Logic**:
- The robot starts in the `HOME` state.
- Upon receiving an order via the `/order_received` topic, it transitions to the `TO_KITCHEN` state after a delay.
- If the order is canceled while the robot is moving to the kitchen (`TO_KITCHEN` state), it immediately transitions to the `CANCELED_RETURN_HOME` state and returns to the home position.
- If the robot reaches the kitchen without cancellation, it transitions to the `TO_TABLE` state.
- If the order is canceled while the robot is moving to the table (`TO_TABLE` state), it first returns to the kitchen (`RETURN_TO_KITCHEN` state) and then to the home position (`CANCELED_RETURN_HOME` state).
- After reaching the table or completing the cancellation process, the robot transitions back to the `HOME` state, ready for the next order.

**Delays**:
- A 15-second delay is applied between each state transition, making the robot's actions more observable during testing.



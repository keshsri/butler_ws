
## Custom State Machine Implementation in ROS 2

### 1. Creating the ROS 2 Package

- Created a new ROS 2 package named `butler_controller` using the following command:
  ```bash
  ros2 pkg create --build-type ament_python butler_controller
  ```

- This package will host the custom state machine logic implemented using Python.

### 2. Implementing the Custom State Machine

- Implemented a custom state machine inside a new script `custom_state_machine.py` within the `butler_controller` package.
- The state machine transitions through the following states:
  - **HOME**: The initial state where the robot waits for an order.
  - **TO_KITCHEN**: The robot moves to the kitchen.
  - **AT_KITCHEN**: The robot waits at the kitchen for food preparation.
  - **TO_TABLE**: The robot delivers food to the customer's table.
  - **AT_TABLE**: The robot confirms the delivery with the customer.
  - **RETURN_HOME**: The robot returns to the home position after delivery.

- Example code for the state machine:
  ```python
  from transitions import Machine
  import time
  import random

  class ButlerRobot:
      def __init__(self):
          states = ['home', 'to_kitchen', 'at_kitchen', 'to_table', 'at_table', 'return_home']
          self.machine = Machine(model=self, states=states, initial='home')
          self.machine.add_transition(trigger='order_received', source='home', dest='to_kitchen')
          self.machine.add_transition(trigger='arrived_kitchen', source='to_kitchen', dest='at_kitchen')
          self.machine.add_transition(trigger='food_ready', source='at_kitchen', dest='to_table')
          self.machine.add_transition(trigger='arrived_table', source='to_table', dest='at_table')
          self.machine.add_transition(trigger='delivery_confirmed', source='at_table', dest='return_home')
          self.machine.add_transition(trigger='arrived_home', source='return_home', dest='home')

      def simulate_order(self):
          print("Order received, moving to kitchen...")
          self.order_received()
          time.sleep(2)
          print("Arrived at the kitchen.")
          self.arrived_kitchen()
          time.sleep(1)
          if random.random() > 0.5:
              print("An issue occurred, resetting...")
              self.reset()
          else:
              print("Food is ready, moving to the table...")
              self.food_ready()
              time.sleep(2)
              print("Arrived at the table.")
              self.arrived_table()
              time.sleep(1)
              print("Delivery confirmed, returning home...")
              self.delivery_confirmed()
              time.sleep(2)
              print("Arrived home.")
              self.arrived_home()

  butler = ButlerRobot()
  butler.simulate_order()
  ```

### 3. Modifying `setup.py`

- The `setup.py` file was updated to include the `custom_state_machine.py` script as an entry point:
  ```python
  entry_points={
      'console_scripts': [
          'custom_state_machine = butler_controller.custom_state_machine:main',
      ],
  },
  ```

### 4. Building and Running the Package

- Built the package and sourced the workspace:
  ```bash
  cd ~/butler_ws
  colcon build
  source install/setup.bash
  ```

- The custom state machine node was then run using:
  ```bash
  ros2 run butler_controller custom_state_machine
  ```

### 6. Next Steps

- **Integration with ROS Topics/Services**: Next, the plan is to integrate ROS communication to control the robot’s behavior based on real-world events.
- **State Expansion**: May introduce additional states or error handling to improve the state machine's robustness.

---

*End of document.*

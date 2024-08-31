import rclpy
from rclpy.node import Node
from enum import Enum

class State(Enum):
    HOME = 0
    TO_KITCHEN = 1
    AT_KITCHEN = 2
    TO_TABLE = 3
    AT_TABLE = 4
    RETURN_HOME = 5

class ButlerRobot(Node):
    def __init__(self):
        super().__init__('butler_robot')
        self.state = State.HOME
        self.get_logger().info(f"Initial state: {self.state.name}")

    def transition_to(self, new_state):
        self.get_logger().info(f"Transitioning from {self.state.name} to {new_state.name}")
        self.state = new_state
        self.execute_state_logic()

    def execute_state_logic(self):
        if self.state == State.HOME:
            self.get_logger().info("Waiting for order...")
            # Simulate receiving an order after a delay
            self.create_timer(2, lambda: self.transition_to(State.TO_KITCHEN))

        elif self.state == State.TO_KITCHEN:
            self.get_logger().info("Moving to the kitchen...")
            # Simulate arriving at the kitchen
            self.create_timer(2, lambda: self.transition_to(State.AT_KITCHEN))

        elif self.state == State.AT_KITCHEN:
            self.get_logger().info("At the kitchen. Waiting for food...")
            # Simulate food ready after a delay
            self.create_timer(2, lambda: self.transition_to(State.TO_TABLE))

        elif self.state == State.TO_TABLE:
            self.get_logger().info("Delivering food to the table...")
            # Simulate arriving at the table
            self.create_timer(2, lambda: self.transition_to(State.AT_TABLE))

        elif self.state == State.AT_TABLE:
            self.get_logger().info("At the table. Confirming delivery...")
            # Simulate confirmation and return home
            self.create_timer(2, lambda: self.transition_to(State.RETURN_HOME))

        elif self.state == State.RETURN_HOME:
            self.get_logger().info("Returning home...")
            # Simulate arriving home
            self.create_timer(2, lambda: self.transition_to(State.HOME))

def main(args=None):
    rclpy.init(args=args)
    butler_robot = ButlerRobot()

    # Start the state machine by executing the logic for the initial state
    butler_robot.execute_state_logic()

    rclpy.spin(butler_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

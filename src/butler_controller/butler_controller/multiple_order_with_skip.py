import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class State:
    HOME = "HOME"
    TO_KITCHEN = "TO_KITCHEN"
    TO_TABLE = "TO_TABLE"
    RETURN_TO_KITCHEN = "RETURN_TO_KITCHEN"
    RETURN_HOME = "RETURN_HOME"

class MultipleOrderWithSkipRobot(Node):
    def __init__(self):
        super().__init__('multiple_order_with_skip_robot')
        self.state = State.HOME
        self.orders = []
        self.current_order_index = 0
        self.transition_delay = 30.0  # 30 seconds delay between states
        self.order_acceptance_timer = None
        self.order_acceptance_duration = 60.0  # Time window to accept multiple orders

        # Subscribers
        self.order_received_sub = self.create_subscription(
            String,
            '/order_received',
            self.order_received_callback,
            10)

        self.kitchen_confirmation_sub = self.create_subscription(
            String,
            '/kitchen_confirmation',
            self.kitchen_confirmation_callback,
            10)

        self.table_delivery_confirmation_sub = self.create_subscription(
            String,
            '/table_delivery_confirmation',
            self.table_delivery_confirmation_callback,
            10)

        # Publisher for state updates
        self.state_pub = self.create_publisher(String, '/current_state', 10)

    def publish_state(self):
        msg = String()
        msg.data = f"Current state: {self.state}"
        self.state_pub.publish(msg)

    def transition_to(self, new_state):
        self.get_logger().info(f"Transitioning from {self.state} to {new_state} in {self.transition_delay} seconds...")
        self.create_timer(self.transition_delay, lambda: self._perform_transition(new_state))

    def _perform_transition(self, new_state):
        self.state = new_state
        self.publish_state()
        if new_state == State.TO_KITCHEN:
            self.move_to_kitchen()
        elif new_state == State.TO_TABLE:
            self.move_to_table()
        elif new_state == State.RETURN_TO_KITCHEN:
            self.return_to_kitchen()
        elif new_state == State.RETURN_HOME:
            self.return_home()

    def order_received_callback(self, msg):
        if self.state == State.HOME:  # Only accept new orders if at HOME
            self.orders.append(msg.data)
            self.get_logger().info(f'Received order for table: {msg.data}')
            if not self.order_acceptance_timer:  # Start the timer to collect multiple orders
                self.get_logger().info(f"Starting order acceptance timer for {self.order_acceptance_duration} seconds...")
                self.order_acceptance_timer = self.create_timer(self.order_acceptance_duration, self.process_orders)

    def process_orders(self):
        if self.state == State.HOME and self.orders:
            self.get_logger().info(f"Finished accepting orders: {self.orders}. Moving to kitchen...")
            self.order_acceptance_timer.cancel()
            self.order_acceptance_timer = None
            self.transition_to(State.TO_KITCHEN)
        else:
            self.get_logger().info("No orders to process.")

    def kitchen_confirmation_callback(self, msg):
        if self.state == State.TO_KITCHEN:
            self.get_logger().info('Kitchen confirmed. Moving to first table...')
            self.current_order_index = 0
            self.transition_to(State.TO_TABLE)

    def table_delivery_confirmation_callback(self, msg):
        if self.state == State.TO_TABLE and self.current_order_index < len(self.orders):
            self.get_logger().info(f'Delivery confirmed for table {self.orders[self.current_order_index]}')
            self.current_order_index += 1
            if self.current_order_index < len(self.orders):
                self.get_logger().info(f'Moving to next table: {self.orders[self.current_order_index]}')
                self.transition_to(State.TO_TABLE)
            else:
                self.get_logger().info('All deliveries completed. Returning to kitchen...')
                self.transition_to(State.RETURN_TO_KITCHEN)

    def move_to_kitchen(self):
        self.get_logger().info('Moving to kitchen...')
        self.transition_to(State.TO_TABLE)

    def move_to_table(self):
        if self.current_order_index < len(self.orders):
            self.get_logger().info(f'Moving to table {self.orders[self.current_order_index]}...')
            # Simulate waiting for delivery confirmation (skip to next if none received after a delay)
            self.create_timer(self.transition_delay, self.check_for_confirmation)
        else:
            self.transition_to(State.RETURN_TO_KITCHEN)

    def check_for_confirmation(self):
        if self.state == State.TO_TABLE and self.current_order_index < len(self.orders):
            self.get_logger().info(f'No confirmation received for table {self.orders[self.current_order_index]}. Moving to the next table...')
            self.current_order_index += 1
            if self.current_order_index < len(self.orders):
                self.transition_to(State.TO_TABLE)
            else:
                self.transition_to(State.RETURN_TO_KITCHEN)

    def return_to_kitchen(self):
        self.get_logger().info('Returning to kitchen...')
        self.transition_to(State.RETURN_HOME)

    def return_home(self):
        self.get_logger().info('Returning to home...')
        self.transition_to(State.HOME)
        self.get_logger().info('Task complete.')
        self.orders.clear()  # Clear orders after returning home

def main(args=None):
    rclpy.init(args=args)
    multiple_order_with_skip_robot = MultipleOrderWithSkipRobot()
    rclpy.spin(multiple_order_with_skip_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

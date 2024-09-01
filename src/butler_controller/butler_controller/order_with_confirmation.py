import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class State:
    HOME = "HOME"
    TO_KITCHEN = "TO_KITCHEN"
    WAITING_FOR_KITCHEN_CONFIRMATION = "WAITING_FOR_KITCHEN_CONFIRMATION"
    TO_TABLE = "TO_TABLE"
    WAITING_FOR_DELIVERY_CONFIRMATION = "WAITING_FOR_DELIVERY_CONFIRMATION"
    RETURN_HOME = "RETURN_HOME"

class OrderWithConfirmationsRobot(Node):
    def __init__(self):
        super().__init__('order_with_confirmations_robot')
        self.state = State.HOME
        self.table_number = None

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

        self.delivery_confirmation_sub = self.create_subscription(
            String,
            '/delivery_confirmation',
            self.delivery_confirmation_callback,
            10)

        # Publisher for state updates
        self.state_pub = self.create_publisher(String, '/current_state', 10)

    def publish_state(self):
        msg = String()
        msg.data = f"Current state: {self.state}"
        self.state_pub.publish(msg)

    def transition_to(self, new_state):
        self.get_logger().info(f"Transitioning from {self.state} to {new_state}")
        self.state = new_state
        self.publish_state()

    def order_received_callback(self, msg):
        if self.state == State.HOME:  # Process the order only if at HOME
            self.table_number = msg.data
            self.get_logger().info(f'Received order for table: {self.table_number}')
            self.transition_to(State.TO_KITCHEN)
            self.move_to_kitchen()
        else:
            self.get_logger().info('Ignoring order; robot is not at HOME state.')

    def move_to_kitchen(self):
        self.get_logger().info('Moving to kitchen...')
        # Simulate moving to the kitchen
        self.transition_to(State.WAITING_FOR_KITCHEN_CONFIRMATION)

    def kitchen_confirmation_callback(self, msg):
        if self.state == State.WAITING_FOR_KITCHEN_CONFIRMATION:
            self.get_logger().info('Kitchen confirmed. Moving to table...')
            self.transition_to(State.TO_TABLE)
            self.move_to_table()

    def move_to_table(self):
        self.get_logger().info(f'Moving to table {self.table_number}...')
        # Simulate moving to the table
        self.transition_to(State.WAITING_FOR_DELIVERY_CONFIRMATION)

    def delivery_confirmation_callback(self, msg):
        if self.state == State.WAITING_FOR_DELIVERY_CONFIRMATION:
            self.get_logger().info('Delivery confirmed. Returning home...')
            self.transition_to(State.RETURN_HOME)
            self.return_home()

    def return_home(self):
        self.get_logger().info('Returning to home...')
        # Simulate returning to the home position
        self.transition_to(State.HOME)
        self.get_logger().info('Task complete.')

def main(args=None):
    rclpy.init(args=args)
    order_with_confirmations_robot = OrderWithConfirmationsRobot()
    rclpy.spin(order_with_confirmations_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

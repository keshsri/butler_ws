import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.timer import Timer
from rclpy.duration import Duration

class State:
    HOME = "HOME"
    TO_KITCHEN = "TO_KITCHEN"
    TO_TABLE = "TO_TABLE"
    RETURN_TO_KITCHEN = "RETURN_TO_KITCHEN"
    RETURN_HOME = "RETURN_HOME"
    CANCELED_RETURN_HOME = "CANCELED_RETURN_HOME"

class OrderWithCancellationRobot(Node):
    def __init__(self):
        super().__init__('order_with_cancellation_robot')
        self.state = State.HOME
        self.table_number = None
        self.order_canceled = False
        self.transition_delay = 15.0  # 5 seconds delay between states

        # Subscribers
        self.order_received_sub = self.create_subscription(
            String,
            '/order_received',
            self.order_received_callback,
            10)

        self.order_canceled_sub = self.create_subscription(
            String,
            '/order_canceled',
            self.order_canceled_callback,
            10)

        self.kitchen_confirmation_sub = self.create_subscription(
            String,
            '/kitchen_confirmation',
            self.kitchen_confirmation_callback,
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
        elif new_state == State.CANCELED_RETURN_HOME or new_state == State.RETURN_HOME:
            self.return_home()

    def order_received_callback(self, msg):
        if self.state == State.HOME:  # Process the order only if at HOME
            self.table_number = msg.data
            self.order_canceled = False  # Reset cancellation flag
            self.get_logger().info(f'Received order for table: {self.table_number}')
            self.transition_to(State.TO_KITCHEN)
        else:
            self.get_logger().info('Ignoring order; robot is not at HOME state.')

    def order_canceled_callback(self, msg):
        if self.state == State.TO_KITCHEN:
            self.get_logger().info('Order canceled. Returning home...')
            self.order_canceled = True
            self.transition_to(State.CANCELED_RETURN_HOME)
        elif self.state == State.TO_TABLE:
            self.get_logger().info('Order canceled. Returning to kitchen...')
            self.order_canceled = True
            self.transition_to(State.RETURN_TO_KITCHEN)

    def move_to_kitchen(self):
        self.get_logger().info('Moving to kitchen...')
        if not self.order_canceled:
            self.transition_to(State.TO_TABLE)
        else:
            self.transition_to(State.CANCELED_RETURN_HOME)

    def kitchen_confirmation_callback(self, msg):
        if self.state == State.TO_TABLE and not self.order_canceled:
            self.get_logger().info('Kitchen confirmed. Moving to table...')
            self.transition_to(State.TO_TABLE)

    def move_to_table(self):
        self.get_logger().info(f'Moving to table {self.table_number}...')
        if self.order_canceled:
            self.transition_to(State.RETURN_TO_KITCHEN)
        else:
            self.transition_to(State.RETURN_HOME)

    def return_to_kitchen(self):
        self.get_logger().info('Returning to kitchen...')
        self.transition_to(State.CANCELED_RETURN_HOME)

    def return_home(self):
        self.get_logger().info('Returning to home...')
        self.transition_to(State.HOME)
        self.get_logger().info('Task complete.')

def main(args=None):
    rclpy.init(args=args)
    order_with_cancellation_robot = OrderWithCancellationRobot()
    rclpy.spin(order_with_cancellation_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

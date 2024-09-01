import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class State:
    HOME = "HOME"
    TO_KITCHEN = "TO_KITCHEN"
    TO_TABLE = "TO_TABLE"
    RETURN_HOME = "RETURN_HOME"

class OrderToDeliveryRobot(Node):
    def __init__(self):
        super().__init__('order_to_delivery_robot')
        self.state = State.HOME
        self.table_number = None

        # Subscriber to receive order
        self.order_received_sub = self.create_subscription(
            String,
            '/order_received',
            self.order_received_callback,
            10)

        # Example Publisher for state updates
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
        if self.state == State.HOME:  # Only accept new orders when at HOME
            self.table_number = msg.data
            self.get_logger().info(f'Received order for table: {self.table_number}')
            self.transition_to(State.TO_KITCHEN)
            self.move_to_kitchen()

    def move_to_kitchen(self):
        # Simulate moving to the kitchen
        self.get_logger().info('Moving to kitchen...')
        # After reaching the kitchen, move to the table
        self.transition_to(State.TO_TABLE)
        self.move_to_table()

    def move_to_table(self):
        # Simulate moving to the table
        self.get_logger().info(f'Moving to table {self.table_number}...')
        # After delivering, return home
        self.transition_to(State.RETURN_HOME)
        self.return_home()

    def return_home(self):
        # Simulate returning to the home position
        self.get_logger().info('Returning to home...')
        self.transition_to(State.HOME)
        self.get_logger().info('Task complete.')

def main(args=None):
    rclpy.init(args=args)
    order_to_delivery_robot = OrderToDeliveryRobot()
    rclpy.spin(order_to_delivery_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

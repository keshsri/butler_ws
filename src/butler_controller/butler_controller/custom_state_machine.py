import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class State:
    HOME = "HOME"
    TO_KITCHEN = "TO_KITCHEN"
    AT_KITCHEN = "AT_KITCHEN"
    TO_TABLE = "TO_TABLE"
    AT_TABLE = "AT_TABLE"
    RETURN_HOME = "RETURN_HOME"

class ButlerRobot(Node):
    def __init__(self):
        super().__init__('butler_robot')
        self.state = State.HOME

        # Define Subscribers
        self.order_received_sub = self.create_subscription(
            String,
            '/order_received',
            self.order_received_callback,
            10)

        self.kitchen_arrival_sub = self.create_subscription(
            String,
            '/kitchen_arrival',
            self.kitchen_arrival_callback,
            10)

        self.food_ready_sub = self.create_subscription(
            String,
            '/food_ready',
            self.food_ready_callback,
            10)

        self.table_arrival_sub = self.create_subscription(
            String,
            '/table_arrival',
            self.table_arrival_callback,
            10)

        self.delivery_confirmed_sub = self.create_subscription(
            String,
            '/delivery_confirmed',
            self.delivery_confirmed_callback,
            10)

        self.cancel_order_sub = self.create_subscription(
            String,
            '/cancel_order',
            self.cancel_order_callback,
            10)

        # Define Publishers (if necessary, for example to publish state changes)
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
        self.get_logger().info(f'Received order: {msg.data}')
        self.transition_to(State.TO_KITCHEN)

    def kitchen_arrival_callback(self, msg):
        self.get_logger().info('Arrived at the kitchen')
        self.transition_to(State.AT_KITCHEN)

    def food_ready_callback(self, msg):
        self.get_logger().info('Food is ready, moving to table')
        self.transition_to(State.TO_TABLE)

    def table_arrival_callback(self, msg):
        self.get_logger().info('Arrived at the table')
        self.transition_to(State.AT_TABLE)

    def delivery_confirmed_callback(self, msg):
        self.get_logger().info('Delivery confirmed, returning home')
        self.transition_to(State.RETURN_HOME)

    def cancel_order_callback(self, msg):
        self.get_logger().info('Order canceled, returning home')
        self.transition_to(State.HOME)

def main(args=None):
    rclpy.init(args=args)
    butler_robot = ButlerRobot()
    rclpy.spin(butler_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

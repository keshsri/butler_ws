import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

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

        # Publisher for sending navigation goals
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Define Services
        self.request_order_srv = self.create_service(Trigger, '/request_order', self.request_order_callback)
        self.confirm_kitchen_srv = self.create_service(Trigger, '/confirm_kitchen', self.confirm_kitchen_callback)
        self.confirm_delivery_srv = self.create_service(Trigger, '/confirm_delivery', self.confirm_delivery_callback)
        self.abort_mission_srv = self.create_service(Trigger, '/abort_mission', self.abort_mission_callback)

    def send_navigation_goal(self, x, y, theta):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.z = theta
        goal_msg.pose.orientation.w = 1.0
        self.nav_goal_pub.publish(goal_msg)
        self.get_logger().info(f"Navigation goal sent: x={x}, y={y}, theta={theta}")

    def transition_to(self, new_state):
        self.get_logger().info(f"Transitioning from {self.state} to {new_state}")
        self.state = new_state

        if self.state == State.TO_KITCHEN:
            self.send_navigation_goal(1.0, 1.0, 0.0)  # Example coordinates
        elif self.state == State.TO_TABLE:
            self.send_navigation_goal(2.0, 2.0, 0.0)  # Example coordinates
        elif self.state == State.RETURN_HOME:
            self.send_navigation_goal(0.0, 0.0, 0.0)  # Return to home

    # Service Callbacks
    def request_order_callback(self, request, response):
        self.get_logger().info('Order requested')
        self.transition_to(State.TO_KITCHEN)
        response.success = True
        response.message = "Order started, moving to kitchen."
        return response

    def confirm_kitchen_callback(self, request, response):
        if self.state == State.TO_KITCHEN or self.state == State.AT_KITCHEN:
            self.get_logger().info('Kitchen confirmed')
            self.transition_to(State.TO_TABLE)
            response.success = True
            response.message = "Kitchen confirmed, moving to table."
        else:
            response.success = False
            response.message = "Cannot confirm kitchen, wrong state."
        return response

    def confirm_delivery_callback(self, request, response):
        if self.state == State.AT_TABLE:
            self.get_logger().info('Delivery confirmed')
            self.transition_to(State.RETURN_HOME)
            response.success = True
            response.message = "Delivery confirmed, returning home."
        else:
            response.success = False
            response.message = "Cannot confirm delivery, wrong state."
        return response

    def abort_mission_callback(self, request, response):
        self.get_logger().info('Mission aborted, returning home')
        self.transition_to(State.HOME)
        response.success = True
        response.message = "Mission aborted, returning home."
        return response

def main(args=None):
    rclpy.init(args=args)
    butler_robot = ButlerRobot()
    rclpy.spin(butler_robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

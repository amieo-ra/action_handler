import rclpy
from rclpy.node import Node
from topological_navigation_msgs.msg import ExecutePolicyModeActionGoal

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(ExecutePolicyModeActionGoal, 'topological_navigation/execute_policy_mode/goal', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ExecutePolicyModeActionGoal()
        msg.route.edge_id = (['WayPoint144_WayPoint141', 'WayPoint141_WayPoint155'])
        Last_WayPoint = 'WayPoint155'
        msg.route.source = (['WayPoint155'])
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.route.edge_id)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from topological_navigation_msgs.msg import ExecutePolicyModeFeedback, ExecutePolicyMode, ExecutePolicyModeResult
from topological_navigation_msgs.action import GotoNode, ExecutePolicyMode
from std_msgs.msg import String  # Adjust according to the data types you expect
from actionlib_msgs.msg import GoalID, GoalStatusArray


class ActionMiddleman(Node):
    def __init__(self):
        super().__init__('action_middleman')
        # Initialize the action client

        self.action_server_name = '/topological_navigation'        
        self.client = ActionClient(self, GotoNode, self.action_server_name)

        # Subscribe to the custom topics
        self.goal_subscriber = self.create_subscription(ExecutePolicyMode, 'topological_navigation/execute_policy_mode/goal', self.goal_callback, 10)
        self.cancel_subscriber = self.create_subscription(GoalID, 'topological_navigation/execute_policy_mode/cancel', self.cancel_callback, 10)
        # Publishers for feedback and result
        self.feedback_publisher = self.create_publisher(ExecutePolicyModeFeedback, 'topological_navigation/execute_policy_mode/feedback', 10)
        self.result_publisher = self.create_publisher(ExecutePolicyModeResult, 'topological_navigation/execute_policy_mode/result', 10)
        self.status_publisher = self.create_publisher(GoalStatusArray, 'topological_navigation/execute_policy_mode/status', 10)
        self.current_goal = None

    def callback_goto_client(self, msg):
        self.get_logger().info(f'Received new goal: {msg.data}')
        goal = msg.route.edge_id[-1].split('_')[-1]
        # Parse the message according to your actual message structure
        goal_msg = GotoNode.Goal(goal)
        #goal_msg = goal
        #goal_msg.order = int(msg.data)  # Assuming the message is just an integer order
        self.current_goal = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.current_goal.add_done_callback(self.goal_response_callback)

    def cancel_callback(self, msg):
        self.get_logger().info('Received cancel request')
        if self.current_goal:
            self.current_goal.cancel_goal_async()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.partial_sequence}')
        feedback_array = ExecutePolicyModeFeedback()
        feedback_array.data = feedback_msg.feedback.partial_sequence
        self.feedback_publisher.publish(feedback_array)

    def goal_response_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal completed with result: {result.sequence}')
        result_array = ExecutePolicyModeResult()
        result_array.data = result.sequence
        self.result_publisher.publish(result_array)

def main(args=None):
    rclpy.init(args=args)
    middleman_node = ActionMiddleman()
    rclpy.spin(middleman_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
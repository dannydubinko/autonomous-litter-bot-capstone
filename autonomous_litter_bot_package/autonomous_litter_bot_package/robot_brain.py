import rclpy
# Import your class from the file you saved it in
from autonomous_litter_bot_package.pose_tracker import PoseTracker

class RobotBrain(PoseTracker):
    def __init__(self):
        # This calls the __init__ of PoseTracker, setting up the subscriber
        super().__init__() 
        
        # Now add your specific "Brain" logic
        self.get_logger().info("Robot Brain is starting up...")
        
        # Example: A timer that makes a decision every 2 seconds
        self.decision_timer = self.create_timer(2.0, self.make_decision)

    def make_decision(self):
        # You can now call the method directly because RobotBrain inherited it!
        current_position = self.capture_my_position_now()
        
        if current_position is not None:
            x, y, theta = current_position
            self.get_logger().info(f"Thinking... I am currently at X: {x:.2f}, Y: {y:.2f}, Theta: {theta:.2f}")
            # Add logic here: "If X > 5, turn left", etc.

def main(args=None):
    rclpy.init(args=args)
    node = RobotBrain()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
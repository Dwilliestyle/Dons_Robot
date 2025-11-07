#!/usr/bin/env python3
"""
Rover line following integration
Connects the line follower to your rover's motor control
"""

import time
import logging
from line_follower import LineFollower

# You'll need to import your existing motor control module
# For example:
# from your_motor_controller import MotorController
# Or if using GPIO directly:
# import RPi.GPIO as GPIO

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LineFollowingRover:
    def __init__(self):
        """Initialize the rover with line following capability"""
        # Initialize line follower
        # Set debug=True to see camera view windows (useful for tuning)
        self.follower = LineFollower(camera_index=0, debug=False)
        
        # Initialize your motor controller here
        # self.motors = MotorController()
        # OR setup GPIO pins for motor control
        
        # Control parameters
        self.base_speed = 40      # Base forward speed (0-100)
        self.turn_factor = 35     # How aggressively to turn (0-100)
        self.min_speed = 15       # Minimum speed to keep motors moving
        
        # State
        self.enabled = False
        self.last_line_seen = time.time()
        self.search_timeout = 2.0  # Seconds before stopping if line lost
        
    def setup_motors(self):
        """
        Initialize your motor control here
        This will depend on your specific motor driver/setup
        """
        # Example for L298N or similar motor driver:
        """
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        
        # Motor A (Left)
        self.MOTOR_A_EN = 17
        self.MOTOR_A_IN1 = 27
        self.MOTOR_A_IN2 = 22
        
        # Motor B (Right)
        self.MOTOR_B_EN = 18
        self.MOTOR_B_IN1 = 23
        self.MOTOR_B_IN2 = 24
        
        # Setup pins
        GPIO.setup(self.MOTOR_A_EN, GPIO.OUT)
        GPIO.setup(self.MOTOR_A_IN1, GPIO.OUT)
        GPIO.setup(self.MOTOR_A_IN2, GPIO.OUT)
        GPIO.setup(self.MOTOR_B_EN, GPIO.OUT)
        GPIO.setup(self.MOTOR_B_IN1, GPIO.OUT)
        GPIO.setup(self.MOTOR_B_IN2, GPIO.OUT)
        
        # Setup PWM for speed control
        self.pwm_a = GPIO.PWM(self.MOTOR_A_EN, 100)
        self.pwm_b = GPIO.PWM(self.MOTOR_B_EN, 100)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        """
        pass
    
    def set_motor_speeds(self, left_speed, right_speed):
        """
        Set motor speeds
        
        Args:
            left_speed: -100 to 100 (negative for reverse)
            right_speed: -100 to 100 (negative for reverse)
        """
        # Implement based on your motor controller
        # This is an example for L298N:
        """
        # Left motor
        if left_speed > 0:
            GPIO.output(self.MOTOR_A_IN1, GPIO.HIGH)
            GPIO.output(self.MOTOR_A_IN2, GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(abs(left_speed))
        elif left_speed < 0:
            GPIO.output(self.MOTOR_A_IN1, GPIO.LOW)
            GPIO.output(self.MOTOR_A_IN2, GPIO.HIGH)
            self.pwm_a.ChangeDutyCycle(abs(left_speed))
        else:
            GPIO.output(self.MOTOR_A_IN1, GPIO.LOW)
            GPIO.output(self.MOTOR_A_IN2, GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(0)
        
        # Right motor (similar for MOTOR_B)
        ...
        """
        
        # For now, just log the speeds
        logger.info(f"Motor speeds - L: {left_speed:+3.0f}, R: {right_speed:+3.0f}")
    
    def stop_motors(self):
        """Stop both motors"""
        self.set_motor_speeds(0, 0)
    
    def follow_line(self):
        """Main line following behavior"""
        # Get motor speeds from line follower
        left, right = self.follower.get_motor_speeds(
            base_speed=self.base_speed,
            turn_factor=self.turn_factor
        )
        
        # Check if line is detected
        if self.follower.line_detected:
            self.last_line_seen = time.time()
            
            # Apply minimum speed threshold
            if abs(left) < self.min_speed and abs(left) > 0:
                left = self.min_speed if left > 0 else -self.min_speed
            if abs(right) < self.min_speed and abs(right) > 0:
                right = self.min_speed if right > 0 else -self.min_speed
            
            # Set motor speeds
            self.set_motor_speeds(left, right)
            
        else:
            # Line not detected - check timeout
            time_since_line = time.time() - self.last_line_seen
            
            if time_since_line < self.search_timeout:
                # Recently lost line - try to find it
                # You could implement a search pattern here
                # For now, just slow down and continue straight
                self.set_motor_speeds(self.min_speed, self.min_speed)
                logger.info("Searching for line...")
            else:
                # Line lost for too long - stop
                self.stop_motors()
                logger.warning("Line lost - stopping")
    
    def start(self):
        """Start line following mode"""
        logger.info("Starting line following mode")
        self.setup_motors()
        self.follower.start()
        self.enabled = True
        
        try:
            while self.enabled:
                self.follow_line()
                time.sleep(0.05)  # Control loop rate (~20 Hz)
                
        except KeyboardInterrupt:
            logger.info("Line following interrupted by user")
        finally:
            self.stop()
    
    def stop(self):
        """Stop line following mode"""
        logger.info("Stopping line following mode")
        self.enabled = False
        self.stop_motors()
        self.follower.stop()
    
    def calibrate(self):
        """
        Interactive calibration mode to tune parameters
        """
        print("\n=== Line Follower Calibration Mode ===")
        print("Place the rover on your line and adjust parameters")
        print("Commands:")
        print("  b/B - Decrease/Increase base speed")
        print("  t/T - Decrease/Increase turn factor")
        print("  m/M - Decrease/Increase minimum speed")
        print("  d   - Toggle debug display")
        print("  s   - Start/stop following")
        print("  q   - Quit calibration")
        print("\nCurrent parameters:")
        print(f"  Base speed: {self.base_speed}")
        print(f"  Turn factor: {self.turn_factor}")
        print(f"  Min speed: {self.min_speed}")
        
        self.follower.start()
        following = False
        
        try:
            import cv2
            while True:
                key = cv2.waitKey(100) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('b'):
                    self.base_speed = max(0, self.base_speed - 5)
                    print(f"Base speed: {self.base_speed}")
                elif key == ord('B'):
                    self.base_speed = min(100, self.base_speed + 5)
                    print(f"Base speed: {self.base_speed}")
                elif key == ord('t'):
                    self.turn_factor = max(0, self.turn_factor - 5)
                    print(f"Turn factor: {self.turn_factor}")
                elif key == ord('T'):
                    self.turn_factor = min(100, self.turn_factor + 5)
                    print(f"Turn factor: {self.turn_factor}")
                elif key == ord('m'):
                    self.min_speed = max(0, self.min_speed - 5)
                    print(f"Min speed: {self.min_speed}")
                elif key == ord('M'):
                    self.min_speed = min(100, self.min_speed + 5)
                    print(f"Min speed: {self.min_speed}")
                elif key == ord('d'):
                    self.follower.debug = not self.follower.debug
                    print(f"Debug display: {self.follower.debug}")
                elif key == ord('s'):
                    following = not following
                    print(f"Following: {following}")
                    if not following:
                        self.stop_motors()
                
                if following:
                    self.follow_line()
                
        except ImportError:
            print("OpenCV not available for keyboard input")
            print("Running in follow mode for 30 seconds...")
            for _ in range(600):  # 30 seconds at 0.05s per loop
                self.follow_line()
                time.sleep(0.05)
        
        finally:
            self.stop_motors()
            self.follower.stop()
            print("\nCalibration complete!")
            print(f"Final parameters:")
            print(f"  Base speed: {self.base_speed}")
            print(f"  Turn factor: {self.turn_factor}")
            print(f"  Min speed: {self.min_speed}")


if __name__ == "__main__":
    rover = LineFollowingRover()
    
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        rover.calibrate()
    else:
        print("Starting line following rover...")
        print("Press Ctrl+C to stop")
        rover.start()
#!/usr/bin/env python3
"""
Camera-based line following for Raspberry Pi rover
Uses OpenCV to detect and follow a line on the ground
"""

import cv2
import numpy as np
import time
from threading import Thread, Lock
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LineFollower:
    def __init__(self, camera_index=0, debug=False):
        """
        Initialize the line follower
        
        Args:
            camera_index: Camera device index (0 for default camera)
            debug: If True, shows debug windows with processed images
        """
        self.camera = cv2.VideoCapture(camera_index)
        self.debug = debug
        self.running = False
        self.line_position = 0  # -1 to 1, where 0 is centered
        self.line_detected = False
        self.frame_lock = Lock()
        self.current_frame = None
        self.processed_frame = None
        
        # Image processing parameters (tune these for your line color/conditions)
        self.blur_kernel = 5
        self.threshold_low = 50   # For black line on white background
        self.threshold_high = 255
        self.min_line_width = 20  # Minimum pixels wide for valid line
        
        # Region of Interest - bottom portion of image where line should be
        # Adjust these based on your camera mounting angle
        self.roi_top_percent = 0.5    # Start looking at bottom 50% of image
        self.roi_bottom_percent = 1.0  # Look all the way to bottom
        
    def process_frame(self, frame):
        """
        Process a single frame to detect the line
        
        Returns:
            processed_frame: The processed binary image
            line_center_x: X coordinate of line center (None if no line found)
        """
        height, width = frame.shape[:2]
        
        # Define Region of Interest (ROI)
        roi_top = int(height * self.roi_top_percent)
        roi_bottom = int(height * self.roi_bottom_percent)
        roi = frame[roi_top:roi_bottom, :]
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
        
        # Threshold to create binary image
        # For black line on white: use standard threshold
        # For white line on black: use inverse threshold
        _, binary = cv2.threshold(blurred, self.threshold_low, self.threshold_high, 
                                  cv2.THRESH_BINARY_INV)  # INV for black line
        
        # Find contours in the binary image
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        line_center_x = None
        
        if contours:
            # Find the largest contour (assumed to be the line)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get bounding box of the contour
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Check if contour is wide enough to be the line
            if w >= self.min_line_width:
                # Calculate center of the line
                line_center_x = x + w // 2
                
                # Draw debugging info on processed frame if in debug mode
                if self.debug:
                    cv2.rectangle(binary, (x, y), (x + w, y + h), 128, 2)
                    cv2.circle(binary, (line_center_x, y + h // 2), 5, 128, -1)
        
        return binary, line_center_x
    
    def calculate_line_position(self, line_center_x, frame_width):
        """
        Calculate normalized line position (-1 to 1)
        
        Args:
            line_center_x: X coordinate of line center
            frame_width: Width of the frame
            
        Returns:
            Normalized position: -1 (far left) to 0 (center) to 1 (far right)
        """
        if line_center_x is None:
            return None
        
        frame_center = frame_width // 2
        # Normalize to -1 to 1 range
        position = (line_center_x - frame_center) / frame_center
        return position
    
    def tracking_loop(self):
        """Main loop for continuous line tracking"""
        while self.running:
            ret, frame = self.camera.read()
            if not ret:
                logger.warning("Failed to read frame from camera")
                continue
            
            # Process the frame
            processed, line_center_x = self.process_frame(frame)
            
            # Calculate line position
            height, width = frame.shape[:2]
            position = self.calculate_line_position(line_center_x, width)
            
            # Update shared variables
            with self.frame_lock:
                self.current_frame = frame
                self.processed_frame = processed
                if position is not None:
                    self.line_position = position
                    self.line_detected = True
                else:
                    self.line_detected = False
            
            # Show debug windows if enabled
            if self.debug:
                self.show_debug_windows()
            
            # Small delay to limit CPU usage
            time.sleep(0.03)  # ~30 FPS
    
    def show_debug_windows(self):
        """Display debug windows showing original and processed frames"""
        with self.frame_lock:
            if self.current_frame is not None:
                # Draw center line and detected position on original frame
                display_frame = self.current_frame.copy()
                height, width = display_frame.shape[:2]
                
                # Draw center reference line
                cv2.line(display_frame, (width // 2, 0), (width // 2, height), 
                        (0, 255, 0), 2)
                
                # Draw detected line position if found
                if self.line_detected:
                    line_x = int(width // 2 + self.line_position * width // 2)
                    cv2.line(display_frame, (line_x, 0), (line_x, height), 
                            (0, 0, 255), 3)
                    
                    # Add text overlay
                    text = f"Position: {self.line_position:.2f}"
                    cv2.putText(display_frame, text, (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, "No line detected", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                cv2.imshow("Original", display_frame)
            
            if self.processed_frame is not None:
                cv2.imshow("Processed", self.processed_frame)
        
        cv2.waitKey(1)
    
    def start(self):
        """Start the line tracking thread"""
        if not self.running:
            self.running = True
            self.tracking_thread = Thread(target=self.tracking_loop)
            self.tracking_thread.daemon = True
            self.tracking_thread.start()
            logger.info("Line follower started")
    
    def stop(self):
        """Stop the line tracking thread"""
        self.running = False
        if hasattr(self, 'tracking_thread'):
            self.tracking_thread.join(timeout=2.0)
        if self.debug:
            cv2.destroyAllWindows()
        logger.info("Line follower stopped")
    
    def get_motor_speeds(self, base_speed=50, turn_factor=30):
        """
        Calculate motor speeds based on line position
        
        Args:
            base_speed: Base speed for motors (0-100)
            turn_factor: How aggressively to turn (0-100)
            
        Returns:
            Tuple of (left_speed, right_speed) from -100 to 100
        """
        if not self.line_detected:
            # No line detected - could stop or search
            return (0, 0)
        
        # Calculate differential steering
        # If line is to the right (positive), turn right (slow right motor)
        # If line is to the left (negative), turn left (slow left motor)
        turn_adjustment = self.line_position * turn_factor
        
        left_speed = base_speed + turn_adjustment
        right_speed = base_speed - turn_adjustment
        
        # Clamp speeds to valid range
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        return (left_speed, right_speed)
    
    def __del__(self):
        """Cleanup when object is destroyed"""
        self.stop()
        if hasattr(self, 'camera'):
            self.camera.release()


# Example usage and testing
if __name__ == "__main__":
    # Create line follower with debug visualization
    follower = LineFollower(camera_index=0, debug=True)
    
    print("Starting line follower test...")
    print("Press 'q' in any window to quit")
    print("Press 's' to toggle motor speed display")
    
    follower.start()
    
    show_speeds = True
    
    try:
        while True:
            # Get current motor speeds
            left, right = follower.get_motor_speeds(base_speed=50, turn_factor=30)
            
            if show_speeds:
                if follower.line_detected:
                    print(f"Line position: {follower.line_position:+.2f} | "
                          f"Motors L:{left:+3.0f} R:{right:+3.0f}", end='\r')
                else:
                    print("No line detected - searching...        ", end='\r')
            
            # Check for key press
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                show_speeds = not show_speeds
                if not show_speeds:
                    print("\n")
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nStopping line follower...")
    finally:
        follower.stop()
        cv2.destroyAllWindows()
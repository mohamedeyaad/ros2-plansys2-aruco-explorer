#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess
import time
import threading  # <--- NEW: Required for non-blocking execution

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        self.id_map = {} 
        self.current_target_wp = ""
        
        # Subscribe to the ID reports
        self.create_subscription(Int32, '/found_id', self.id_callback, 10)
        
        self.get_logger().info("Mission Controller Initialized.")
        
        # --- THE FIX: Run mission in a background thread ---
        # This allows rclpy.spin() (in the main block) to keep working
        # and processing callbacks while this thread sleeps.
        self.mission_thread = threading.Thread(target=self.run_mission_logic)
        self.mission_thread.start()

    def id_callback(self, msg):
        # Only record if we are looking for something
        if self.current_target_wp:
            # Avoid duplicates
            if msg.data not in self.id_map:
                self.id_map[msg.data] = self.current_target_wp
                self.get_logger().info(f"--> RECORDED: Marker ID {msg.data} found at {self.current_target_wp}")

    def send_plansys2_cmd(self, cmd_string):
        self.get_logger().info(f"Sending PDDL: {cmd_string}")
        p = subprocess.Popen(
            ["ros2", "run", "plansys2_terminal", "plansys2_terminal"],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True
        )
        full_input = f"{cmd_string}\nquit\n"
        p.communicate(input=full_input)
        time.sleep(0.5)

    def wait_for_predicate(self, predicate_str, timeout=60):
        """
        Blocks execution until the specific predicate becomes true in PlanSys2.
        Example: self.wait_for_predicate("(surveyed wp1)")
        """
        self.get_logger().info(f"Waiting for predicate: {predicate_str}...")
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            # 1. Run the 'get problem predicate' command
            # We use a specific PDDL syntax to check if it exists
            # Note: plansys2_terminal returns *all* predicates details if we query it.
            # A simpler way is to grep the output of 'get problem predicates'
            
            p = subprocess.Popen(
                ["ros2", "run", "plansys2_terminal", "plansys2_terminal"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # We ask for the list of ALL current predicates
            cmd = "get problem predicates\nquit\n"
            stdout, _ = p.communicate(input=cmd)

            # 2. Check if our target predicate is in the output
            # stdout will look like:
            # (connected wp1 wp2)
            # (at r wp1)
            # (surveyed wp1)  <-- We look for this!
            
            if predicate_str in stdout:
                self.get_logger().info(f"SUCCESS: {predicate_str} achieved!")
                return True
            
            # 3. Wait a bit before checking again to save CPU
            time.sleep(1.0)

        self.get_logger().error(f"TIMEOUT: Predicate {predicate_str} never became true.")
        return False
    
    def run_mission_logic(self):
        # Allow some startup time for PlanSys2 to process the problem file
        time.sleep(3.0) 

        # =========================================
        # 0. SETUP: SKIPPED! 
        # (Loaded automatically by launch file)
        # =========================================
        self.get_logger().info("Map and Robot State loaded from PDDL file.")

        # =========================================
        # 1. PHASE 1: SURVEY (Dynamic Goal Setting)
        # =========================================
        waypoints = ['wp1', 'wp2', 'wp3', 'wp4']
        print("\n--- STARTING PHASE 1: SURVEY ---")
        
        for wp in waypoints:
            self.current_target_wp = wp
            print(f"Surveying {wp}...")
            
            # OVERWRITE the dummy goal with your actual Phase 1 goal
            self.send_plansys2_cmd(f"set goal (and (surveyed {wp}))")
            
            self.send_plansys2_cmd("run")
            
            # Wait loop...
            print("Waiting for robot to finish survey...")
            # 3. SMART WAIT (Replaces time.sleep)
            # We wait specifically for the result of this action
            success = self.wait_for_predicate(f"(surveyed {wp})")
        # =========================================
        # 2. PHASE 2: EXECUTION (Lowest ID First)
        # =========================================
        print("\n--- STARTING PHASE 2: EXECUTION ---")
        
        sorted_ids = sorted(self.id_map.keys())
        
        for marker_id in sorted_ids:
            target_wp = self.id_map[marker_id]
            print(f"Targeting ID {marker_id} at {target_wp}")

            # OVERWRITE goal again for Phase 2
            self.send_plansys2_cmd(f"set goal (and (picture_taken {target_wp}))")
            
            self.send_plansys2_cmd("run")
            
            print("Waiting for robot to take picture...")
            self.wait_for_predicate(f"(picture_taken {target_wp})")            
            print(f">>> ID {marker_id} COMPLETED.\n")

        print("MISSION ACCOMPLISHED.")
        # Optional: Kill self after done
        # rclpy.shutdown() 

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    
    # The main thread just spins to handle callbacks (ID reports)
    # The actual mission runs in the background thread we started in __init__
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
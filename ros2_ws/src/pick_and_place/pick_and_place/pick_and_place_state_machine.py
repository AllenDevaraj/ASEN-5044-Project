#!/usr/bin/env python3

"""
A state machine for pick-and-place tasks.

The robot starts in the home position. If objects are detected on the workbench,
it randomly selects one. Once an object is selected, the robot picks and places it to 
the same color bin. Then, returns to the home position. If no objects are 
detected on the workbench, it stops.

Author: Elena Oikonomou (adapted to ROS2)
Date:   Fall 2023 / 2025
"""

import rclpy
from rclpy.node import Node
import time

try:
    from statemachine import State, StateMachine
    STATEMACHINE_AVAILABLE = True
except ImportError:
    STATEMACHINE_AVAILABLE = False
    print("Warning: python-statemachine not available, using simplified state machine")

from pick_and_place.controller import Controller


if STATEMACHINE_AVAILABLE:
    class PickAndPlaceStateMachine(StateMachine):
        # States
        home = State("Home", initial=True)
        selecting_object = State("SelectingObject")
        picking_and_placing = State("PickingAndPlacing")
        done = State("Done", final=True)
        
        # Events & Transitions
        select_object = home.to(selecting_object, cond="are_objects_detected") | home.to(done, unless="are_objects_detected")
        pick_object = selecting_object.to(picking_and_placing, cond="object_selected")
        get_ready = picking_and_placing.to(home, cond="object_placed")
        
        def __init__(self, controller):
            self.controller = controller
            self.object_selected = False
            self.object_placed = False
            self.currently_selected_object = None
            
            print('\n' + 80*'=')
            controller.get_logger().info("*** Pick-and-Place Begins! ***")
            print(80*'=')
            
            super().__init__()
        
        def are_objects_detected(self) -> bool:
            """Guard to transition to 'selecting_object' state when in 'home' state."""
            return self.controller.are_objects_on_workbench()
        
        def on_enter_home(self) -> None:
            """Moves robot to home position and triggers the 'select_object' event."""
            self.controller.get_logger().info("Moving to home position..")
            self.controller.move_to_neutral()
            self.object_selected = False
            self.object_placed = False
            time.sleep(0.2)
            
            self.send("select_object")
        
        def on_enter_selecting_object(self) -> None:
            """Selects an object to pick from the workbench and triggers the 'pick_object' event."""
            self.controller.get_logger().info("Selecting object to pick..")
            self.currently_selected_object = self.controller.select_random_object()
            self.object_selected = True
            self.controller.get_logger().info("Object selected.")
            
            self.send("pick_object")
        
        def on_enter_picking_and_placing(self) -> None:
            """Picks and places the selected object to its bin and triggers the 'get_ready' event."""
            self.controller.get_logger().info("Starting pick & place operation of selected object..")
            self.controller.move_object(self.currently_selected_object)
            self.object_placed = True
            self.controller.get_logger().info("Object placed in its bin.")
            
            self.send("get_ready")
        
        def on_enter_done(self) -> None:
            """Reports that the robot has finished its pick-and-place tasks."""
            print('\n' + 60*'=')
            self.controller.get_logger().info("*** Mission Complete! *** \nAll objects have been placed in their bins.")
            print(60*'=')
else:
    # Simplified state machine without python-statemachine library
    class PickAndPlaceStateMachine:
        def __init__(self, controller):
            self.controller = controller
            self.state = "home"
            self.currently_selected_object = None
            
            print('\n' + 80*'=')
            controller.get_logger().info("*** Pick-and-Place Begins! ***")
            print(80*'=')
        
        def run(self):
            """Run the state machine loop."""
            while rclpy.ok():
                if self.state == "home":
                    self.controller.get_logger().info("Moving to home position..")
                    self.controller.move_to_neutral()
                    time.sleep(0.2)
                    
                    if self.controller.are_objects_on_workbench():
                        self.state = "selecting_object"
                    else:
                        self.state = "done"
                
                elif self.state == "selecting_object":
                    self.controller.get_logger().info("Selecting object to pick..")
                    self.currently_selected_object = self.controller.select_random_object()
                    self.controller.get_logger().info("Object selected.")
                    self.state = "picking_and_placing"
                
                elif self.state == "picking_and_placing":
                    self.controller.get_logger().info("Starting pick & place operation of selected object..")
                    self.controller.move_object(self.currently_selected_object)
                    self.controller.get_logger().info("Object placed in its bin.")
                    self.state = "home"
                
                elif self.state == "done":
                    print('\n' + 60*'=')
                    self.controller.get_logger().info("*** Mission Complete! *** \nAll objects have been placed in their bins.")
                    print(60*'=')
                    break
                
                rclpy.spin_once(self.controller, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    
    # Give time for initialization
    time.sleep(2.0)
    
    try:
        state_machine = PickAndPlaceStateMachine(controller)
        if not STATEMACHINE_AVAILABLE:
            state_machine.run()
        else:
            rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


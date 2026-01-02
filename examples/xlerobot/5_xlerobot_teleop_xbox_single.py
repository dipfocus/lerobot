# To Run on the host
'''
PYTHONPATH=src python -m lerobot.robots.xlerobot.xlerobot_host --robot.id=my_xlerobot
'''

# To Run the teleop:
'''
PYTHONPATH=src python -m examples.xlerobot.teleoperate_XBOX
'''

"""
This script provides teleoperation for the right arm and base of the XLerobot using an Xbox controller.
It is intended to be run on a Jetson or similar Linux-based system connected directly to the robot.

- Right Stick: Controls the arm's end-effector position (X/Y).
- Right Stick Press + Right Stick: Controls shoulder pan.
- RB + Right Stick: Controls end-effector pitch and wrist roll.
- Right Trigger: Controls the gripper.
- D-Pad: Controls the base movement (forward, backward, rotate).
- Y/A Buttons: Increase/decrease base speed.
- Menu Button: Resets the arm to its zero position.
"""

import time
import numpy as np
import math
import pygame

from typing import Final, Dict
from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from lerobot.model.SO101Robot import SO101Kinematics

AXIS_RX: Final = 2   # right stick X
AXIS_RY: Final = 3   # right stick Y
AXIS_RT: Final = 4   # right trigger


BTN_A: Final         = 0
BTN_B: Final         = 1
BTN_X: Final         = 3
BTN_Y: Final         = 4
BTN_RB: Final        = 7
BTN_VIEW: Final      = 10    # 视图键
BTN_MENU: Final      = 11
BTN_START: Final     = 12
BTN_R_STICK: Final   = 14
BTN_SHARE: Final     = 15

# Keymaps (semantic action: controller mapping) - Intuitive human control
RIGHT_KEYMAP = {
    # Right stick controls right arm XY (when not pressed)
    'x+': 'right_stick_up', 'x-': 'right_stick_down',
    'y+': 'right_stick_right', 'y-': 'right_stick_left',
    # Right stick pressed controls right arm shoulder_pan
    'shoulder_pan+': 'right_stick_pressed_right', 'shoulder_pan-': 'right_stick_pressed_left',
    # RB pressed controls right arm pitch and wrist_roll
    'pitch+': 'rb_up', 'pitch-': 'rb_down',
    'wrist_roll+': 'rb_right', 'wrist_roll-': 'rb_left',
    # Right trigger controls right gripper
    'gripper+': 'right_trigger',
}

# Base control keymap - Only forward/backward and rotate left/right
BASE_KEYMAP = {
    'forward': 'dpad_down', 'backward': 'dpad_up',
    'rotate_left': 'dpad_left', 'rotate_right': 'dpad_right',
}

# Global reset key for all components
RESET_KEY = 'back'

RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}

class SimpleTeleopArm:
    GRIPPER_CLOSED_POS: Final = 2
    GRIPPER_OPEN_POS: Final = 90

    def __init__(self, kinematics, joint_map, initial_obs, prefix="left", kp=1):
        self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix  # To distinguish left and right arm
        self.kp = kp
        # Initial joint positions
        self.joint_positions = {
            "shoulder_pan": initial_obs[f"{prefix}_arm_shoulder_pan.pos"],
            "shoulder_lift": initial_obs[f"{prefix}_arm_shoulder_lift.pos"],
            "elbow_flex": initial_obs[f"{prefix}_arm_elbow_flex.pos"],
            "wrist_flex": initial_obs[f"{prefix}_arm_wrist_flex.pos"],
            "wrist_roll": initial_obs[f"{prefix}_arm_wrist_roll.pos"],
            "gripper": initial_obs[f"{prefix}_arm_gripper.pos"],
        }
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        # Set the degree step and xy step
        self.degree_step = 2
        self.xy_step = 0.005
        # Set target positions to zero for P control
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }

    def move_to_zero_position(self, robot):
        print(f"[{self.prefix}] Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = self.zero_pos.copy()  # Use copy to avoid reference issues
        
        # Reset kinematic variables to their initial state
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        
        # Don't let handle_keys recalculate wrist_flex - set it explicitly
        self.target_positions["wrist_flex"] = 0.0
        
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_keys(self, key_state):
        # Joint increments
        if key_state.get('shoulder_pan+'):
            self.target_positions["shoulder_pan"] += self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('shoulder_pan-'):
            self.target_positions["shoulder_pan"] -= self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('wrist_roll+'):
            self.target_positions["wrist_roll"] += self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        if key_state.get('wrist_roll-'):
            self.target_positions["wrist_roll"] -= self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        
        # Gripper control with auto-close functionality
        if key_state.get('gripper+'):
            # Trigger pressed - open gripper (0.1)
            self.target_positions["gripper"] = self.GRIPPER_CLOSED_POS
            print(f"[{self.prefix}] gripper: CLOSED")
        else:
            self.target_positions["gripper"] = self.GRIPPER_OPEN_POS
            print(f"[{self.prefix}] gripper: auto-opening to {self.GRIPPER_OPEN_POS:.1f}")
        
        if key_state.get('pitch+'):
            self.pitch += self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")
        if key_state.get('pitch-'):
            self.pitch -= self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")

        # XY plane (IK)
        moved = False
        if key_state.get('x+'):
            self.current_x += self.xy_step
            moved = True
            print(f"[{self.prefix}] x+: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('x-'):
            self.current_x -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x-: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('y+'):
            self.current_y += self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y+: {self.current_y:.4f}")
        if key_state.get('y-'):
            self.current_y -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y-: {self.current_y:.4f}")
        if moved:
            joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            self.target_positions["shoulder_lift"] = joint2
            self.target_positions["elbow_flex"] = joint3
            print(f"[{self.prefix}] shoulder_lift: {joint2}, elbow_flex: {joint3}")

        # Wrist flex is always coupled to pitch and the other two
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            -self.target_positions["elbow_flex"]
            + self.pitch
        )
        # print(f"[{self.prefix}] wrist_flex: {self.target_positions['wrist_flex']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action
    

# --- XBOX Controller Mapping ---
def get_xbox_key_state(joystick, keymap):
    """
    Map XBOX controller state to semantic action booleans using the provided keymap.
    """
    # Read axes, buttons, hats
    axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    hats = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)
    
    # Get stick pressed states
    right_stick_pressed = bool(buttons[BTN_R_STICK]) if len(buttons) > 14 else False
    rb_pressed = bool(buttons[BTN_RB]) if len(buttons) > 14 else False

    # Map controller state to semantic actions
    state = {}
    for action, control in keymap.items():
        if control == 'right_trigger':
            state[action] = axes[AXIS_RT] > 0.5 if len(axes) > 5 else False
        elif control == 'a':
            state[action] = bool(buttons[BTN_A])
        elif control == 'b':
            state[action] = bool(buttons[BTN_B])
        elif control == 'x':
            state[action] = bool(buttons[BTN_X])
        elif control == 'y':
            state[action] = bool(buttons[BTN_Y])
        elif control == 'back':
            state[action] = bool(buttons[BTN_MENU])
        elif control == 'dpad_up':
            state[action] = hats[1] == 1
        elif control == 'dpad_down':
            state[action] = hats[1] == -1
        elif control == 'dpad_left':
            state[action] = hats[0] == -1
        elif control == 'dpad_right':
            state[action] = hats[0] == 1
        # Right stick controls (when not pressed) - Fixed axis mapping
        elif control == 'right_stick_up': # x+
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (axes[BTN_X] < -0.5) if len(axes) > 4 else False
        elif control == 'right_stick_down': # x-
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (axes[BTN_X] > 0.5) if len(axes) > 4 else False
        elif control == 'right_stick_left':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (axes[BTN_Y] < -0.5) if len(axes) > 3 else False
        elif control == 'right_stick_right':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (axes[BTN_Y] > 0.5) if len(axes) > 3 else False
        # Right stick pressed controls - Fixed axis mapping
        elif control == 'right_stick_pressed_right':
            state[action] = right_stick_pressed and (not rb_pressed) and (axes[BTN_Y] > 0.5) if len(axes) > 3 else False
        elif control == 'right_stick_pressed_left':
            state[action] = right_stick_pressed and (not rb_pressed) and (axes[BTN_Y] < -0.5) if len(axes) > 3 else False
        # RB pressed controls (only when stick is moved)
        elif control == 'rb_up':
            state[action] = rb_pressed and (abs(axes[BTN_X]) > 0.5) and (axes[BTN_X] < -0.5) if len(axes) > 4 else False
        elif control == 'rb_down':
            state[action] = rb_pressed and (abs(axes[BTN_X]) > 0.5) and (axes[BTN_X] > 0.5) if len(axes) > 4 else False
        elif control == 'rb_right':
            state[action] = rb_pressed and (abs(axes[BTN_Y]) > 0.5) and (axes[BTN_Y] > 0.5) if len(axes) > 3 else False
        elif control == 'rb_left':
            state[action] = rb_pressed and (abs(axes[BTN_Y]) > 0.5) and (axes[BTN_Y] < -0.5) if len(axes) > 3 else False
        else:
            state[action] = False
    return state

def get_base_action(joystick, robot):
    """
    Get base action from XBOX controller input - simplified to only forward/backward and rotate.
    """
    # Read controller state
    hats = joystick.get_hat(0) if joystick.get_numhats() > 0 else (0, 0)
    
    # Get pressed keys for base control
    pressed_keys = set()
    
    # Map controller inputs to keyboard-like keys for base control
    if hats[1] == 1:   # D-pad up
        pressed_keys.add('k')  # Forward
    if hats[1] == -1:  # D-pad down
        pressed_keys.add('i')  # Backward
    if hats[0] == -1:  # D-pad left
        pressed_keys.add('u')  # Rotate left
    if hats[0] == 1:   # D-pad right
        pressed_keys.add('o')  # Rotate right
    
    # Convert to numpy array and get base action
    keyboard_keys = np.array(list(pressed_keys))
    base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}
    
    return base_action

def get_base_speed_control(joystick):
    """
    Get base speed control from XBOX controller - LB for speed decrease, RB for speed increase.
    Returns speed multiplier (1.0, 2.0, or 3.0) and prints current speed level.
    """
    # Read controller state
    buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
    
    # Get LB and RB states
    lb_pressed = bool(buttons[BTN_LB]) if len(buttons) > 4 else False
    rb_pressed = bool(buttons[BTN_RB]) if len(buttons) > 5 else False
    
    # Get current speed level from global variable
    global current_base_speed_level
    if 'current_base_speed_level' not in globals():
        current_base_speed_level = 1  # Default speed level
    
    # Speed control logic
    if lb_pressed and not rb_pressed:
        # LB pressed alone - decrease speed
        if current_base_speed_level > 1:
            current_base_speed_level -= 1
            print(f"[BASE] Speed decreased to level {current_base_speed_level}")
    elif rb_pressed and not lb_pressed:
        # RB pressed alone - increase speed
        if current_base_speed_level < 3:
            current_base_speed_level += 1
            print(f"[BASE] Speed increased to level {current_base_speed_level}")
    
    # Map speed level to multiplier
    speed_multiplier = float(current_base_speed_level)
    
    return speed_multiplier


def main():
    FPS = 30
    robot_name = "xlerobot_teleop_xbox"
    port1: str = "/dev/ttyACM0"  # port to connect to the bus (so101 + head camera)
    port2: str = "/dev/ttyACM1"  # port to connect to the bus (same as lekiwi setup)
    robot_config = XLerobotConfig(id=robot_name, port1=port1, port2=port2)
    robot = XLerobot(robot_config)
    try:
        robot.connect()
        print(f"[MAIN] Successfully connected to robot")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(robot_config)
        print(robot)
        return

    init_rerun(session_name=robot_name)

    # Init XBOX controller
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No XBOX controller detected!")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"[MAIN] Using controller: {joystick.get_name()}")

    # Init the arm instance
    obs = robot.get_observation()
    kin_right = SO101Kinematics()
    right_arm = SimpleTeleopArm(kin_right, RIGHT_JOINT_MAP, obs, prefix="right")

    # Move right arm to zero position at start
    right_arm.move_to_zero_position(robot)

    # Base speed control state
    base_speed_level = 1
    last_speed_buttons_state = {"up": False, "down": False}

    try:
        while True:
            pygame.event.pump()
            right_key_state = get_xbox_key_state(joystick, RIGHT_KEYMAP)
            
            # Check for global reset (back button)
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            global_reset = bool(buttons[BTN_MENU]) if len(buttons) > 11 else False
            
            # Handle global reset
            if global_reset:
                print("[MAIN] Global reset triggered!")
                right_arm.move_to_zero_position(robot)
                continue

            # Handle right arm
            right_arm.handle_keys(right_key_state)

            right_action = right_arm.p_control_action(robot)

            # Get base action and speed control from controller
            base_action = get_base_action(joystick, robot)
            speed_multiplier = get_base_speed_control(joystick)
            
            # Apply speed multiplier to base actions if they exist
            if base_action:
                for key in base_action:
                    if 'vel' in key or 'velocity' in key:  # Apply to velocity commands
                        base_action[key] *= speed_multiplier

            # Merge all actions
            action = {**right_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            log_rerun_data(obs, action)
    finally:
        robot.disconnect()
        print("Teleoperation ended.")

if __name__ == "__main__":
    main()

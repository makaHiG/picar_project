import math
import time
from enum import Enum
from typing import Tuple, List, Dict
import sys
import os

# Add parent directory to path to import picar modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'SunFounder_PiCar'))

# Direction enum: 8 possible directions in multiples of 45 degrees
class Direction(Enum):
    EAST = 0          # → (0°)
    NORTHEAST = 45    # ↗ (45°)
    NORTH = 90        # ↑ (90°)
    NORTHWEST = 135   # ↖ (135°)
    WEST = 180        # ← (180°)
    SOUTHWEST = 225   # ↙ (225°)
    SOUTH = 270       # ↓ (270°)
    SOUTHEAST = 315   # ↘ (315°)
    
    def turn_left(self):
        """Turn 45 degrees counter-clockwise"""
        angles = [d.value for d in Direction]
        current_idx = angles.index(self.value)
        next_idx = (current_idx + 1) % 8
        return Direction(angles[next_idx])
    
    def turn_right(self):
        """Turn 45 degrees clockwise"""
        angles = [d.value for d in Direction]
        current_idx = angles.index(self.value)
        next_idx = (current_idx - 1) % 8
        return Direction(angles[next_idx])

class GridRobot:
    def __init__(self, cell_size: float = 1.0):
        """
        Initialize robot on an infinite grid.
        
        Args:
            cell_size: physical size of each grid cell
        """
        self.cell_size = cell_size
        self.diagonal_step = cell_size * math.sqrt(2)
        
        # Current position in continuous coordinates
        self.x = 0.0
        self.y = 0.0
        self.direction = Direction.EAST
        
        # Goal tracking
        self.goal_x: float = None  # Exact goal location (identified by sensor later)
        self.goal_y: float = None
        self.initial_goal_direction: Direction = None  # Initial direction towards goal
        
        # Obstacles detected by sensor: dict of (grid_x, grid_y) -> detection_radius
        # A point is blocked if its distance to obstacle <= detection_radius
        # Using sparse dictionary to handle infinite grid
        self.obstacles: Dict[Tuple[int, int], float] = {}
        
    def get_step_offset(self, step_type: str = "normal") -> Tuple[float, float]:
        """
        Get x, y offset for one step in current direction.
        
        Args:
            step_type: "normal" (cell size) or "diagonal" (cell diagonal)
        
        Returns:
            (dx, dy) offset
        """
        step_distance = self.diagonal_step if step_type == "diagonal" else self.cell_size
        angle_rad = math.radians(self.direction.value)
        dx = step_distance * math.cos(angle_rad)
        dy = step_distance * math.sin(angle_rad)
        return dx, dy
    
    def can_move_to(self, x: float, y: float) -> bool:
        """
        Check if position is safe (not blocked by obstacles).
        On infinite grid, no bounds checking - only check distance to obstacles.
        
        Args:
            x, y: continuous coordinates
        
        Returns:
            True if position is safe, False if blocked by obstacle
        """
        # Convert to grid cell
        grid_x = round(x / self.cell_size)
        grid_y = round(y / self.cell_size)
        
        # Check distance to all obstacles
        for (obs_x, obs_y), detection_radius in self.obstacles.items():
            distance = math.sqrt((grid_x - obs_x) ** 2 + (grid_y - obs_y) ** 2)
            if distance <= detection_radius:
                return False
        
        return True
    
    def step(self, step_type: str = "normal") -> bool:
        """
        Move one step in current direction.
        
        Args:
            step_type: "normal" or "diagonal"
        
        Returns:
            True if move successful, False if blocked by obstacle or boundary
        """
        dx, dy = self.get_step_offset(step_type)
        new_x = self.x + dx
        new_y = self.y + dy
        
        if self.can_move_to(new_x, new_y):
            self.x = new_x
            self.y = new_y
            return True
        else:
            print(f"Cannot move to ({new_x:.2f}, {new_y:.2f}) - obstacle or boundary")
            return False
    
    def turn(self, direction: str) -> None:
        """
        Turn in-place (left or right).
        
        Args:
            direction: "left" or "right" (45 degree increments)
        """
        if direction.lower() == "left":
            self.direction = self.direction.turn_left()
        elif direction.lower() == "right":
            self.direction = self.direction.turn_right()
    
    def full_turn_at_gridpoint(self, pause_duration: float = 0.5) -> None:
        """
        Perform a complete 360-degree turn at current grid point.
        Turn in 45-degree chunks (8 total) with pause after each chunk.
        
        Args:
            pause_duration: time in seconds to pause after each 45-degree turn
        """
        gx, gy = self.get_grid_position()
        print(f"\n--- Full turn at grid point ({gx}, {gy}) ---")
        
        starting_direction = self.direction
        
        # Perform 8 x 45-degree turns (full 360° rotation)
        for turn_num in range(8):
            self.turn("left")
            print(f"  Turn {turn_num + 1}/8: Now facing {self.direction.name}")
            time.sleep(pause_duration)
        
        # Should return to starting direction
        print(f"Full turn complete. Back to {self.direction.name}")
    
    def add_obstacle(self, grid_x: int, grid_y: int, detection_radius: float = 1.0) -> None:
        """
        Sensor feedback: add detected obstacle with detection radius.
        Works on infinite grid - can add obstacles anywhere.
        
        Args:
            grid_x, grid_y: grid coordinates of obstacle
            detection_radius: how far from this point blocks movement
        """
        self.obstacles[(grid_x, grid_y)] = detection_radius
        print(f"Obstacle detected at grid ({grid_x}, {grid_y}) with radius {detection_radius}")
    
    def update_obstacle_radius(self, grid_x: int, grid_y: int, new_radius: float) -> None:
        """
        Update detection radius of an existing obstacle.
        Call this when viewing obstacle from different angle gives new measurement.
        
        Args:
            grid_x, grid_y: grid coordinates of obstacle
            new_radius: new detection radius
        """
        if (grid_x, grid_y) in self.obstacles:
            old_radius = self.obstacles[(grid_x, grid_y)]
            self.obstacles[(grid_x, grid_y)] = new_radius
            print(f"Obstacle at ({grid_x}, {grid_y}) radius updated: {old_radius} → {new_radius}")
        else:
            print(f"No obstacle found at ({grid_x}, {grid_y})")
    
    def remove_obstacle(self, grid_x: int, grid_y: int) -> None:
        """Sensor feedback: remove obstacle (no longer detected)"""
        if (grid_x, grid_y) in self.obstacles:
            self.obstacles.pop((grid_x, grid_y))
            print(f"Obstacle cleared at grid ({grid_x}, {grid_y})")
    
    def get_obstacles(self) -> Dict[Tuple[int, int], float]:
        """Get all detected obstacles and their detection radii"""
        return self.obstacles.copy()
    
    def set_goal_direction(self, direction: Direction) -> None:
        """
        Set initial direction towards goal (before exact goal location is known).
        Used when robot knows direction to goal but not exact distance/location.
        
        Args:
            direction: Direction toward the goal
        """
        self.initial_goal_direction = direction
        print(f"Goal direction set to: {direction.name}")
    
    def set_goal_location(self, goal_x: float, goal_y: float) -> None:
        """
        Sensor identifies exact goal location.
        Once goal is identified, robot can navigate to it.
        This method can be called at any time while the program is running.

        Args:
            goal_x, goal_y: continuous coordinates of goal
        """
        self.goal_x = goal_x
        self.goal_y = goal_y
        print(f"Goal location identified at: ({goal_x:.2f}, {goal_y:.2f})")

    def point_towards_goal(self) -> bool:
        """
        Adjust the robot's heading so that it faces the current goal.
        This is useful when the goal has just been added or changed while the
        robot is already running.

        Returns:
            True if the robot now has a goal and its direction was updated,
            False otherwise.
        """
        if not self.has_goal():
            print("No goal identified; cannot point towards it.")
            return False

        goal_angle = self.direction_to_goal()
        # choose nearest enum direction
        def angle_diff(a, b):
            d = abs(a - b) % 360
            return min(d, 360 - d)

        best = min(Direction, key=lambda d: angle_diff(d.value, goal_angle))
        if best != self.direction:
            self.direction = best
            print(f"Now facing {self.direction.name} (goal at {goal_angle:.0f}°)")
        else:
            print(f"Already facing {self.direction.name} towards goal")
        return True
    
    def has_goal(self) -> bool:
        """Check if goal location has been identified by sensor"""
        return self.goal_x is not None and self.goal_y is not None
    
    def distance_to_goal(self) -> float:
        """
        Calculate distance to goal (if goal is identified).
        
        Returns:
            Distance in continuous coordinates, or None if goal not identified
        """
        if not self.has_goal():
            return None
        return math.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)
    
    def direction_to_goal(self) -> float:
        """
        Calculate direction angle to goal.
        
        Returns:
            Angle in degrees (0-360), or None if goal not identified
        """
        if not self.has_goal():
            return None
        angle_rad = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        angle_deg = math.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg += 360
        return angle_deg
    
    def goal_in_line_of_sight(self) -> bool:
        """
        Check if goal is reachable without obstacles blocking the path.
        
        Returns:
            True if path to goal is clear (or goal is close)
        """
        if not self.has_goal():
            return False
        
        # Simple check: sample points along the line to goal
        dist = self.distance_to_goal()
        if dist < self.cell_size:
            return True
        
        # Sample path every cell
        steps = int(dist / self.cell_size)
        for step in range(steps + 1):
            t = step / steps if steps > 0 else 0
            check_x = self.x + (self.goal_x - self.x) * t
            check_y = self.y + (self.goal_y - self.y) * t
            if not self.can_move_to(check_x, check_y):
                return False
        
        return True
    
    def is_goal_reached(self, tolerance: float = None) -> bool:
        """
        Check if robot has reached the goal.
        
        Args:
            tolerance: distance threshold to consider goal reached (default: cell_size)
        
        Returns:
            True if at goal, False otherwise
        """
        if not self.has_goal():
            return False
        
        if tolerance is None:
            tolerance = self.cell_size
        
        return self.distance_to_goal() <= tolerance
    
    def move_towards_goal(self, step_type: str = "normal") -> bool:
        """
        Move one step towards identified goal (if reachable in current direction).
        
        Args:
            step_type: "normal" or "diagonal"
        
        Returns:
            True if moved, False if blocked or no goal identified
        """
        if not self.has_goal():
            print("No goal identified yet")
            return False
        
        goal_angle = self.direction_to_goal()
        current_angle = self.direction.value
        
        # Check if goal is roughly in the direction we're facing (within 90 degrees)
        angle_diff = abs(goal_angle - current_angle)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        if angle_diff > 90:
            print(f"Goal is not in current direction (goal at {goal_angle:.0f}°, facing {current_angle}°)")
            return False
        
        return self.step(step_type)
    
    def get_position(self) -> Tuple[float, float]:
        """Get current position in continuous coordinates"""
        return self.x, self.y
    
    def get_grid_position(self) -> Tuple[int, int]:
        """Get current position in grid cell coordinates"""
        return round(self.x / self.cell_size), round(self.y / self.cell_size)
    
    def get_direction(self) -> str:
        """Get current direction as string"""
        return self.direction.name
    
    def print_state(self) -> None:
        """Print robot state"""
        gx, gy = self.get_grid_position()
        state = f"Position: ({self.x:.2f}, {self.y:.2f}) | Grid: ({gx}, {gy}) | Direction: {self.direction.name}"
        
        if self.has_goal():
            dist = self.distance_to_goal()
            goal_angle = self.direction_to_goal()
            state += f" | Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) | Distance: {dist:.2f} | Angle: {goal_angle:.0f}°"
        elif self.initial_goal_direction:
            state += f" | Goal direction: {self.initial_goal_direction.name}"
        
        print(state)


class HardwareRobot(GridRobot):
    """
    Hardware-integrated robot that combines GridRobot navigation with actual back_wheels motor control.
    Commands the physical robot while tracking position and navigation state.
    """
    
    def __init__(self, back_wheels=None, cell_size: float = 1.0, speed: int = 50, 
                 step_delay: float = 0.5, debug: bool = False):
        """
        Initialize hardware robot with back wheels controller.
        
        Args:
            back_wheels: Back_Wheels instance to control physical motors
                        If None, will try to import and create one
            cell_size: physical size of each grid cell (in cm or other units)
            speed: motor speed (0-100)
            step_delay: time in seconds to run motors for one step
            debug: enable debug output
        """
        super().__init__(cell_size=cell_size)
        
        # Try to import back_wheels if not provided
        if back_wheels is None:
            try:
                from picar.back_wheels import Back_Wheels
                self.back_wheels = Back_Wheels(debug=debug)
            except ImportError as e:
                print(f"Warning: Could not import Back_Wheels: {e}")
                print("Hardware robot created but motors will not work")
                self.back_wheels = None
        else:
            self.back_wheels = back_wheels
        
        self.speed = speed
        self.step_delay = step_delay
        self.debug = debug

        # placeholders for physical motion control
        # distance the robot travels for one "step" command (same units as cell_size)
        self.step_size = cell_size
        # degrees the robot rotates for one left/right command
        self.turn_angle = 45.0

        if self.debug:
            print(f"[DEBUG] step_size={self.step_size}, turn_angle={self.turn_angle}")
    
    def _execute_motor_command(self, command: str) -> None:
        """
        Execute a motor command on the physical robot.
        
        Args:
            command: "forward", "backward", "left", "right", or "stop"
        """
        if self.back_wheels is None:
            if self.debug:
                print(f"[DEBUG] Motor command (no hardware): {command}")
            return
        
        try:
            self.back_wheels.speed = self.speed
            
            if command == "forward":
                self.back_wheels.forward()
            elif command == "backward":
                self.back_wheels.backward()
            elif command == "left":
                # spin in place left
                self.back_wheels.left()
            elif command == "right":
                # spin in place right
                self.back_wheels.right()
            elif command == "stop":
                self.back_wheels.stop()
            else:
                print(f"Unknown motor command: {command}")
                return
            
            if self.debug:
                print(f"[DEBUG] Motor command executed: {command}")
        except Exception as e:
            print(f"Error executing motor command '{command}': {e}")
    
    def set_speed(self, speed: int) -> None:
        """
        Set motor speed (0-100).
        
        Args:
            speed: motor speed value
        """
        if 0 <= speed <= 100:
            self.speed = speed
            if self.back_wheels:
                self.back_wheels.speed = speed
            if self.debug:
                print(f"[DEBUG] Speed set to {speed}")
        else:
            raise ValueError("Speed must be between 0 and 100")

    def set_step_size(self, distance: float) -> None:
        """Configure how far one "step" command moves the robot."""
        self.step_size = distance
        if self.debug:
            print(f"[DEBUG] step_size set to {self.step_size}")

    def set_turn_angle(self, degrees: float) -> None:
        """Configure how many degrees the robot turns for each spin command."""
        self.turn_angle = degrees
        if self.debug:
            print(f"[DEBUG] turn_angle set to {self.turn_angle}°")
    
    def set_step_delay(self, delay: float) -> None:
        """
        Set the delay for executing one step.
        
        Args:
            delay: time in seconds for one step
        """
        self.step_delay = delay
        if self.debug:
            print(f"[DEBUG] Step delay set to {delay}s")
    
    def physical_forward(self) -> None:
        """Run forward on physical robot for one step duration"""
        self._execute_motor_command("forward")
        time.sleep(self.step_delay)
        self._execute_motor_command("stop")
        # Update position using configured step_size rather than grid cell
        angle_rad = math.radians(self.direction.value)
        self.x += self.step_size * math.cos(angle_rad)
        self.y += self.step_size * math.sin(angle_rad)
        if self.debug:
            print(f"[DEBUG] After forward: position ({self.x:.2f}, {self.y:.2f})")
    
    def physical_backward(self) -> None:
        """Run backward on physical robot for one step duration"""
        self._execute_motor_command("backward")
        time.sleep(self.step_delay)
        self._execute_motor_command("stop")
        # Update position using configured step_size
        angle_rad = math.radians(self.direction.value)
        self.x -= self.step_size * math.cos(angle_rad)
        self.y -= self.step_size * math.sin(angle_rad)
        if self.debug:
            print(f"[DEBUG] After backward: position ({self.x:.2f}, {self.y:.2f})")
    
    def physical_spin_left(self, pause_duration: float = 0.3) -> None:
        """Spin left (turning in place) and update direction by configured angle"""
        print(f"Spinning left... (was facing {self.direction.name})")
        self._execute_motor_command("left")
        time.sleep(pause_duration)
        self._execute_motor_command("stop")
        # update internal direction by turn_angle
        turns = int(self.turn_angle / 45)  # number of 45° increments
        for _ in range(turns):
            self.turn("left")
        print(f"Now facing {self.direction.name}")
    
    def physical_spin_right(self, pause_duration: float = 0.3) -> None:
        """Spin right (turning in place) and update direction by configured angle"""
        print(f"Spinning right... (was facing {self.direction.name})")
        self._execute_motor_command("right")
        time.sleep(pause_duration)
        self._execute_motor_command("stop")
        turns = int(self.turn_angle / 45)
        for _ in range(turns):
            self.turn("right")
        print(f"Now facing {self.direction.name}")
    
    def physical_step(self, step_type: str = "normal") -> bool:
        """
        Execute one step on physical robot.
        
        Args:
            step_type: "normal" or "diagonal"
        
        Returns:
            True if successful, False if blocked
        """
        dx, dy = self.get_step_offset(step_type)
        new_x = self.x + dx
        new_y = self.y + dy
        
        if not self.can_move_to(new_x, new_y):
            print(f"Cannot move to ({new_x:.2f}, {new_y:.2f}) - obstacle or boundary")
            return False
        
        self.physical_forward()
        return True
    
    def physical_turn_to_face(self, target_angle: float) -> None:
        """
        Turn to face a specific angle by executing left/right spins.
        
        Args:
            target_angle: target direction angle in degrees
        """
        current_angle = self.direction.value
        angle_diff = target_angle - current_angle
        
        # Normalize to -180 to 180
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        
        # Turn until facing target direction (within half a turn_angle)
        tolerance = self.turn_angle / 2
        while abs(angle_diff) > tolerance:
            if angle_diff > 0:
                self.physical_spin_left()
            else:
                self.physical_spin_right()
            
            # Recalculate
            angle_diff = target_angle - self.direction.value
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
    
    def point_towards_goal(self) -> bool:
        """Hardware override: physically rotate to face current goal."""
        if not self.has_goal():
            print("No goal identified; cannot point towards it.")
            return False
        goal_angle = self.direction_to_goal()
        self.physical_turn_to_face(goal_angle)
        return True

    def move_to_goal_physical(self) -> bool:
        """
        Navigate physically towards the identified goal.
        Repeatedly turns to face goal and steps forward until goal is reached.
        
        Returns:
            True if goal reached, False if unable to proceed (blocked)
        """
        if not self.has_goal():
            print("No goal identified. Cannot navigate.")
            return False
        
        print("\n=== Physical Navigation to Goal ===")
        self.print_state()
        
        step_count = 0
        max_steps = 50  # Prevent infinite loops
        
        while not self.is_goal_reached() and step_count < max_steps:
            step_count += 1
            dist = self.distance_to_goal()
            goal_angle = self.direction_to_goal()
            
            print(f"\n[Step {step_count}] Distance: {dist:.2f} | Goal angle: {goal_angle:.0f}°")
            
            # Turn to face goal
            self.physical_turn_to_face(goal_angle)
            
            # Try to move forward
            if not self.physical_step("normal"):
                print("❌ Blocked! Cannot reach goal.")
                return False
            
            self.print_state()
        
        if self.is_goal_reached():
            print("\n✓ GOAL REACHED!")
            self.print_state()
            return True
        else:
            print(f"\n⚠ Max steps ({max_steps}) exceeded")
            return False
    
    def stop_motors(self) -> None:
        """Stop all motors immediately"""
        self._execute_motor_command("stop")
        print("Motors stopped")


# Example usage
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Robot Navigation')
    parser.add_argument('--hardware', action='store_true', help='Use physical hardware (back_wheels)')
    parser.add_argument('--simulate', action='store_true', help='Use simulation only')
    parser.add_argument('--speed', type=int, default=50, help='Motor speed (0-100, default: 50)')
    parser.add_argument('--delay', type=float, default=0.5, help='Step delay in seconds (default: 0.5)')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    parser.add_argument('--interactive', action='store_true',
                        help='Enter interactive command loop instead of running demo')
    args = parser.parse_args()
    
    # helper to run the original demonstration sequence
    def run_demo(robot_obj, hardware_mode: bool):
        if hardware_mode:
            print(f"Speed: {args.speed}, Step delay: {args.delay}s\n")
            robot_obj.set_goal_location(goal_x=5.0, goal_y=5.0)
            robot_obj.move_to_goal_physical()
            robot_obj.stop_motors()
        else:
            print("=== SIMULATION MODE: Grid Navigation ===\n")
            print("=== Goal-Based Navigation on Infinite Grid ===\n")
            robot_obj.print_state()
            print("\n--- Step 1: Set initial goal direction (from sensor scan) ---")
            robot_obj.set_goal_direction(Direction.NORTHEAST)
            robot_obj.turn("right")  # Turn to northeast
            robot_obj.turn("right")
            robot_obj.print_state()
            print("\n--- Step 2: Move toward goal direction and perform full turn ---")
            for i in range(3):
                robot_obj.step("normal")
                robot_obj.print_state()
                robot_obj.full_turn_at_gridpoint(pause_duration=0.3)
            print("\n--- Step 3: Add detected obstacles ---")
            robot_obj.add_obstacle(8, 8, detection_radius=1.5)
            robot_obj.add_obstacle(5, 10, detection_radius=1.0)
            print("\n--- Step 4: Sensor identifies exact goal location ---")
            robot_obj.set_goal_location(goal_x=12.0, goal_y=12.0)
            robot_obj.print_state()
            print("\n--- Step 5: Check visibility to goal ---")
            if robot_obj.goal_in_line_of_sight():
                print("✓ Goal is in line of sight!")
            else:
                print("✗ Goal is blocked by obstacles")
            print("\n--- Step 6: Navigate towards goal ---")
            for i in range(5):
                dist = robot_obj.distance_to_goal()
                print(f"\n[Step {i+1}] Distance to goal: {dist:.2f}")
                if robot_obj.is_goal_reached():
                    print("✓ GOAL REACHED!")
                    robot_obj.print_state()
                    break
                goal_angle = robot_obj.direction_to_goal()
                current_angle = robot_obj.direction.value
                angle_diff = goal_angle - current_angle
                while angle_diff > 180:
                    angle_diff -= 360
                while angle_diff < -180:
                    angle_diff += 360
                while abs(angle_diff) > 22.5:
                    if angle_diff > 0:
                        robot_obj.turn("left")
                    else:
                        robot_obj.turn("right")
                    angle_diff = robot_obj.direction_to_goal() - robot_obj.direction.value
                    while angle_diff > 180:
                        angle_diff -= 360
                    while angle_diff < -180:
                        angle_diff += 360
                if robot_obj.move_towards_goal("normal"):
                    robot_obj.print_state()
                else:
                    print("Cannot move (blocked by obstacle or goal not in direction)")
            print("\n--- Final Status ---")
            if robot_obj.is_goal_reached():
                print("✓ Success: Robot reached the goal!")
            else:
                print(f"✗ Goal not reached. Distance remaining: {robot_obj.distance_to_goal():.2f}")
            robot_obj.print_state()

    def run_interactive(robot_obj, hardware_mode: bool):
        """Simple command interpreter for manual control and dynamic goal updates."""
        print("Entering interactive mode. type 'help' for commands, 'exit' to quit.")
        while True:
            try:
                line = input("cmd> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting interactive mode")
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            args_cmd = parts[1:]
            if cmd in ("exit", "quit"):
                break
            elif cmd == "help":
                print("Commands:\n  state                  - print robot state\n  step [normal|diagonal] - move one step\n  turn left|right        - turn 45°\n  set_goal x y           - set exact goal location\n  set_dir DIRECTION       - set initial goal direction\n  orient                 - point robot towards current goal\n  move_goal              - try move towards goal once\n  obstacles              - list obstacles\n  add_obs x y [r]        - add obstacle at grid with optional radius\n  clear_obs x y          - remove obstacle\n  exit                  - quit interactive mode")
            elif cmd == "state":
                robot_obj.print_state()
            elif cmd == "step":
                typ = args_cmd[0] if args_cmd else "normal"
                if hardware_mode:
                    robot_obj.physical_step(typ)
                else:
                    robot_obj.step(typ)
            elif cmd == "turn" and args_cmd:
                robot_obj.turn(args_cmd[0])
            elif cmd == "set_goal" and len(args_cmd) >= 2:
                x = float(args_cmd[0]); y = float(args_cmd[1])
                robot_obj.set_goal_location(x, y)
            elif cmd == "set_dir" and args_cmd:
                try:
                    d = Direction[args_cmd[0].upper()]
                    robot_obj.set_goal_direction(d)
                except KeyError:
                    print("Unknown direction. Use one of:", [d.name for d in Direction])
            elif cmd == "orient":
                robot_obj.point_towards_goal()
            elif cmd == "move_goal":
                if hardware_mode:
                    robot_obj.move_to_goal_physical()
                else:
                    robot_obj.move_towards_goal()
            elif cmd == "obstacles":
                print(robot_obj.get_obstacles())
            elif cmd == "add_obs" and len(args_cmd) >= 2:
                gx = int(args_cmd[0]); gy = int(args_cmd[1])
                radius = float(args_cmd[2]) if len(args_cmd) > 2 else 1.0
                robot_obj.add_obstacle(gx, gy, radius)
            elif cmd == "clear_obs" and len(args_cmd) == 2:
                gx = int(args_cmd[0]); gy = int(args_cmd[1])
                robot_obj.remove_obstacle(gx, gy)
            else:
                print(f"Unknown command: {cmd}. type 'help' for list.")

    # create robot instance (common for both modes)
    if args.hardware:
        print("=== HARDWARE MODE: Using Physical Back Wheels ===\n")
        try:
            from picar.back_wheels import Back_Wheels
            back_wheels = Back_Wheels(debug=args.debug)
            robot = HardwareRobot(back_wheels=back_wheels, speed=args.speed,
                                 step_delay=args.delay, debug=args.debug)
        except ImportError as e:
            print(f"ERROR: Could not import back_wheels module: {e}")
            print("Make sure picar module is in the path and TB6612/PCA9685 are installed")
            sys.exit(1)
    else:
        robot = GridRobot(cell_size=1.0)

    # choose mode
    if args.interactive:
        run_interactive(robot, args.hardware)
    else:
        run_demo(robot, args.hardware)

# !/usr/bin/env python3

import asyncio
import time
import math
import logging.handlers
from pathlib import Path
from typing import Dict, Any, List, Tuple
from dataclasses import dataclass, field

from pymavlink import mavutil

from litestar import Litestar, get, post
from litestar.controller import Controller
from litestar.datastructures import State
from litestar.logging import LoggingConfig
from litestar.static_files.config import StaticFilesConfig
from litestar.dto import DataclassDTO
from litestar.exceptions import HTTPException

# ========================
# Enhanced Mission Configuration with Larger Patterns
# ========================

@dataclass
class MissionConfig:
    mission_depth: float = 2.0
    duration: int = 60
    movement_pattern: str = "survey"
    # For survey - larger default area
    waypoints: List[Tuple[float, float, float]] = None
    # For circle scan - much larger and more visible
    circle_center: Tuple[float, float] = (0.0, 0.0)
    circle_radius: float = 8.0  # Increased from 1.0 to 8.0 meters
    circle_steps: int = 32  # More steps for smoother visualization
    # For advanced patterns - larger scales
    spiral_turns: int = 4
    figure_eight_size: float = 10.0  # Increased from 2.0 to 10.0 meters
    hold_time_per_waypoint: int = 2  # Reduced for faster movement
    # Multi-depth scanning
    depth_levels: List[float] = field(default_factory=lambda: [1.0, 2.0, 3.0])
    # Advanced circular patterns - larger radii
    multi_circle_radii: List[float] = field(default_factory=lambda: [3.0, 6.0, 9.0, 12.0])
    oscillation_amplitude: float = 1.0  # Increased amplitude
    # Lawnmower pattern - larger area
    lawnmower_width: float = 20.0
    lawnmower_height: float = 15.0
    lawnmower_spacing: float = 2.0

@dataclass
class MissionStatus:
    active: bool = False
    phase: str = "idle"
    connected: bool = False
    armed: bool = False
    last_position: Tuple[float, float, float] = (0, 0, 0)
    last_depth: float = 0.0
    last_yaw: float = 0.0
    current_waypoint: int = 0
    total_waypoints: int = 0
    mission_progress: float = 0.0
    pattern_dimensions: str = ""

class MissionConfigDTO(DataclassDTO[MissionConfig]):
    pass

class MissionStatusDTO(DataclassDTO[MissionStatus]):
    pass

# ========================
# Enhanced MAVLink Mission Controller
# ========================

class BlueROVMissionController:
    def __init__(self):
        self.master = None
        self.mission_active = False
        self.mission_status = MissionStatus()
        self.current_task = None
        self.connection_string = 'udp:127.0.0.1:14552'  # BlueOS default
        self.loop_rate = 0.5  # Faster updates for better visualization
        self.position_update_rate = 0.2  # Even faster position updates

    async def connect_mavlink(self):
        try:
            if self.master:
                return True
            print("Connecting to vehicle...")
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat(timeout=10)
            print("Connected to system ID:", self.master.target_system)
            self.mission_status.connected = True
            return True
        except Exception as e:
            print(f"MAVLink connection failed: {e}")
            self.mission_status.connected = False
            self.master = None
            return False

    def set_mode(self, mode_name):
        if not self.master:
            return False
        try:
            mode_id = self.master.mode_mapping()[mode_name]
            self.master.set_mode(mode_id)
            print(f"Mode set to {mode_name}")
            
            # Wait for mode confirmation with timeout
            start_time = time.time()
            while time.time() - start_time < 5:  # 5 second timeout
                ack = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if ack and ack.custom_mode == mode_id:
                    return True
            return False
        except Exception as e:
            print(f"Failed to set mode {mode_name}: {e}")
            return False

    def arm(self):
        if not self.master:
            return False
        try:
            print("Arming...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0)
            self.master.motors_armed_wait()
            print("Vehicle armed.")
            self.mission_status.armed = True
            return True
        except Exception as e:
            print(f"Failed to arm: {e}")
            return False

    def disarm(self):
        if not self.master:
            return False
        try:
            print("Disarming...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0)
            print("Vehicle disarmed.")
            self.mission_status.armed = False
            return True
        except Exception as e:
            print(f"Failed to disarm: {e}")
            return False

    def goto_position(self, x, y, z):
        if not self.master:
            return
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # Enable x,y,z
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0)
        print(f"Moving to: X={x:.2f} Y={y:.2f} Z={z:.2f}")
        self.mission_status.last_position = (x, y, z)

    def get_position(self):
        if not self.master:
            return 0.0, 0.0
        try:
            attitude = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
            yaw_deg = math.degrees(attitude.yaw) if attitude else 0.0
            hud = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            depth = -hud.alt if hud else 0.0
            self.mission_status.last_depth = depth
            self.mission_status.last_yaw = yaw_deg
            return depth, yaw_deg
        except:
            return 0.0, 0.0

    def go_and_hold(self, x, y, z, hold_time):
        self.goto_position(x, y, z)
        
        # Use faster update rate for better visualization
        updates_per_second = int(1 / self.position_update_rate)
        total_updates = hold_time * updates_per_second
        
        for i in range(total_updates):
            if not self.mission_active:
                break
            depth, yaw = self.get_position()
            
            # Print position updates less frequently to avoid spam
            if i % 5 == 0:  # Every 5th update
                print(f"Hold: x={x:.1f}, y={y:.1f}, z={z:.1f}, depth={depth:.1f}, yaw={yaw:.1f}")
            
            time.sleep(self.position_update_rate)

    def debug_print_waypoints(self, waypoints, pattern_name):
        """Print waypoints for debugging and verification"""
        print(f"\n=== {pattern_name} Pattern Waypoints ===")
        for i, (x, y, z) in enumerate(waypoints[:5]):  # Show first 5 waypoints
            print(f"Waypoint {i+1}: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m")
        
        if len(waypoints) > 5:
            print(f"... and {len(waypoints) - 5} more waypoints")
        
        # Calculate pattern dimensions
        x_coords = [wp[0] for wp in waypoints]
        y_coords = [wp[1] for wp in waypoints]
        
        x_range = max(x_coords) - min(x_coords)
        y_range = max(y_coords) - min(y_coords)
        
        dimensions = f"{x_range:.1f}m x {y_range:.1f}m"
        self.mission_status.pattern_dimensions = dimensions
        
        print(f"Pattern dimensions: {dimensions}")
        print(f"Total waypoints: {len(waypoints)}")
        print("=" * 50)

    # ========================
    # Enhanced Pattern Generation with Large Scales
    # ========================

    def generate_large_survey_pattern(self, config: MissionConfig):
        """Generate large survey pattern for better visibility"""
        waypoints = [
            (0, 0, -config.mission_depth),
            (15, 0, -config.mission_depth),    # 15m instead of 3m
            (15, 8, -config.mission_depth),    # 8m instead of 2m
            (0, 8, -config.mission_depth),
            (0, 16, -config.mission_depth),    # 16m instead of 4m
            (15, 16, -config.mission_depth),
            (15, 24, -config.mission_depth),   # Extended pattern
            (0, 24, -config.mission_depth),
            (0, 32, -config.mission_depth),    # Even larger
            (15, 32, -config.mission_depth),
        ]
        return waypoints

    def generate_circular_pattern(self, center, radius, steps, depth):
        """Generate circular pattern waypoints"""
        cx, cy = center
        waypoints = []
        for i in range(steps):
            angle = 2 * math.pi * i / steps
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            z = -depth
            waypoints.append((x, y, z))
        return waypoints

    def generate_spiral_pattern(self, center, max_radius, turns, steps_per_turn, depth):
        """Generate spiral pattern waypoints"""
        cx, cy = center
        waypoints = []
        total_steps = turns * steps_per_turn
        
        for i in range(total_steps):
            angle = 2 * math.pi * i / steps_per_turn
            radius = max_radius * (i / total_steps)
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            z = -depth
            waypoints.append((x, y, z))
        return waypoints

    def generate_figure_eight_pattern(self, center, size, steps, depth):
        """Generate large figure-eight pattern waypoints"""
        cx, cy = center
        waypoints = []
        
        for i in range(steps):
            t = 2 * math.pi * i / steps
            x = cx + size * math.sin(t)
            y = cy + size * math.sin(t) * math.cos(t)
            z = -depth
            waypoints.append((x, y, z))
        return waypoints

    def generate_multi_circle_pattern(self, center, radii, steps_per_circle, depth):
        """Generate multiple concentric circles"""
        waypoints = []
        for radius in radii:
            circle_waypoints = self.generate_circular_pattern(center, radius, steps_per_circle, depth)
            waypoints.extend(circle_waypoints)
        return waypoints

    def generate_oscillating_circle_pattern(self, center, radius, steps, depth, amplitude):
        """Generate circular pattern with depth oscillation"""
        cx, cy = center
        waypoints = []
        
        for i in range(steps):
            angle = 2 * math.pi * i / steps
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            # Oscillate depth
            depth_offset = amplitude * math.sin(4 * angle)  # 4 oscillations per circle
            z = -depth + depth_offset
            waypoints.append((x, y, z))
        return waypoints

    def generate_large_lawnmower_pattern(self, config: MissionConfig):
        """Generate large lawnmower/grid search pattern"""
        width = config.lawnmower_width
        height = config.lawnmower_height
        spacing = config.lawnmower_spacing
        depth = config.mission_depth
        
        waypoints = []
        y = 0
        direction = 1
        
        while y <= height:
            if direction == 1:
                # Left to right
                for x_step in range(int(width / spacing) + 1):
                    x = x_step * spacing
                    waypoints.append((x, y, -depth))
            else:
                # Right to left
                for x_step in range(int(width / spacing), -1, -1):
                    x = x_step * spacing
                    waypoints.append((x, y, -depth))
            
            y += spacing
            direction *= -1
        
        return waypoints

    def generate_star_pattern(self, center, outer_radius, inner_radius, points, depth):
        """Generate star pattern for interesting visualization"""
        cx, cy = center
        waypoints = []
        
        for i in range(points * 2):
            angle = math.pi * i / points
            if i % 2 == 0:
                radius = outer_radius
            else:
                radius = inner_radius
            
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            z = -depth
            waypoints.append((x, y, z))
        
        # Close the star
        waypoints.append(waypoints[0])
        return waypoints

    # ========================
    # Enhanced Mission Execution
    # ========================

    def execute_waypoint_mission(self, waypoints, hold_time):
        """Execute a list of waypoints with progress tracking"""
        self.mission_status.total_waypoints = len(waypoints)
        
        for i, wp in enumerate(waypoints):
            if not self.mission_active:
                break
                
            self.mission_status.current_waypoint = i + 1
            self.mission_status.mission_progress = (i + 1) / len(waypoints) * 100
            
            x, y, z = wp
            print(f"Waypoint {i+1}/{len(waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f})")
            self.go_and_hold(x, y, z, hold_time)

    def do_advanced_circle(self, config: MissionConfig):
        """Execute advanced circular patterns"""
        if config.multi_circle_radii:
            # Multi-circle pattern
            waypoints = self.generate_multi_circle_pattern(
                config.circle_center, 
                config.multi_circle_radii, 
                config.circle_steps, 
                config.mission_depth
            )
            self.debug_print_waypoints(waypoints, "Multi-Circle")
        elif config.oscillation_amplitude > 0:
            # Oscillating circle
            waypoints = self.generate_oscillating_circle_pattern(
                config.circle_center,
                config.circle_radius,
                config.circle_steps,
                config.mission_depth,
                config.oscillation_amplitude
            )
            self.debug_print_waypoints(waypoints, "Oscillating Circle")
        else:
            # Standard large circle
            waypoints = self.generate_circular_pattern(
                config.circle_center,
                config.circle_radius,
                config.circle_steps,
                config.mission_depth
            )
            self.debug_print_waypoints(waypoints, "Large Circle")
        
        self.execute_waypoint_mission(waypoints, config.hold_time_per_waypoint)

    def do_spiral_scan(self, config: MissionConfig):
        """Execute spiral scan pattern"""
        waypoints = self.generate_spiral_pattern(
            config.circle_center,
            config.circle_radius,
            config.spiral_turns,
            config.circle_steps,
            config.mission_depth
        )
        self.debug_print_waypoints(waypoints, "Spiral Scan")
        self.execute_waypoint_mission(waypoints, config.hold_time_per_waypoint)

    def do_figure_eight(self, config: MissionConfig):
        """Execute figure-eight pattern"""
        waypoints = self.generate_figure_eight_pattern(
            config.circle_center,
            config.figure_eight_size,
            config.circle_steps * 2,  # More points for smooth figure-eight
            config.mission_depth
        )
        self.debug_print_waypoints(waypoints, "Figure Eight")
        self.execute_waypoint_mission(waypoints, config.hold_time_per_waypoint)

    def do_multi_depth_scan(self, config: MissionConfig):
        """Execute scan at multiple depth levels"""
        depths = config.depth_levels
        
        for depth in depths:
            if not self.mission_active:
                break
                
            print(f"Scanning at depth: {depth}m")
            # Go to depth
            self.go_and_hold(0, 0, -depth, 5)
            
            # Execute circular pattern at this depth
            waypoints = self.generate_circular_pattern(
                config.circle_center,
                config.circle_radius,
                config.circle_steps,
                depth
            )
            self.execute_waypoint_mission(waypoints, config.hold_time_per_waypoint)

    def do_star_pattern(self, config: MissionConfig):
        """Execute star pattern"""
        waypoints = self.generate_star_pattern(
            config.circle_center,
            config.circle_radius,
            config.circle_radius * 0.5,
            5,  # 5-pointed star
            config.mission_depth
        )
        self.debug_print_waypoints(waypoints, "Star Pattern")
        self.execute_waypoint_mission(waypoints, config.hold_time_per_waypoint)

    async def execute_mission(self, config: MissionConfig):
        """Execute enhanced mission with large, visible patterns"""
        try:
            self.mission_active = True
            self.mission_status.active = True
            self.mission_status.phase = "connecting"
            
            if not await self.connect_mavlink():
                raise Exception("Failed to connect to vehicle")

            self.mission_status.phase = "arming"
            if not self.arm():
                raise Exception("Failed to arm vehicle")
            time.sleep(2)

            self.mission_status.phase = "set_mode"
            if not self.set_mode("GUIDED"):
                raise Exception("Failed to set GUIDED mode")
            time.sleep(2)

            # Step 1: Go to operating depth
            self.mission_status.phase = "descending"
            self.go_and_hold(0, 0, -config.mission_depth, 8)

            # Execute different mission patterns
            if config.movement_pattern == "survey":
                self.mission_status.phase = "large_survey"
                waypoints = config.waypoints or self.generate_large_survey_pattern(config)
                self.debug_print_waypoints(waypoints, "Large Survey")
                self.execute_waypoint_mission(waypoints, config.hold_time_per_waypoint)
                
            elif config.movement_pattern == "circle":
                self.mission_status.phase = "circle_scan"
                self.do_advanced_circle(config)
                
            elif config.movement_pattern == "spiral":
                self.mission_status.phase = "spiral_scan"
                self.do_spiral_scan(config)
                
            elif config.movement_pattern == "figure_eight":
                self.mission_status.phase = "figure_eight"
                self.do_figure_eight(config)
                
            elif config.movement_pattern == "multi_depth":
                self.mission_status.phase = "multi_depth_scan"
                self.do_multi_depth_scan(config)
                
            elif config.movement_pattern == "lawnmower":
                self.mission_status.phase = "lawnmower_pattern"
                waypoints = self.generate_large_lawnmower_pattern(config)
                self.debug_print_waypoints(waypoints, "Large Lawnmower")
                self.execute_waypoint_mission(waypoints, config.hold_time_per_waypoint)
                
            elif config.movement_pattern == "star":
                self.mission_status.phase = "star_pattern"
                self.do_star_pattern(config)
                
            else:
                self.mission_status.phase = "hover"
                self.go_and_hold(0, 0, -config.mission_depth, config.duration)

            # Return to start and surface
            self.mission_status.phase = "return"
            self.go_and_hold(0, 0, -config.mission_depth, 5)

            self.mission_status.phase = "surface"
            self.go_and_hold(0, 0, 0.2, 5)

            self.mission_status.phase = "disarming"
            self.disarm()
            self.mission_status.phase = "completed"
            print("Mission completed successfully!")

        except Exception as e:
            self.mission_status.phase = f"error: {str(e)}"
            print(f"Mission failed: {e}")
        finally:
            self.mission_active = False
            self.mission_status.active = False
            self.mission_status.mission_progress = 100.0

    def stop_mission(self):
        self.mission_active = False
        self.mission_status.phase = "stopped"
        print("Mission stopped by user.")

# ========================
# Enhanced API Controller
# ========================

mission_controller = BlueROVMissionController()

class MissionController(Controller):
    path = "/mission"

    @get("/status")
    async def get_status(self) -> MissionStatus:
        return mission_controller.mission_status

    @post("/start")
    async def start_mission(self, data: MissionConfig) -> Dict[str, Any]:
        if mission_controller.mission_active:
            raise HTTPException(status_code=400, detail="Mission already active")
        
        mission_controller.current_task = asyncio.create_task(
            mission_controller.execute_mission(data)
        )
        
        return {
            "status": "Mission started",
            "pattern": data.movement_pattern,
            "config": {
                "mission_depth": data.mission_depth,
                "movement_pattern": data.movement_pattern,
                "circle_center": data.circle_center,
                "circle_radius": data.circle_radius,
                "circle_steps": data.circle_steps,
                "pattern_scale": "Large scale for better QGC visibility"
            }
        }

    @post("/stop")
    async def stop_mission(self) -> Dict[str, str]:
        mission_controller.stop_mission()
        if mission_controller.current_task:
            mission_controller.current_task.cancel()
        return {"status": "Mission stopped"}

    @post("/connect")
    async def connect_vehicle(self) -> Dict[str, Any]:
        success = await mission_controller.connect_mavlink()
        return {"status": "Connected" if success else "Connection failed", "connected": success}

    @get("/patterns")
    async def get_available_patterns(self) -> Dict[str, Any]:
        """Get list of available mission patterns with size information"""
        return {
            "patterns": [
                "survey",
                "circle", 
                "spiral",
                "figure_eight",
                "multi_depth",
                "lawnmower",
                "star",
                "hover"
            ],
            "descriptions": {
                "survey": "Large zigzag survey pattern (15x32m area)",
                "circle": "Large circular scan (8m radius default)",
                "spiral": "Spiral pattern from center outward (8m radius)",
                "figure_eight": "Large figure-eight pattern (10m size)",
                "multi_depth": "Multi-level depth scanning with circles",
                "lawnmower": "Large grid search pattern (20x15m area)",
                "star": "5-pointed star pattern for testing",
                "hover": "Simple hover at depth"
            },
            "default_scales": {
                "circle_radius": "8.0m (highly visible)",
                "survey_area": "15x32m",
                "lawnmower_area": "20x15m",
                "figure_eight_size": "10.0m",
                "recommended_qgc_zoom": "Zoom to 50-100m range for best visibility"
            }
        }

    @get("/test-pattern/<pattern_name:str>")
    async def preview_pattern(self, pattern_name: str) -> Dict[str, Any]:
        """Preview waypoints for a specific pattern"""
        config = MissionConfig(movement_pattern=pattern_name)
        
        if pattern_name == "survey":
            waypoints = mission_controller.generate_large_survey_pattern(config)
        elif pattern_name == "circle":
            waypoints = mission_controller.generate_circular_pattern(
                config.circle_center, config.circle_radius, config.circle_steps, config.mission_depth
            )
        elif pattern_name == "lawnmower":
            waypoints = mission_controller.generate_large_lawnmower_pattern(config)
        elif pattern_name == "spiral":
            waypoints = mission_controller.generate_spiral_pattern(
                config.circle_center, config.circle_radius, config.spiral_turns, config.circle_steps, config.mission_depth
            )
        else:
            return {"error": "Pattern not found"}
        
        # Calculate dimensions
        x_coords = [wp[0] for wp in waypoints]
        y_coords = [wp[1] for wp in waypoints]
        x_range = max(x_coords) - min(x_coords)
        y_range = max(y_coords) - min(y_coords)
        
        return {
            "pattern": pattern_name,
            "waypoint_count": len(waypoints),
            "dimensions": f"{x_range:.1f}m x {y_range:.1f}m",
            "first_5_waypoints": waypoints[:5],
            "bounding_box": {
                "min_x": min(x_coords),
                "max_x": max(x_coords),
                "min_y": min(y_coords),
                "max_y": max(y_coords)
            }
        }

# ========================
# Logging & App Setup
# ========================

logging_config = LoggingConfig(
    loggers={
        __name__: dict(
            level='INFO',
            handlers=['queue_listener'],
        )
    },
)

log_dir = Path('/app/logs')
log_dir.mkdir(parents=True, exist_ok=True)
fh = logging.handlers.RotatingFileHandler(log_dir / 'mission.log', maxBytes=2**16, backupCount=1)

app = Litestar(
    route_handlers=[MissionController],
    state=State({'bag_url':'http://host.docker.internal/bag/v1.0'}),
    static_files_config=[
        StaticFilesConfig(directories=['app/static'], path='/', html_mode=True)
    ],
    logging_config=logging_config,
)

app.logger.addHandler(fh)

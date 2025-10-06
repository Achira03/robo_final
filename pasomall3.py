import robomaster
from robomaster import robot, camera, gimbal, blaster, sensor
import cv2
import numpy as np
import time
import math
import threading
import queue
import heapq
import itertools
import sys
import pickle

# ===================================================================
# SECTION 1: MERGED CONFIGURATION & GLOBAL VARIABLES
# ===================================================================

DATA_FILENAME = "maze_run_data.pkl"

# --- المتغيرات من حل المتاهة (achi.py) ---
tof_distances = [0, 0, 0, 0]
current_position = [0, 0, 0]
current_attitude = [0, 0, 0]
initial_yaw = None
initial_yaw_set = False
is_tof_scanning = True
YOUR_A_VALUE = 29.58
YOUR_B_VALUE = -1.53
NO_OBJECT_VOLTAGE_THRESHOLD = 1.0
MAX_ADC_VALUE = 1023.0
REFERENCE_VOLTAGE = 5.0
GRID_DISTANCE = 0.60
BLOCKED_DISTANCE_MM = 400
WALL_STOP_DISTANCE_MM = 150
TARGET_SIDE_DISTANCE_CM = 14
P_GAIN_SLIDE_ADJUST = 1.8
MAX_SLIDE_ADJUST_SPEED = 0.25
ADJUSTMENT_TIME_LIMIT_S = 3.0

# --- المتغيرات من اكتشاف الكائنات (ditect_project_gimbal11.py) ---
WIDTH_SQUARE = 7.0
DIAMETER_CIRCLE = 7.0
WIDTH_RECT_H = 9.0
HEIGHT_RECT_H = 6.0
WIDTH_RECT_V = 6.0
HEIGHT_RECT_V = 9.0
FOCAL_LENGTH_PIXELS = 600
CENTER_TOLERANCE = 15
YAW_KP = 0.12
PITCH_KP = 0.15
MAX_YAW_SPEED = 60
MAX_PITCH_SPEED = 60
MAX_SHOOT_DISTANCE_CM = 120
FIRE_COOLDOWN_SECONDS = 5.0
color_ranges = {
    "brown_red":      ([0, 150, 50],   [10, 255, 120],   (0, 0, 255)),
    "dark_green":     ([70, 150, 20],   [90, 255, 70],    (0, 255, 0)),
    "yellow_brown":   ([15, 150, 80],   [30, 255, 180],   (0, 255, 255)),
}

# --- Threading & Shared State Variables ---
gimbal_lock = threading.Lock()
program_running = True
robot_is_stationary = threading.Event()
shot_targets = set()
last_fire_time = 0
robot_current_grid_pos = (0, 0)
robot_current_heading_deg = 0 

# ===================================================================
# SECTION 2: MERGED CALLBACKS & HELPER FUNCTIONS
# ===================================================================
def sub_tof_data_handler(sub_info):
    global tof_distances, is_tof_scanning
    if is_tof_scanning:
        tof_distances = sub_info

def position_handler(info):
    global current_position
    current_position = info

def attitude_handler(info):
    global current_attitude, initial_yaw, initial_yaw_set
    current_attitude = info
    if not initial_yaw_set:
        initial_yaw = current_attitude[0]
        initial_yaw_set = True
        print(f"--- Initial yaw set to: {initial_yaw:.2f} degrees ---")

def adc_to_cm(voltage):
    if voltage < NO_OBJECT_VOLTAGE_THRESHOLD: return float('inf')
    if voltage > 3.2: return 5.0
    try:
        return YOUR_A_VALUE * math.pow(voltage, YOUR_B_VALUE)
    except (ValueError, ZeroDivisionError):
        return float('inf')

def active_brake_sleep(ep_chassis, duration):
    start_time = time.time()
    while time.time() - start_time < duration:
        ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(0.05)

def detect_shape(contour):
    shape = "Unknown"
    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
    num_vertices = len(approx)

    if num_vertices == 4:
        (x, y, w, h) = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        if 0.9 <= aspect_ratio <= 1.1:
            shape = "Square"
        elif aspect_ratio > 1.1:
            shape = "Rectangle-H"
        else:
            shape = "Rectangle-V"
    elif num_vertices > 5:
        shape = "Circle"
    return shape

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(maze_map, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {pos: float('inf') for pos in maze_map.keys()}
    g_score[start] = 0
    f_score = {pos: float('inf') for pos in maze_map.keys()}
    f_score[start] = heuristic(start, goal)

    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for direction, is_wall in maze_map.get(current, {}).items():
            if not is_wall:
                neighbor = get_neighbor_coords(current[0], current[1], direction)
                if neighbor not in maze_map:
                    continue
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    if not any(neighbor == item[1] for item in open_set):
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

def save_run_data(maze_map, targets, filename):
    print(f"\n--- Saving run data to {filename} ---")
    data_to_save = {'maze_map': maze_map, 'targets_found': targets}
    with open(filename, 'wb') as f:
        pickle.dump(data_to_save, f)
    print("Data saved successfully.")

def load_run_data(filename):
    try:
        with open(filename, 'rb') as f:
            data = pickle.load(f)
            print(f"--- Loaded run data from {filename} ---")
            return data
    except FileNotFoundError:
        print(f"[Error] Data file '{filename}' not found. Please run 'run1' mode first.")
        return None

# ===================================================================
# SECTION 3: THREAD 1 - DETECTION AND SHOOTING LOGIC
# ===================================================================

def detection_and_shooting_thread_func(ep_robot, current_pos_getter, current_heading_getter):
    global shot_targets, last_fire_time, program_running

    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_blaster = ep_robot.blaster
    
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    morph_kernel = np.ones((5, 5), np.uint8)
    print("[Detection Thread] Started.")

    while program_running:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            if img is None: continue
        except queue.Empty:
            continue

        frame = img.copy()
        frame_height, frame_width, _ = frame.shape
        roi_w, roi_h = 800, 450
        roi_x, roi_y = (frame_width - roi_w) // 2, (frame_height - roi_h) // 2
        roi_center_x, roi_center_y = roi_w // 2, roi_h // 2
        roi_frame = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
        hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        
        # ✅✅✅ FIX: เปลี่ยนจากการหา "พื้นที่ใหญ่สุด" เป็นหา "ระยะห่างจากศูนย์กลางน้อยที่สุด"
        closest_center_dist = float('inf') 
        aiming_target_info = None
        found_target_in_frame = False

        for color_name, (lower, upper, box_color) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morph_kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    shape_name = detect_shape(cnt)
                    if shape_name != "Unknown":
                        target_id = (color_name, shape_name)
                        
                        distance_cm, new_x, new_y = 0, 0, 0
                        draw_x, draw_y, w, h = 0,0,0,0 
                        
                        if shape_name == "Circle":
                            (x_c, y_c), radius = cv2.minEnclosingCircle(cnt)
                            new_x, new_y = int(x_c - roi_center_x), int(roi_center_y - y_c)
                            center_abs, radius_val = (int(x_c) + roi_x, int(y_c) + roi_y), int(radius)
                            draw_x, draw_y, w, h = center_abs[0] - radius_val, center_abs[1] - radius_val, 2*radius_val, 2*radius_val
                            if radius_val > 0: distance_cm = (DIAMETER_CIRCLE * FOCAL_LENGTH_PIXELS) / (2 * radius_val)
                        else: 
                            x, y, w, h = cv2.boundingRect(cnt)
                            new_x, new_y = int((x + w / 2) - roi_center_x), int(roi_center_y - (y + h / 2))
                            draw_x, draw_y = x + roi_x, y + roi_y
                            if shape_name == "Square": distance_cm = (WIDTH_SQUARE * FOCAL_LENGTH_PIXELS) / w
                            elif shape_name == "Rectangle-V": distance_cm = (HEIGHT_RECT_V * FOCAL_LENGTH_PIXELS) / h
                            elif shape_name == "Rectangle-H": distance_cm = (WIDTH_RECT_H * FOCAL_LENGTH_PIXELS) / w

                        # วาดข้อมูลลงบน frame สำหรับ debug (ทำก่อนเงื่อนไข)
                        label, coord_label = f"{shape_name} {distance_cm:.1f}cm", f"({new_x}, {new_y})"
                        if shape_name == "Circle":
                            cv2.circle(frame, center_abs, radius_val, box_color, 2)
                            cv2.putText(frame, label, (center_abs[0] - radius_val, center_abs[1] - radius_val - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                            cv2.putText(frame, coord_label, (center_abs[0] - radius_val, center_abs[1] - radius_val - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                        else:
                            cv2.rectangle(frame, (draw_x, draw_y), (draw_x + w, draw_y + h), box_color, 2)
                            cv2.putText(frame, label, (draw_x, draw_y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                            cv2.putText(frame, coord_label, (draw_x, draw_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                        if any(t[0] == target_id for t in shot_targets):
                            cv2.putText(frame, "SHOT!", (draw_x, draw_y + h + 20), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)
                        
                        # ✅✅✅ FIX: เปลี่ยนเงื่อนไขการเลือกเป้าหมาย
                        # ถ้าเป้านี้ยังไม่เคยถูกยิง...
                        if not any(t[0] == target_id for t in shot_targets):
                            # คำนวณระยะห่างจากจุดศูนย์กลาง (Euclidean distance)
                            center_dist = math.sqrt(new_x**2 + new_y**2)
                            # ถ้าเป้านี้อยู่ใกล้ศูนย์กลางมากกว่าเป้าที่เคยเจอในเฟรมนี้
                            if center_dist < closest_center_dist:
                                # อัปเดตเป้าหมายที่จะเล็ง
                                closest_center_dist = center_dist
                                found_target_in_frame = True
                                aiming_target_info = {"x_offset": new_x, "y_offset": new_y, "id": target_id, "distance": distance_cm}
        
        can_shoot = robot_is_stationary.is_set()
        
        if found_target_in_frame and aiming_target_info and can_shoot:
            if gimbal_lock.acquire(blocking=False):
                try:
                    yaw_speed = np.clip(YAW_KP * aiming_target_info["x_offset"], -MAX_YAW_SPEED, MAX_YAW_SPEED)
                    pitch_speed = np.clip(PITCH_KP * aiming_target_info["y_offset"], -MAX_PITCH_SPEED, MAX_PITCH_SPEED)
                    ep_gimbal.drive_speed(yaw_speed=yaw_speed, pitch_speed=pitch_speed)

                    is_in_range = aiming_target_info["distance"] <= MAX_SHOOT_DISTANCE_CM
                    is_centered = abs(aiming_target_info["x_offset"]) <= CENTER_TOLERANCE and abs(aiming_target_info["y_offset"]) <= CENTER_TOLERANCE
                    is_cooldown_passed = (time.time() - last_fire_time) > FIRE_COOLDOWN_SECONDS

                    if is_in_range and is_centered and is_cooldown_passed:
                        target_color, target_shape = aiming_target_info["id"]
                        current_robot_pos = current_pos_getter()
                        current_robot_heading = current_heading_getter()
                        print(f"[Detection Thread] Target {target_color} {target_shape} at {current_robot_pos} (facing {current_robot_heading} deg) centered. FIRING!")
                        ep_blaster.fire(fire_type=blaster.WATER_FIRE, times=3)
                        shot_targets.add((aiming_target_info["id"], current_robot_pos, current_robot_heading))
                        last_fire_time = time.time()
                finally:
                    gimbal_lock.release()
        
        cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 255, 255), 2)
        cv2.imshow("Detection Feed", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            shot_targets.clear()
            print("[Detection Thread] --- All shot targets have been reset. ---")

    ep_camera.stop_video_stream()
    cv2.destroyAllWindows()
    print("[Detection Thread] Stopped.")

# ===================================================================
# SECTION 4: MOVEMENT & MAZE FUNCTIONS
# ===================================================================
def scan_surroundings_and_targets(ep_gimbal, ep_chassis):
    global is_tof_scanning, gimbal_lock
    is_tof_scanning = True
    distances = {'left': 0, 'front': 0, 'right': 0}
    scan_speed = 300
    pause_for_detection_s = 2.0 

    print("--- Scanning surroundings and for targets... ---")
    try:
        with gimbal_lock:
            ep_gimbal.moveto(pitch=-10, yaw=90, yaw_speed=scan_speed).wait_for_completed()
            ep_chassis.move(x=0,y=0,z=0).wait_for_completed()
            time.sleep(0.9)
        print("Scanning Left. Pausing for detection...")
        time.sleep(pause_for_detection_s)
        distances['left'] = tof_distances[0]

        with gimbal_lock:
            ep_gimbal.moveto(pitch=-10, yaw=-90, yaw_speed=scan_speed).wait_for_completed()
            ep_chassis.move(x=0,y=0,z=0).wait_for_completed()
            time.sleep(0.9)
        print("Scanning Right. Pausing for detection...")
        time.sleep(pause_for_detection_s)
        distances['right'] = tof_distances[0]

        with gimbal_lock:
            ep_gimbal.moveto(pitch=-10, yaw=0, yaw_speed=scan_speed).wait_for_completed()
            ep_chassis.move(x=0,y=0,z=0).wait_for_completed()
            time.sleep(0.9)
        print("Scanning Front. Pausing for detection...")
        time.sleep(pause_for_detection_s)
        distances['front'] = tof_distances[0]
        
    finally:
        with gimbal_lock:
            ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=scan_speed).wait_for_completed()
            ep_chassis.move(x=0,y=0,z=0).wait_for_completed()
            time.sleep(0.9)
        ep_chassis.drive_speed(x=0, y=0, z=0)
    
    print(f"--- Scan complete: L={distances['left']} F={distances['front']} R={distances['right']} ---")
    return distances

def adjust_side_distance(ep_chassis, ep_sensor_adaptor):
    print(f"\n--- Adjusting Side Distance (Active-Low Logic) ---")
    start_time = time.time()
    
    ir_left_check = ep_sensor_adaptor.get_io(id=1, port=1)
    ir_right_check = ep_sensor_adaptor.get_io(id=2, port=1)
    
    if ir_left_check == 1 and ir_right_check == 1:
        print("No walls detected by IR -> skipping adjustment.")
        print("--- Side Adjustment Complete ---")
        return

    while time.time() - start_time < ADJUSTMENT_TIME_LIMIT_S:
        left_raw_adc = ep_sensor_adaptor.get_adc(id=1, port=2)
        right_raw_adc = ep_sensor_adaptor.get_adc(id=2, port=2)
        ir_left = ep_sensor_adaptor.get_io(id=1, port=1)
        ir_right = ep_sensor_adaptor.get_io(id=2, port=1)

        left_dist_cm = adc_to_cm((left_raw_adc / MAX_ADC_VALUE) * REFERENCE_VOLTAGE)
        right_dist_cm = adc_to_cm((right_raw_adc / MAX_ADC_VALUE) * REFERENCE_VOLTAGE)
        
        if math.isinf(left_dist_cm) or math.isinf(right_dist_cm):
            print(f"\r[Safety] Infinite distance detected. Cancelling adjustment.", end="")
            break

        error_side = 0.0
        side_found = "None"

        if ir_left == 0 and ir_right == 0:
            error_side = (left_dist_cm - right_dist_cm) / 2.0
            side_found = 'Both walls'
        elif ir_left == 0 and ir_right == 1:
            error_side = left_dist_cm - TARGET_SIDE_DISTANCE_CM
            side_found = 'Left wall only'
        elif ir_right == 0 and ir_left == 1:
            error_side = TARGET_SIDE_DISTANCE_CM - right_dist_cm
            side_found = 'Right wall only'
        else:
            print("Lost wall detection, stopping adjustment.")
            break
            
        if abs(error_side) < 1.5:
            print(f"\nAdjustment successful. Final error: {error_side:.2f} cm")
            break

        slide_speed = -P_GAIN_SLIDE_ADJUST * (error_side / 100.0)
        slide_speed = max(min(slide_speed, MAX_SLIDE_ADJUST_SPEED), -MAX_SLIDE_ADJUST_SPEED)

        ep_chassis.drive_speed(x=0, y=slide_speed, z=0)
        print(f"\rAdjusting based on {side_found}... Error: {error_side:.2f} cm, Speed: {slide_speed:.2f}", end="")
        time.sleep(0.02)

    ep_chassis.drive_speed(x=0, y=0, z=0)
    time.sleep(0.2)
    print("\n--- Side Adjustment Complete ---")
    
def move_straight_with_pid(ep_chassis, distance, max_speed=0.4):
    global robot_current_grid_pos
    print(f"--- PID Move: Moving {distance}m ---")
    EMERGENCY_STOP_DISTANCE_MM = 150 
    kp, ki, kd = 1.0, 0.5, 0.1
    last_error_dist, integral_dist = 0, 0
    start_pos = list(current_position)

    while program_running:
        dx = current_position[0] - start_pos[0]
        dy = current_position[1] - start_pos[1]
        distance_traveled = math.sqrt(dx**2 + dy**2)
        dist_error = distance - distance_traveled
        front_distance_mm = tof_distances[0]

        if front_distance_mm <= EMERGENCY_STOP_DISTANCE_MM:
            print(f"\n[!] EMERGENCY STOP! Obstacle detected at {front_distance_mm}mm. Stopping.")
            break
        if abs(dist_error) < 0.02:
            print("\nTarget distance reached.")
            break

        integral_dist += dist_error * 0.02
        derivative_dist = (dist_error - last_error_dist) / 0.02
        forward_speed = (kp * dist_error) + (ki * integral_dist) + (kd * derivative_dist)
        forward_speed = max(min(forward_speed, max_speed), -max_speed)
        last_error_dist = dist_error
        ep_chassis.drive_speed(x=forward_speed, y=0, z=0)
        print(f"\rDist: {distance_traveled:.2f}m, Speed: {forward_speed:.2f}, FrontTOF: {front_distance_mm}mm", end="")
        time.sleep(0.02)

    ep_chassis.drive_speed(x=0, y=0, z=0)
    time.sleep(0.5)
    print(f"--- PID Move Finished ---")
    
def turn_to_absolute_angle_pid(ep_chassis, target_absolute_angle, max_turn_speed=80):
    if initial_yaw is None: return
    print(f"--- PID Absolute Turn: Rotating to target angle {target_absolute_angle}° ---")
    kp, ki, kd = 3.6, 0.07, 0.5
    target_yaw = initial_yaw + target_absolute_angle
    if target_yaw > 180: target_yaw -= 360
    if target_yaw < -180: target_yaw += 360
    last_error_yaw, integral_yaw = 0, 0

    while program_running:
        current_yaw = current_attitude[0]
        error_yaw = target_yaw - current_yaw
        if error_yaw > 180: error_yaw -= 360
        if error_yaw < -180: error_yaw += 360
        if abs(error_yaw) < 1.0: break

        integral_yaw += error_yaw * 0.02
        derivative_yaw = (error_yaw - last_error_yaw) / 0.02
        turn_speed = (kp * error_yaw) + (ki * integral_yaw) + (kd * derivative_yaw)
        turn_speed = max(min(turn_speed, max_turn_speed), -max_turn_speed)
        last_error_yaw = error_yaw
        ep_chassis.drive_speed(x=0, y=0, z=turn_speed)
        print(f"\rCurrent Yaw: {current_yaw:.1f}°, Target: {target_yaw:.1f}°, Speed: {turn_speed:.1f}", end="")
        time.sleep(0.02)

    ep_chassis.drive_speed(x=0, y=0, z=0)
    print(f"\n--- PID Absolute Turn Complete ---")

def turn_and_recenter_gimbal(ep_chassis, ep_gimbal, target_angle):
    turn_to_absolute_angle_pid(ep_chassis, target_angle)
    with gimbal_lock:
        ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=200).wait_for_completed()

def get_absolute_directions(scan_data, robot_heading):
    if scan_data is None:
        print("[Warning] scan_data is None in get_absolute_directions. Assuming all walls are blocked.")
        return { 'N': True, 'S': True, 'E': True, 'W': True }
        
    heading_map = {
        0:   {'front': 'N', 'left': 'W', 'right': 'E'}, -90: {'front': 'E', 'left': 'N', 'right': 'S'},
        90:  {'front': 'W', 'left': 'S', 'right': 'N'}, 180: {'front': 'S', 'left': 'E', 'right': 'W'},
        -180:{'front': 'S', 'left': 'E', 'right': 'W'}
    }
    current_map = heading_map.get(robot_heading, {})
    walls = { 'N': True, 'S': True, 'E': True, 'W': True }

    if 'front' in current_map and scan_data.get('front', 0) > BLOCKED_DISTANCE_MM: walls[current_map['front']] = False
    if 'left' in current_map and scan_data.get('left', 0) > BLOCKED_DISTANCE_MM: walls[current_map['left']] = False
    if 'right' in current_map and scan_data.get('right', 0) > BLOCKED_DISTANCE_MM: walls[current_map['right']] = False
    return walls

def get_neighbor_coords(cx, cy, direction):
    if direction == 'N': return (cx, cy + 1)
    if direction == 'S': return (cx, cy - 1)
    if direction == 'E': return (cx + 1, cy)
    if direction == 'W': return (cx - 1, cy)
    return (cx, cy)

def get_heading_for_direction(direction):
    if direction == 'N': return 0
    if direction == 'S': return 180
    if direction == 'E': return -90
    if direction == 'W': return 90
    return 0

# ===================================================================
# SECTION 5: RUN MODE FUNCTIONS
# ===================================================================

def run_exploration_mode(ep_robot):
    global program_running, robot_current_grid_pos, robot_current_heading_deg
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    
    targets_found_at_coords = {}
    print("--- Starting Maze Exploration (DFS) ---")
    maze_map, visited, path_stack, scanned_cells = {}, set(), [], set()
    opposite_direction = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
    
    DFS_DIRECTION_PREFERENCE = ['N', 'W', 'E', 'S']

    current_pos = (0, 0)
    robot_current_grid_pos = current_pos
    visited.add(current_pos)
    path_stack.append(current_pos)
    current_heading = 0
    robot_current_heading_deg = current_heading
            
    while path_stack and program_running:
        current_pos = path_stack[-1]
        robot_current_grid_pos = current_pos
        robot_current_heading_deg = current_heading
        cx, cy = current_pos
        print(f"\n=================================================")
        print(f"[Main Thread] Position: {current_pos}, Facing: {current_heading} deg")
        
        robot_is_stationary.set()
        turn_and_recenter_gimbal(ep_chassis, ep_gimbal, current_heading)
        scanned_dist = scan_surroundings_and_targets(ep_gimbal, ep_chassis)
        robot_is_stationary.clear()
        
        if current_pos not in scanned_cells:
            print(f"--- First full scan at {current_pos}. Creating golden map record. ---")
            wall_data = get_absolute_directions(scanned_dist, current_heading)
            if current_pos in maze_map: maze_map[current_pos].update(wall_data)
            else: maze_map[current_pos] = wall_data
            scanned_cells.add(current_pos)
        else:
            print(f"--- Re-visiting {current_pos}. Map is locked. ---")

        possible_moves = []
        heading_to_scan_key = {
            0:   {'N': 'front', 'W': 'left', 'E': 'right'}, -90: {'E': 'front', 'N': 'left', 'S': 'right'},
            90:  {'W': 'front', 'S': 'left', 'N': 'right'}, 180: {'S': 'front', 'E': 'left', 'W': 'right'},
            -180:{'S': 'front', 'E': 'left', 'W': 'right'}
        }
        scan_map = heading_to_scan_key.get(current_heading, {})
        for direction, is_wall in maze_map.get(current_pos, {}).items():
            if not is_wall:
                neighbor_pos = get_neighbor_coords(cx, cy, direction)
                if neighbor_pos not in visited:
                    current_distance = scanned_dist.get(scan_map.get(direction), 0)
                    if current_distance > BLOCKED_DISTANCE_MM:
                        possible_moves.append({'direction': direction, 'distance': current_distance})
                    else:
                        print(f"[Safety Check] Path '{direction}' is currently blocked (Dist: {current_distance}mm). Ignoring.")
        
        next_move = None
        if possible_moves:
            possible_moves.sort(key=lambda x: DFS_DIRECTION_PREFERENCE.index(x['direction']))
            next_move = possible_moves[0]
            print(f"Possible moves (sorted by DFS pref): {[p['direction'] for p in possible_moves]} -> Chose '{next_move['direction']}'")

        if next_move:
            next_move_direction = next_move['direction']
            target_pos = get_neighbor_coords(cx, cy, next_move_direction)
            print(f"--> Found path: {next_move_direction}. Moving to {target_pos}")
            
            maze_map[current_pos][next_move_direction] = False
            if target_pos not in maze_map: maze_map[target_pos] = {}
            maze_map[target_pos][opposite_direction[next_move_direction]] = False
            print(f"Binding map: {current_pos} <--> {target_pos}")

            visited.add(target_pos)
            path_stack.append(target_pos)
            current_heading = get_heading_for_direction(next_move_direction)
            turn_and_recenter_gimbal(ep_chassis, ep_gimbal, current_heading)
            move_straight_with_pid(ep_chassis, GRID_DISTANCE)
            adjust_side_distance(ep_chassis, ep_sensor_adaptor)
        else:
            print(f"--- Dead end at {current_pos}. Backtracking... ---")
            path_stack.pop()
            if not path_stack: break
            target_pos = path_stack[-1]
            print(f"--> Backtracking to {target_pos}")
            dx, dy = target_pos[0] - cx, target_pos[1] - cy
            backtrack_direction = ''
            if dy == 1: backtrack_direction = 'N'
            elif dy == -1: backtrack_direction = 'S'
            elif dx == 1: backtrack_direction = 'E'
            elif dx == -1: backtrack_direction = 'W'
            
            current_heading = get_heading_for_direction(backtrack_direction)
            turn_and_recenter_gimbal(ep_chassis, ep_gimbal, current_heading)
            move_straight_with_pid(ep_chassis, GRID_DISTANCE)
            adjust_side_distance(ep_chassis, ep_sensor_adaptor)
    
    print("\n\n✅✅✅ --- DFS Exploration Complete! --- ✅✅✅")
    
    for target_id, pos, heading in shot_targets:
        if pos not in targets_found_at_coords:
            targets_found_at_coords[pos] = []
        if not any(t['id'] == target_id and t['heading'] == heading for t in targets_found_at_coords[pos]):
             targets_found_at_coords[pos].append({'id': target_id, 'heading': heading})


    print("Final Maze Map:")
    for pos in sorted(maze_map.keys()):
        print(f"  {pos}: {maze_map[pos]}")
    
    print("\nTargets Found and Shot in Run 1:")
    for pos, targets in targets_found_at_coords.items():
        print(f"  At position {pos}:")
        for target_info in targets:
            print(f"    - {target_info['id']} seen when facing {target_info['heading']} degrees")
    
    save_run_data(maze_map, targets_found_at_coords, DATA_FILENAME)

def run_optimal_path_mode(ep_robot):
    global shot_targets, last_fire_time, robot_current_grid_pos, program_running
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_sensor_adaptor = ep_robot.sensor_adaptor

    run_data = load_run_data(DATA_FILENAME)
    if not run_data: return

    maze_map = run_data['maze_map']
    targets_found = run_data['targets_found']

    print("\n--- Starting Run 2: Optimal Path Execution ---")
    print("Resetting robot for Run 2...")
    turn_and_recenter_gimbal(ep_chassis, ep_gimbal, 0)
    shot_targets.clear()
    last_fire_time = 0
    
    start_node = (0, 0)
    robot_current_grid_pos = start_node
    
    target_locations = list(targets_found.keys())

    if not target_locations:
        print("No targets found in Run 1 data. Skipping Run 2.")
        return

    print("\n--- Phase A: Pre-calculating all path distances using A* ---")
    points_of_interest = [start_node] + list(set(target_locations))
    path_distances = {}
    for a, b in itertools.combinations(points_of_interest, 2):
        path = a_star_search(maze_map, a, b)
        if path:
            distance = len(path) - 1
            path_distances[(a, b)] = distance
            path_distances[(b, a)] = distance
            print(f"Distance between {a} and {b} = {distance} steps")

    print("\n--- Phase B: Finding the best order to visit targets (TSP) ---")
    best_path_order = None
    min_total_distance = float('inf')
    
    unique_target_locations = list(set(target_locations))
    for order in itertools.permutations(unique_target_locations):
        current_sequence = [start_node] + list(order)
        current_total_distance = sum(path_distances.get((current_sequence[i], current_sequence[i+1]), float('inf')) for i in range(len(current_sequence) - 1))
        if current_total_distance < min_total_distance:
            min_total_distance = current_total_distance
            best_path_order = current_sequence
    
    if not best_path_order:
        print("[Error] Could not determine a valid path order. Aborting Run 2.")
        return

    print(f"\n✅ Best visit order found: {best_path_order}")
    print(f"✅ Minimum total distance: {min_total_distance} steps")

    print("\n--- Phase C: Executing the optimal path ---")
    for i in range(len(best_path_order) - 1):
        if not program_running: break
        start_segment, end_segment = best_path_order[i], best_path_order[i+1]
        print(f"\nMoving from {start_segment} to {end_segment}...")
        segment_path = a_star_search(maze_map, start_segment, end_segment)
        
        if not segment_path:
            print(f"[Error] Could not find a path from {start_segment} to {end_segment}. Skipping.")
            continue

        for j in range(len(segment_path) - 1):
            if not program_running: break
            current_step, next_step = segment_path[j], segment_path[j+1]
            robot_current_grid_pos = current_step
            
            dx, dy = next_step[0] - current_step[0], next_step[1] - current_step[1]
            move_direction = ''
            if dy == 1: move_direction = 'N'
            elif dy == -1: move_direction = 'S'
            elif dx == 1: move_direction = 'E'
            elif dx == -1: move_direction = 'W'
            
            target_heading = get_heading_for_direction(move_direction)
            print(f"Step: {current_step} -> {next_step} (Direction: {move_direction})")
            turn_and_recenter_gimbal(ep_chassis, ep_gimbal, target_heading)
            move_straight_with_pid(ep_chassis, GRID_DISTANCE)
            
            adjust_side_distance(ep_chassis, ep_sensor_adaptor)
        
        if not program_running: break
        robot_current_grid_pos = end_segment

        print(f"\nArrived at target location {end_segment}. Acquiring and firing all targets here...")
        targets_at_this_pos = targets_found.get(end_segment, [])
        for target_to_shoot in targets_at_this_pos:
            if not program_running: break
            
            target_id = tuple(target_to_shoot['id'])
            target_heading = target_to_shoot['heading']
            
            if any(t[0] == target_id for t in shot_targets):
                print(f"  - Target {target_id} already shot. Skipping.")
                continue

            print(f"  -> Preparing to shoot {target_id} by turning to {target_heading} degrees.")
            turn_and_recenter_gimbal(ep_chassis, ep_gimbal, target_heading)
            
            robot_is_stationary.set()
            print("  -> Robot stationary. Waiting for detection and firing...")
            time.sleep(5) 
            robot_is_stationary.clear()
            print(f"  -> Time up for target {target_id}.")

    print("\n\n🏆🏆🏆 --- Run 2 Complete! --- 🏆🏆🏆")

# ===================================================================
# SECTION 6: MAIN PROGRAM ENTRY POINT
# ===================================================================

if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in ['run1', 'run2']:
        print("\nUsage: python your_script_name.py [mode]")
        print("Modes:")
        print("  run1: Exploration and mapping (DFS)")
        print("  run2: Optimal path execution (A*) using saved data")
        sys.exit(1)

    mode = sys.argv[1]
    
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    ep_gimbal = ep_robot.gimbal
    ep_tof_sensor = ep_robot.sensor
    ep_tof_sensor.sub_distance(freq=20, callback=sub_tof_data_handler)
    ep_chassis.sub_position(freq=50, callback=position_handler)
    ep_chassis.sub_attitude(freq=50, callback=attitude_handler)

    ep_gimbal.recenter().wait_for_completed()

    print("Waiting for initial sensor data...")
    while not initial_yaw_set and program_running:
        time.sleep(0.1)
    
    if program_running:
        print("Initial data received! Starting program.")
        time.sleep(1)

    current_pos_getter = lambda: robot_current_grid_pos
    current_heading_getter = lambda: robot_current_heading_deg
    detection_thread = threading.Thread(
        target=detection_and_shooting_thread_func, 
        args=(ep_robot, current_pos_getter, current_heading_getter), 
        daemon=True
    )

    try:
        if program_running:
            detection_thread.start()

            if mode == 'run1':
                run_exploration_mode(ep_robot)
            elif mode == 'run2':
                run_optimal_path_mode(ep_robot)
    
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Stopping threads and robot...")
        program_running = False
        robot_is_stationary.set() 
        if detection_thread.is_alive():
            detection_thread.join(timeout=2)

        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=1)
        ep_tof_sensor.unsub_distance()
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        ep_robot.close()
        print("Disconnected successfully.")
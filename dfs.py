import robomaster
from robomaster import robot
import time
import math

# ===================================================================
# Global Variables & Callbacks
# ===================================================================
tof_distances = [0, 0, 0, 0]
current_position = [0, 0, 0]
current_attitude = [0, 0, 0]
initial_yaw = None
initial_yaw_set = False
is_scanning = True   # ✅ ควบคุมการอ่านค่า TOF

def sub_tof_data_handler(sub_info):
    global tof_distances, is_scanning
    if is_scanning:
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

# ===================================================================
# Configuration
# ===================================================================
YOUR_A_VALUE = 29.58
YOUR_B_VALUE = -1.53
NO_OBJECT_VOLTAGE_THRESHOLD = 1.0

MAX_ADC_VALUE = 1023.0
REFERENCE_VOLTAGE = 5.0

GRID_DISTANCE = 0.65
BLOCKED_DISTANCE_MM = 350
WALL_STOP_DISTANCE_MM = 150

TARGET_SIDE_DISTANCE_CM = 14    # 🎯 กลาง 13–15 cm
P_GAIN_SLIDE_ADJUST = 1.8
MAX_SLIDE_ADJUST_SPEED = 0.25
ADJUSTMENT_TIME_LIMIT_S = 3.0

# ===================================================================
# Helper Functions
# ===================================================================
def adc_to_cm(voltage):
    if voltage < NO_OBJECT_VOLTAGE_THRESHOLD: return float('inf')
    if voltage > 3.2: return 5.0
    try:
        return YOUR_A_VALUE * math.pow(voltage, YOUR_B_VALUE)
    except (ValueError, ZeroDivisionError):
        return float('inf')
    
def active_brake_sleep(ep_chassis, duration):
    """
    ฟังก์ชันที่ทำหน้าที่เหมือน time.sleep() แต่จะส่งคำสั่งเบรกล้อซ้ำๆ
    เพื่อให้หุ่นยนต์หยุดนิ่งสนิทระหว่างการหน่วงเวลา
    """
    start_time = time.time()
    while time.time() - start_time < duration:
        ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(0.05) # ส่งคำสั่งเบรกทุกๆ 0.05 วินาที เพื่อให้ล้อถูกล็อกไว้

# ✅ แก้ไข: เพิ่ม ep_chassis เข้ามาใน parameter ของฟังก์ชัน
def scan_surroundings(ep_gimbal, ep_chassis):
    """
    ใช้ TOF + gimbal scan หาทางเดิน พร้อมล็อกล้อระหว่างรอค่า sensor
    """
    global is_scanning
    is_scanning = True

    distances = {'left': 0, 'front': 0, 'right': 0}
    scan_speed = 300
    pause_duration = 0.7 # ระยะเวลาหน่วงเพื่อให้ค่าวัดนิ่ง และการเบรกทำงาน

    try:
        # --- 1. Scan Left ---
        print("--- Scanning Left...")
        ep_gimbal.moveto(pitch=0, yaw=90, yaw_speed=scan_speed).wait_for_completed()
        # ✅ แก้ไข: ใช้ active_brake_sleep แทน ep_chassis.move(x=0,y=0,z=0) และ time.sleep()
        active_brake_sleep(ep_chassis, pause_duration)
        distances['left'] = tof_distances[0]
        print(f"Scan Left result: {distances['left']} mm")

        # --- ✅ Reset to Center ---
        print("--- Resetting gimbal to center ---")
        ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=scan_speed).wait_for_completed()
        # ✅ แก้ไข: ใช้ active_brake_sleep แทน ep_chassis.move(x=0,y=0,z=0) และ time.sleep()
        active_brake_sleep(ep_chassis, 0.2) # หน่วงสั้นๆ ให้ gimbal นิ่งสนิท
        
        # --- 2. Scan Right ---
        print("--- Scanning Right...")
        ep_gimbal.moveto(pitch=0, yaw=-90, yaw_speed=scan_speed).wait_for_completed()
        # ✅ แก้ไข: ใช้ active_brake_sleep แทน ep_chassis.move(x=0,y=0,z=0) และ time.sleep()
        active_brake_sleep(ep_chassis, pause_duration)
        distances['right'] = tof_distances[0]
        print(f"Scan Right result: {distances['right']} mm")

        # --- ✅ Reset to Center ---
        print("--- Resetting gimbal to center ---")
        ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=scan_speed).wait_for_completed()
        # ✅ แก้ไข: ใช้ active_brake_sleep แทน ep_chassis.move(x=0,y=0,z=0) และ time.sleep()
        active_brake_sleep(ep_chassis, 0.2) # หน่วงสั้นๆ ให้ gimbal นิ่งสนิท

        # --- 3. Scan Front ---
        print("--- Scanning Front...")
        # ตอนนี้ Gimbal อยู่ตรงกลางแล้ว รออ่านค่าได้เลย
        # ✅ แก้ไข: ใช้ active_brake_sleep แทน time.sleep()
        active_brake_sleep(ep_chassis, pause_duration)
        distances['front'] = tof_distances[0]
        print(f"Scan Front result: {distances['front']} mm")
        
    finally:
        # รีเซ็ต gimbal กลับหน้าตรงเสมอ และส่งคำสั่งหยุดครั้งสุดท้ายเพื่อให้แน่ใจว่าล้อหยุดหมุน
        print("--- Scan routine complete. Gimbal is centered. ---")
        # ไม่จำเป็นต้อง moveto อีกรอบ เพราะ Gimbal อยู่ตรงกลางจากการสแกน Right และ reset ไปแล้ว
        ep_chassis.drive_speed(x=0, y=0, z=0) # ส่งคำสั่งหยุดครั้งสุดท้าย

    return distances

# ===================================================================
# Control Functions
# ===================================================================
# ===================================================================
# Control Functions
# ===================================================================

def adjust_side_distance(ep_chassis, ep_sensor_adaptor):
    """
    ปรับปรุงใหม่: แก้ไขตรรกะให้ทำงานกับ Sensor แบบ Active-Low (เจอผนังเป็น 0)
    """
    print(f"\n--- Adjusting Side Distance (Active-Low Logic) ---")
    start_time = time.time()
    
    # อ่านค่า IR เพื่อตรวจสอบเบื้องต้น
    ir_left_check = ep_sensor_adaptor.get_io(id=1, port=1)
    ir_right_check = ep_sensor_adaptor.get_io(id=2, port=1)
    
    # <--- แก้ไข: ถ้า IR ทั้งสองข้างเป็น 1 (ไม่เจอผนัง) ให้ออก
    if ir_left_check == 1 and ir_right_check == 1:
        print("No walls detected by IR → skipping adjustment.")
        print("--- Side Adjustment Complete ---")
        return

    while time.time() - start_time < ADJUSTMENT_TIME_LIMIT_S:
        left_raw_adc = ep_sensor_adaptor.get_adc(id=1, port=2)
        right_raw_adc = ep_sensor_adaptor.get_adc(id=2, port=2)
        ir_left = ep_sensor_adaptor.get_io(id=1, port=1)
        ir_right = ep_sensor_adaptor.get_io(id=2, port=1)

        left_dist_cm = adc_to_cm((left_raw_adc / MAX_ADC_VALUE) * REFERENCE_VOLTAGE)
        right_dist_cm = adc_to_cm((right_raw_adc / MAX_ADC_VALUE) * REFERENCE_VOLTAGE)
        
         # --- ✅ เพิ่ม Safety Net ป้องกันค่า inf ---
        if math.isinf(left_dist_cm) or math.isinf(right_dist_cm):
            print(f"\r[Safety] Infinite distance detected. Cancelling adjustment.", end="")
            break

        error_side = 0.0
        side_found = "None"

        # <--- แก้ไข: กลับเงื่อนไขทั้งหมด จาก 1 เป็น 0 และ 0 เป็น 1
        # กรณีเจอผนัง 2 ข้าง: จัดกึ่งกลาง
        if ir_left == 0 and ir_right == 0:
            error_side = (left_dist_cm - right_dist_cm) / 2.0
            side_found = 'Both walls'
        # กรณีเจอผนังซ้ายข้างเดียว
        elif ir_left == 0 and ir_right == 1:
            error_side = left_dist_cm - TARGET_SIDE_DISTANCE_CM
            side_found = 'Left wall only'
        # กรณีเจอผนังขวาข้างเดียว
        elif ir_right == 0 and ir_left == 1:
            error_side = TARGET_SIDE_DISTANCE_CM - right_dist_cm
            side_found = 'Right wall only'
        # กรณีไม่เจอผนังแล้ว
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
    print(f"--- PID Move: Moving {distance}m ---")
    
    # ✅ FIX: ใช้ค่าที่ต่ำลงมากสำหรับ "หยุดฉุกเฉิน" เท่านั้น
    # การตรวจจับกำแพงจริงๆ ควรทำตอนสแกน ไม่ใช่ตอนกำลังเดิน
    EMERGENCY_STOP_DISTANCE_MM = 150 

    kp, ki, kd = 1.0, 0.5, 0.1
    last_error_dist, integral_dist = 0, 0
    start_pos = list(current_position)

    while True:
        dx = current_position[0] - start_pos[0]
        dy = current_position[1] - start_pos[1]
        distance_traveled = math.sqrt(dx**2 + dy**2)
        dist_error = distance - distance_traveled

        front_distance_mm = tof_distances[0]
        # ✅ FIX: เปลี่ยนไปใช้ค่าสำหรับหยุดฉุกเฉิน
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
    if initial_yaw is None:
        print("[Error] Initial yaw not set. Cannot turn.")
        return

    print(f"--- PID Absolute Turn: Rotating to target angle {target_absolute_angle}° ---")

    kp, ki, kd = 3.6, 0.07, 0.5
    target_yaw = initial_yaw + target_absolute_angle
    if target_yaw > 180: target_yaw -= 360
    if target_yaw < -180: target_yaw += 360

    last_error_yaw, integral_yaw = 0, 0

    while True:
        current_yaw = current_attitude[0]
        error_yaw = target_yaw - current_yaw
        if error_yaw > 180: error_yaw -= 360
        if error_yaw < -180: error_yaw += 360

        if abs(error_yaw) < 1.0:
            break

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
    """
    หมุนหุ่นยนต์ไปยังองศาเป้าหมาย แล้วรีเซ็ต gimbal กลับมาด้านหน้าเสมอ
    """
    # 1. หมุนตัวหุ่นยนต์ (เรียกใช้ฟังก์ชันเดิม)
    turn_to_absolute_angle_pid(ep_chassis, target_angle)
    
    # 2. รีเซ็ต gimbal กลับมาที่ 0 (ด้านหน้าของหุ่น)
    print("--- Recenter gimbal after turn ---")
    ep_gimbal.moveto(pitch=0, yaw=0, yaw_speed=200).wait_for_completed()

# ===================================================================
# Main Program
# ===================================================================
# ===================================================================
# ⭐️⭐️⭐️  START: NEW HELPER FUNCTIONS FOR DFS ⭐️⭐️⭐️
# ===================================================================

def get_absolute_directions(scan_data, robot_heading):
    """
    แปลงผลการสแกน (ซ้าย, หน้า, ขวา) ให้เป็นทิศทางจริง (N, S, E, W)
    """
    # BLOCKED_DISTANCE_MM คือระยะที่ถือว่าเป็นกำแพง
    
    # Mapping: (Robot Heading) -> {'relative_dir': 'absolute_dir'}
    heading_map = {
        0:   {'front': 'N', 'left': 'W', 'right': 'E'},   # Facing North
        -90: {'front': 'E', 'left': 'N', 'right': 'S'},   # Facing East
        90:  {'front': 'W', 'left': 'S', 'right': 'N'},   # Facing West
        180: {'front': 'S', 'left': 'E', 'right': 'W'},   # Facing South
       -180: {'front': 'S', 'left': 'E', 'right': 'W'}    # Facing South
    }
    
    current_map = heading_map.get(robot_heading, {})
    walls = { 'N': True, 'S': True, 'E': True, 'W': True } # Assume all walls exist initially

    # Front
    if scan_data.get('front', 0) > BLOCKED_DISTANCE_MM:
        walls[current_map.get('front')] = False # No wall
    # Left
    if scan_data.get('left', 0) > BLOCKED_DISTANCE_MM:
        walls[current_map.get('left')] = False # No wall
    # Right
    if scan_data.get('right', 0) > BLOCKED_DISTANCE_MM:
        walls[current_map.get('right')] = False # No wall
        
    return walls


def get_neighbor_coords(cx, cy, direction):
    """
    คำนวณพิกัดของช่องถัดไปตามทิศทาง
    """
    if direction == 'N': return (cx, cy + 1)
    if direction == 'S': return (cx, cy - 1)
    if direction == 'E': return (cx + 1, cy)
    if direction == 'W': return (cx - 1, cy)
    return (cx, cy)

def get_heading_for_direction(direction):
    """
    แปลงทิศทาง (N,S,E,W) เป็นองศาที่หุ่นต้องหันไป
    """
    if direction == 'N': return 0
    if direction == 'S': return 180
    if direction == 'E': return -90
    if direction == 'W': return 90
    return 0

# ===================================================================
# ⭐️⭐️⭐️  END: NEW HELPER FUNCTIONS FOR DFS ⭐️⭐️⭐️
# ===================================================================


# ===================================================================
# Main Program (REFACTORED FOR DFS with Two-Way Map Binding)
# ===================================================================
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    # --- ส่วนประกาศตัวแปรเหมือนเดิม ---
    ep_chassis = ep_robot.chassis
    ep_sensor_adaptor = ep_robot.sensor_adaptor
    ep_gimbal = ep_robot.gimbal
    ep_tof_sensor = ep_robot.sensor

    ep_tof_sensor.sub_distance(freq=20, callback=sub_tof_data_handler)
    ep_chassis.sub_position(freq=50, callback=position_handler)
    ep_chassis.sub_attitude(freq=50, callback=attitude_handler)

    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=100, yaw_speed=100).wait_for_completed()

    print("Waiting for sensor data...")
    while not initial_yaw_set:
        time.sleep(0.1)
    print("Initial data received!")
    time.sleep(1)

    # =================================================================
    # ✅✅✅ START: DFS EXPLORATION LOGIC ✅✅✅
    # =================================================================
    print("--- Starting Maze Exploration (DFS) ---")
    try:
            # --- 1. DFS Data Structures ---
        maze_map = {}
        visited = set()
        path_stack = []
        scanned_cells = set() # ✅ ตัวแปรใหม่สำหรับติดตามช่องที่สแกนเต็มรูปแบบแล้ว
        opposite_direction = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}

            # --- 2. Initial State ---
        cx, cy = 0, 0
        current_heading = 0
        current_pos = (cx, cy)
        visited.add(current_pos)
        path_stack.append(current_pos)
            
            # --- 3. Main DFS Loop ---
        while path_stack:
            current_pos = path_stack[-1]
            cx, cy = current_pos
                
            print(f"\n=================================================")
            print(f"Current Position: {current_pos}, Facing: {current_heading} deg, Stack Size: {len(path_stack)}")
                
            turn_and_recenter_gimbal(ep_chassis, ep_gimbal, current_heading)
                
            print("--- Scanning surroundings ---")
            scanned_dist = scan_surroundings(ep_gimbal, ep_chassis)
                
                # --- 4. Update Map (Corrected Golden Record Logic) ---
                # ✅✅✅ แก้ไขเงื่อนไข: ใช้ scanned_cells ในการตัดสินใจ ✅✅✅
            if current_pos not in scanned_cells:
                print(f"--- First full scan at {current_pos}. Creating golden map record. ---")
                wall_data = get_absolute_directions(scanned_dist, current_heading)
                    
                    # ถ้ามีข้อมูลเก่าอยู่แล้ว (จาก binding) ให้อัปเดตข้อมูลใหม่เข้าไป
                if current_pos in maze_map:
                    maze_map[current_pos].update(wall_data)
                else:
                    maze_map[current_pos] = wall_data
                    
                scanned_cells.add(current_pos) # บันทึกว่าช่องนี้ถูกสแกนสมบูรณ์แล้ว
            else:
                print(f"--- Re-visiting {current_pos}. Map is locked. Using fresh scan only for this turn's decision. ---")

            print(f"Using map for {current_pos}: {maze_map.get(current_pos)}")
                        
# --- 5. Find BEST Unvisited Neighbor (เหมือนเดิม) ---
            # (โค้ดส่วนนี้ทั้งหมดเหมือนเดิม ไม่ต้องแก้ไข)
            possible_moves = []
            heading_to_scan_key = {
                0:   {'N': 'front', 'W': 'left', 'E': 'right'}, -90: {'E': 'front', 'N': 'left', 'S': 'right'},
                90:  {'W': 'front', 'S': 'left', 'N': 'right'}, 180: {'S': 'front', 'E': 'left', 'W': 'right'},
               -180: {'S': 'front', 'E': 'left', 'W': 'right'}
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
                            print(f"[Safety Check] Path '{direction}' is currently blocked (Dist: {current_distance}mm). Ignoring this path for now.")
            
            next_move = None
            if possible_moves:
                possible_moves.sort(key=lambda x: x['distance'], reverse=True)
                next_move = possible_moves[0]
                print(f"Possible moves: {possible_moves} -> Chose '{next_move['direction']}'")

            # --- 6. Decide Action (เหมือนเดิม) ---
            # (โค้ดส่วนนี้ทั้งหมดเหมือนเดิม ไม่ต้องแก้ไข)
            if next_move:
                # MOVE FORWARD
                next_move_direction = next_move['direction']
                target_pos = get_neighbor_coords(cx, cy, next_move_direction)
                print(f"--> Found best path: {next_move_direction}. Moving to {target_pos}")
                
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
                # BACKTRACK
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
        print("Final Maze Map:")
        for pos in sorted(maze_map.keys()):
            print(f"  {pos}: {maze_map[pos]}")
            
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Stopping robot and disconnecting...")
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=1)
        ep_tof_sensor.unsub_distance()
        ep_chassis.unsub_position()
        ep_chassis.unsub_attitude()
        ep_robot.close()
        print("Disconnected successfully.")
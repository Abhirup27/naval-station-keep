import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
import math

def naval_to_math_angle(naval_angle):
    """Convert naval navigation angle (clockwise from North) 
    to mathematical angle (counterclockwise from East)"""
    return (90 - naval_angle) % 360

def math_to_naval_angle(math_angle):
    """Convert mathematical angle (counterclockwise from East) 
    to naval navigation angle (clockwise from North)"""
    return (90 - math_angle) % 360

def polar_to_cartesian(range_nm, bearing):
    """Convert polar coordinates (range and bearing) to Cartesian coordinates
    bearing: naval navigation bearing (000° = North, clockwise positive)"""
    math_angle = naval_to_math_angle(bearing)
    x = range_nm * np.cos(np.radians(math_angle))
    y = range_nm * np.sin(np.radians(math_angle))
    return x, y


def calculate_course_and_time(ship1_pos, guide_ship_pos, guide_ship_course, guide_ship_speed, desired_offset, ship1_speed):
    # Convert ship1 and guide ship positions from polar to Cartesian coordinates
    ship1_x = ship1_pos[0] * math.cos(math.radians(ship1_pos[1]))
    ship1_y = ship1_pos[0] * math.sin(math.radians(ship1_pos[1]))
    guide_x = guide_ship_pos[0] * math.cos(math.radians(guide_ship_pos[1]))
    guide_y = guide_ship_pos[0] * math.sin(math.radians(guide_ship_pos[1]))
    
    # Calculate the desired position relative to the guide ship's current position
    desired_x = guide_x + desired_offset[0] * math.cos(math.radians(desired_offset[1]))# + guide_ship_course))
    desired_y = guide_y + desired_offset[0] * math.sin(math.radians(desired_offset[1])) #+ guide_ship_course))
    
    guide_math_course = naval_to_math_angle(guide_ship_course)
   

    # Calculate the relative position vector from ship1 to the desired position
    vector_ship1_desired = (desired_x - ship1_x, desired_y - ship1_y)
    distance_to_target = math.sqrt(vector_ship1_desired[0]**2 + vector_ship1_desired[1]**2)
    
    # Calculate the guide ship's velocity components
    guide_velocity_x = guide_ship_speed * math.cos(math.radians(guide_ship_course))
    guide_velocity_y = guide_ship_speed * math.sin(math.radians(guide_ship_course))
    
    # Calculate required velocity components for ship1 to reach the moving target
    relative_velocity_x = (vector_ship1_desired[0] / distance_to_target) * ship1_speed
    relative_velocity_y = (vector_ship1_desired[1] / distance_to_target) * ship1_speed
    
    # Adjust relative velocity by adding the guide ship's velocity
    total_velocity_x = relative_velocity_x + guide_velocity_x
    total_velocity_y = relative_velocity_y + guide_velocity_y

    
    
    # Calculate the required course and speed for ship1
    new_course = (math.degrees(math.atan2(total_velocity_y, total_velocity_x)) + 360) % 360
    new_speed = math.sqrt(total_velocity_x**2 + total_velocity_y**2)
    
    # Calculate time required to reach the target
    time_required_hours = distance_to_target / ship1_speed if ship1_speed > 0 else 0
    time_required_minutes = time_required_hours * 60
    
    guide_final_x = guide_x + guide_speed * time_required_hours * np.cos(np.radians(guide_math_course))
    guide_final_y = guide_y + guide_speed * time_required_hours * np.sin(np.radians(guide_math_course))
    math_bearing_final = math.degrees(math.atan2(guide_final_y -desired_y , guide_final_x-desired_x ))
    final_bearing = math_to_naval_angle(math_bearing_final)

    # Calculate true bearing to guide ship
    vector_ship1_guide = (guide_x - ship1_x, guide_y - ship1_y)
    true_bearing_to_guide_ship = math.degrees(math.atan2(vector_ship1_guide[1], vector_ship1_guide[0]))
    true_bearing_to_guide_ship = (true_bearing_to_guide_ship + 360) % 360


    
    return new_course, new_speed, time_required_minutes, true_bearing_to_guide_ship, math_bearing_final

def calculate_realistic_path(start_x, start_y, end_x, end_y, start_course, final_course, num_points=100):
    """
    Calculate a more realistic path considering:
    1. Initial course constraint
    2. Final approach course
    3. Gradual course change
    """
    # Convert naval angles to mathematical angles for calculations
    start_math_course = naval_to_math_angle(start_course)
    final_math_course = naval_to_math_angle(final_course)
    
    # Calculate total distance
    total_distance = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
    
    # Convert courses to radians
    start_course_rad = np.radians(start_math_course)
    final_course_rad = np.radians(final_math_course)
    
    # Calculate path points
    t = np.linspace(0, 1, num_points)
    
    def course_profile(t):
        return 1 / (1 + np.exp(-12 * (t - 0.5)))
    
    # Calculate intermediate courses (in mathematical angles)
    course_changes = start_course_rad + (final_course_rad - start_course_rad) * course_profile(t)
    
    x = np.zeros(num_points)
    y = np.zeros(num_points)
    x[0] = start_x
    y[0] = start_y
    
    step_size = total_distance / (num_points - 1)
    
    for i in range(1, num_points):
        dx = step_size * np.cos(course_changes[i])
        dy = step_size * np.sin(course_changes[i])
        x[i] = x[i-1] + dx
        y[i] = y[i-1] + dy
    
    # Apply correction to reach exact destination
    correction_x = (end_x - x[-1]) / num_points
    correction_y = (end_y - y[-1]) / num_points
    
    for i in range(1, num_points):
        x[i] += correction_x * i
        y[i] += correction_y * i
    
    return x, y


    # Calculate curved path for Ship 1
def create_ship_visualization(ship1_pos, guide_ship_pos, guide_course, guide_speed, 
                            desired_offset, ship1_speed, new_course, time_required_minutes):
    # Calculate positions using naval navigation angles
    ship1_x, ship1_y = polar_to_cartesian(ship1_pos[0], ship1_pos[1])
    guide_x, guide_y = polar_to_cartesian(guide_ship_pos[0], guide_ship_pos[1])
    
    # Calculate final guide ship position using mathematical angle
    guide_math_course = naval_to_math_angle(guide_course)
    time_hours = time_required_minutes / 60
    guide_final_x = guide_x + guide_speed * time_hours * np.cos(np.radians(guide_math_course))
    guide_final_y = guide_y + guide_speed * time_hours * np.sin(np.radians(guide_math_course))
    
    # Calculate desired final position
    offset_math_angle = naval_to_math_angle(desired_offset[1])# + guide_course)
    desired_x = guide_final_x + desired_offset[0] * np.cos(np.radians(offset_math_angle))
    desired_y = guide_final_y + desired_offset[0] * np.sin(np.radians(offset_math_angle))
    
    # Calculate initial true bearing
    math_bearing = math.degrees(math.atan2(guide_y - ship1_y, guide_x - ship1_x))
    initial_bearing = math_to_naval_angle(math_bearing)
    
    math_bearing_final = math.degrees(math.atan2(guide_final_y -desired_y , guide_final_x-desired_x ))
    final_bearing = math_to_naval_angle(math_bearing_final)

    # Calculate path
    ship1_path_x, ship1_path_y = calculate_realistic_path(
        ship1_x, ship1_y, desired_x, desired_y, 
        initial_bearing, new_course)
    
    # Create visualization
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Calculate bearing to guide ship
    bearing_to_guide = math_to_naval_angle(
        math.degrees(math.atan2(guide_y - ship1_y, guide_x - ship1_x)))
    
    # Calculate speed
    distance = math.sqrt((desired_x - ship1_x)**2 + (desired_y - ship1_y)**2)
    required_speed = distance / (time_required_minutes / 60)
    
    # Create info box
    info_text = (
        f"Course to Steer: {new_course:.1f}°\n"
        f"Speed Required: {required_speed:.1f} knots\n"
        f"Time to Position: {time_required_minutes:.1f} minutes\n"
        f"Inital True Bearing to Guide: {bearing_to_guide:.1f}°\n"
        f"Final True Bearing to Guide: {final_bearing:.1f}°"
    )
    
    # Add info box
    props = dict(boxstyle='round', facecolor='white', alpha=0.8)
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)
    
    # Plot elements
    ax.plot(0, 0, 'k+', markersize=15, label='Fleet Center')
    ax.plot(ship1_x, ship1_y, 'bo', label='Ship 1 Initial Position')
    ax.plot(guide_x, guide_y, 'go', label='Guide Ship Initial Position')
    ax.plot(guide_final_x, guide_final_y, 'g^', label='Guide Ship Final Position')
    ax.plot(desired_x, desired_y, 'r^', label='Desired Position')
    
    # Plot paths
    guide_path_x = np.linspace(guide_x, guide_final_x, 100)
    guide_path_y = np.linspace(guide_y, guide_final_y, 100)
    ax.plot(guide_path_x, guide_path_y, 'g--', alpha=0.5, label='Guide Ship Path')
    ax.plot(ship1_path_x, ship1_path_y, 'b--', alpha=0.5, label='Ship 1 Path')
    
    # Draw course vectors using naval angles converted to mathematical angles
    course_vector_length = 2
    new_course_math = naval_to_math_angle(new_course)
    guide_course_math = naval_to_math_angle(guide_course)
    
    ax.arrow(ship1_x, ship1_y, 
             course_vector_length * np.cos(np.radians(new_course_math)),
             course_vector_length * np.sin(np.radians(new_course_math)),
             head_width=0.1, head_length=0.15, fc='blue', ec='blue')
    
    ax.arrow(guide_x, guide_y,
             course_vector_length * np.cos(np.radians(guide_course_math)),
             course_vector_length * np.sin(np.radians(guide_course_math)),
             head_width=0.1, head_length=0.15, fc='green', ec='green')
    
    # Add range circles and set plot parameters
    max_range = max(ship1_pos[0], guide_ship_pos[0], 
                   np.sqrt(desired_x**2 + desired_y**2)) * 1.2
    circle = Circle((0, 0), max_range, fill=False, linestyle=':', color='gray')
    ax.add_patch(circle)
    
    # Add compass rose indicators
    ax.text(max_range*0.9, 0, 'E', ha='center', va='center')
    ax.text(-max_range*0.9, 0, 'W', ha='center', va='center')
    ax.text(0, max_range*0.9, 'N', ha='center', va='center')
    ax.text(0, -max_range*0.9, 'S', ha='center', va='center')
    
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend(loc='lower right')
    ax.set_xlabel('Distance East/West (NM)')
    ax.set_ylabel('Distance North/South (NM)')
    ax.set_title('Ship Movement Visualization with Navigation Data')
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    
    return fig, ax, (ship1_x, ship1_y, desired_x, desired_y, guide_x, guide_y, 
                    guide_final_x, guide_final_y, ship1_path_x, ship1_path_y)

def create_animation(ship1_pos, guide_ship_pos, guide_course, guide_speed, 
                    desired_offset, ship1_speed, new_course, time_required_minutes):
    # Create visualization and get positions
    fig, ax, positions = create_ship_visualization(
        ship1_pos, guide_ship_pos, guide_course, guide_speed, 
        desired_offset, ship1_speed, new_course, time_required_minutes
    )
    
    (ship1_x, ship1_y, desired_x, desired_y, guide_x, guide_y, 
     guide_final_x, guide_final_y, ship1_path_x, ship1_path_y) = positions
    
    # Create animation elements
    ship1_dot, = ax.plot([], [], 'bo', markersize=10)
    guide_dot, = ax.plot([], [], 'go', markersize=10)
    time_text = ax.text(0.75, 0.98, '', transform=ax.transAxes,
                       bbox=dict(facecolor='white', alpha=0.8))
    
    def init():
        ship1_dot.set_data([ship1_x], [ship1_y])
        guide_dot.set_data([guide_x], [guide_y])
        time_text.set_text('Elapsed Time: 0.0 minutes')
        return ship1_dot, guide_dot, time_text
    
    def animate(frame):
        # Calculate current frame index
        t = frame / 100
        
        # Guide ship position (linear interpolation)
        current_guide_x = guide_x + (guide_final_x - guide_x) * t
        current_guide_y = guide_y + (guide_final_y - guide_y) * t
        
        # Ship 1 position (follow curved path)
        frame_index = int(t * (len(ship1_path_x) - 1))
        current_ship1_x = ship1_path_x[frame_index]
        current_ship1_y = ship1_path_y[frame_index]
        
        # Update positions
        ship1_dot.set_data([current_ship1_x], [current_ship1_y])
        guide_dot.set_data([current_guide_x], [current_guide_y])
        
        # Update time text
        current_time = time_required_minutes * t
        time_text.set_text(f'Elapsed Time: {current_time:.1f} minutes')
        
        return ship1_dot, guide_dot, time_text
    
    anim = FuncAnimation(fig, animate, init_func=init, frames=101,
                        interval=50, blit=True)
    
    return anim


if __name__ == "__main__":
      
    ship1_pos = (3, 270)  # (distance, bearing)
    guide_ship_pos = (0.0, 0)  # (distance, bearing)
    guide_course = 0  # degrees
    guide_speed = 13  # knots
    desired_offset = (0.15, 180)  # from the guide (distance, bearing)
    ship1_speed = 15  

    # Calculate the results
    new_course, new_speed, time_required_minutes, true_bearing, final_bearing = calculate_course_and_time(
        ship1_pos, 
        guide_ship_pos, 
        guide_course, 
        guide_speed, 
        desired_offset, 
        ship1_speed
    )
    anim = create_animation(
        ship1_pos, guide_ship_pos, guide_course, guide_speed,
        desired_offset, ship1_speed, new_course, time_required_minutes
    )
    plt.show()


    print(f"""
Calculation Results:
-------------------
Course to Steer: {new_course:.1f}°
Speed Required: {new_speed:.1f} knots
Time to Position: {time_required_minutes:.1f} minutes
Initial True Bearing to Guide Ship: {true_bearing:.1f}°
Final True Bearing to Guide Ship: {final_bearing:.1f}°

Input Values Used:
-----------------
Ship 1 Position: {ship1_pos[0]} NM at {ship1_pos[1]}°
Guide Ship Position: {guide_ship_pos[0]} NM at {guide_ship_pos[1]}°
Guide Ship Course: {guide_course}°
Guide Ship Speed: {guide_speed} knots
Desired Offset: {desired_offset[0]} NM at {desired_offset[1]}°
Ship 1 Speed: {ship1_speed} knots
""")
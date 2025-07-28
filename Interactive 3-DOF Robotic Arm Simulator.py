import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

"""
Interactive 3-DOF Robotic Arm Simulator - Standalone Version

This program provides a visual simulation of a 3-link robotic arm with
interactive controls for each joint angle. The arm is composed of three
segments of equal length, and the user can manipulate each joint independently.

Features:
- Real-time visualization of arm configuration
- Interactive sliders integrated in the plot window
- Display of end-effector position and workspace distance
- Grid visualization for spatial reference
- Works in any Python environment (not just Jupyter)

Arm specifications:
- 3 degrees of freedom (DOF)
- Equal link lengths: 12 cm each
- Base fixed at origin O(0,0)
- Total reach: 36 cm (fully extended)
"""

#===============================================================================
# ARM CONFIGURATION - MODIFY THESE VALUES AS NEEDED
#===============================================================================

# Base position (coordinate system origin)
x0 = 0   # Base X-coordinate (cm) - Change this to move the base
y0 = 0   # Base Y-coordinate (cm) - Change this to move the base

# Link lengths (modify these for different arm configurations)
l1 = 12  # First link length (cm)
l2 = 12  # Second link length (cm) 
l3 = 12  # Third link length (cm)

# Calculate derived parameters
total_reach = l1 + l2 + l3  # Maximum possible reach
plot_limit = total_reach + 10  # Plot boundaries

#===============================================================================
# FORWARD KINEMATICS CALCULATION
#===============================================================================

def calculate_arm_position(alpha, beta, gamma):
    """
    Calculate forward kinematics for the robotic arm.
    
    Parameters:
    -----------
    alpha : float
        First joint angle (θ1) in degrees
    beta : float  
        Second joint angle (θ2) in degrees  
    gamma : float
        Third joint angle (θ3) in degrees
        
    Returns:
    --------
    tuple: (arm_x, arm_y, end_effector_angle, distance_to_end)
        arm_x, arm_y: lists of x,y coordinates for each joint
        end_effector_angle: final orientation in degrees
        distance_to_end: distance from base to end-effector
    """
    
    # Convert angles to radians for trigonometric calculations
    alpha_rad = math.radians(alpha)
    beta_rad = math.radians(beta)
    gamma_rad = math.radians(gamma)
    
    # Joint 1 position (end of first link)
    x1 = x0 + l1 * math.cos(alpha_rad)
    y1 = y0 + l1 * math.sin(alpha_rad)
    
    # Joint 2 position (end of second link)
    # Angle accumulates: first link angle + second link angle
    x2 = x1 + l2 * math.cos(alpha_rad + beta_rad)
    y2 = y1 + l2 * math.sin(alpha_rad + beta_rad)
    
    # End-effector position (end of third link)
    # Angle accumulates: sum of all joint angles
    x3 = x2 + l3 * math.cos(alpha_rad + beta_rad + gamma_rad)
    y3 = y2 + l3 * math.sin(alpha_rad + beta_rad + gamma_rad)
    
    # Calculate distance from base to end-effector
    distance_to_end = math.sqrt((x0 - x3)**2 + (y0 - y3)**2)
    
    # Final orientation angle of the end-effector
    end_effector_angle = alpha + beta + gamma
    
    # Return coordinates and metrics
    arm_x = [x0, x1, x2, x3]
    arm_y = [y0, y1, y2, y3]
    
    return arm_x, arm_y, end_effector_angle, distance_to_end

#===============================================================================
# INTERACTIVE VISUALIZATION
#===============================================================================

def create_interactive_arm():
    """
    Create the interactive robotic arm simulator with matplotlib sliders.
    """
    
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(12, 8))
    plt.subplots_adjust(bottom=0.25)  # Make room for sliders
    
    # Initial joint angles
    alpha_init = 0
    beta_init = 0
    gamma_init = 0
    
    # Calculate initial arm position
    arm_x, arm_y, end_angle, distance = calculate_arm_position(alpha_init, beta_init, gamma_init)
    
    # Create initial plot
    line, = ax.plot(arm_x, arm_y, '-o', 
                   linewidth=3, 
                   markersize=8, 
                   color='royalblue',
                   label='Arm links')
    
    # Base marker
    base_point, = ax.plot(x0, y0, 'o', 
                         markersize=12, 
                         color='red',
                         label='Base')
    
    # End-effector marker (highlighted)
    end_point, = ax.plot(arm_x[-1], arm_y[-1], 'o',
                        markersize=10,
                        color='green',
                        label='End-effector')
    
    # Configure plot with dynamic limits based on arm reach
    ax.set_xlim(x0 - plot_limit, x0 + plot_limit)
    ax.set_ylim(y0 - plot_limit, y0 + plot_limit)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position (cm)')
    ax.set_ylabel('Y Position (cm)')
    ax.legend()
    
    # Title (will be updated)
    title = ax.set_title(f"3-DOF Robotic Arm Simulator\n"
                        f"End-effector: ({arm_x[-1]:.2f}, {arm_y[-1]:.2f}) cm\n"
                        f"Distance: {distance:.2f} cm | Orientation: {end_angle:.1f}°")
    
    #===========================================================================
    # CREATE SLIDERS
    #===========================================================================
    
    # Define slider positions [left, bottom, width, height]
    slider_height = 0.03
    slider_spacing = 0.05
    
    ax_alpha = plt.axes([0.15, 0.15, 0.5, slider_height])
    ax_beta = plt.axes([0.15, 0.10, 0.5, slider_height])
    ax_gamma = plt.axes([0.15, 0.05, 0.5, slider_height])
    
    # Create sliders
    slider_alpha = Slider(ax_alpha, 'α (Joint 1)', -180, 180, 
                         valinit=alpha_init, valfmt='%.0f°')
    slider_beta = Slider(ax_beta, 'β (Joint 2)', -180, 180, 
                        valinit=beta_init, valfmt='%.0f°')
    slider_gamma = Slider(ax_gamma, 'γ (Joint 3)', -180, 180, 
                         valinit=gamma_init, valfmt='%.0f°')
    
    #===========================================================================
    # UPDATE FUNCTION
    #===========================================================================
    
    def update_arm(val):
        """
        Update the arm visualization when sliders change.
        """
        # Get current slider values
        alpha = slider_alpha.val
        beta = slider_beta.val
        gamma = slider_gamma.val
        
        # Calculate new arm position
        arm_x, arm_y, end_angle, distance = calculate_arm_position(alpha, beta, gamma)
        
        # Update arm line
        line.set_xdata(arm_x)
        line.set_ydata(arm_y)
        
        # Update end-effector marker
        end_point.set_xdata([arm_x[-1]])
        end_point.set_ydata([arm_y[-1]])
        
        # Update title with current values
        title.set_text(f"3-DOF Robotic Arm Simulator\n"
                      f"End-effector: ({arm_x[-1]:.2f}, {arm_y[-1]:.2f}) cm\n"
                      f"Distance: {distance:.2f} cm | Orientation: {end_angle:.1f}°")
        
        # Redraw the plot
        fig.canvas.draw_idle()
    
    # Connect sliders to update function
    slider_alpha.on_changed(update_arm)
    slider_beta.on_changed(update_arm)
    slider_gamma.on_changed(update_arm)
    
    #===========================================================================
    # ADD RESET BUTTON
    #===========================================================================
    
    from matplotlib.widgets import Button
    
    ax_reset = plt.axes([0.75, 0.15, 0.15, 0.08])
    button_reset = Button(ax_reset, 'Reset Arm')
    
    def reset_arm(event):
        """Reset all sliders to zero position."""
        slider_alpha.reset()
        slider_beta.reset()
        slider_gamma.reset()
    
    button_reset.on_clicked(reset_arm)
    
    #===========================================================================
    # DISPLAY INSTRUCTIONS
    #===========================================================================
    
    # Add text with instructions
    instructions_text = (
        "Instructions:\n"
        "• Use sliders to control joint angles\n"
        "• α: First joint angle (-180° to +180°)\n"
        "• β: Second joint angle (-180° to +180°)\n"
        "• γ: Third joint angle (-180° to +180°)\n"
        f"• Maximum reach: {total_reach} cm"
    )
    
    plt.figtext(0.75, 0.45, instructions_text, 
                fontsize=10, 
                bbox=dict(boxstyle="round,pad=0.3", 
                         facecolor="lightgray", 
                         alpha=0.8))
    
    # Show the interactive plot
    plt.show()

#===============================================================================
# MAIN EXECUTION
#===============================================================================

if __name__ == "__main__":
    print("Starting 3-DOF Robotic Arm Simulator...")
    print("The interactive window will open shortly.")
    print("Close the window to exit the program.")
    
    # Create and display the interactive arm simulator
    create_interactive_arm()
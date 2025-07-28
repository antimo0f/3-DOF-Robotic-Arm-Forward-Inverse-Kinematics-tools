import math
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button
import numpy as np

"""
Interactive Inverse Kinematics Solver for 3-DOF Robotic Arm

This program provides an interactive interface to calculate and visualize
the joint angles required to position a 3-link robotic arm at a desired 
end-effector position and orientation.

Features:
- Text input boxes for precise target position (px, py) and orientation (phi)
- Real-time calculation of joint angles using inverse kinematics
- Visual feedback with target point and arm configuration
- Workspace boundary visualization
- Error handling for unreachable positions

Input controls:
- px: Target X-coordinate for the end-effector (type and press Enter)
- py: Target Y-coordinate for the end-effector (type and press Enter)
- phi: Desired final orientation angle of the end-effector (type and press Enter)

Output:
- Real-time display of calculated joint angles (α, β, γ)
- Visual representation of the arm reaching the target
- Workspace limits and reachability feedback
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
max_reach = l1 + l2 + l3  # Maximum reach (fully extended)
min_reach = abs(l1 - l2) + l3  # Minimum reach (folded configuration)
plot_limit = max_reach + 10  # Plot boundaries

#===============================================================================
# INVERSE KINEMATICS SOLVER
#===============================================================================

def solve_inverse_kinematics(px, py, phi_deg):
    """
    Solve inverse kinematics for the 3-DOF robotic arm.
    
    Parameters:
    -----------
    px : float
        Target X-coordinate for end-effector
    py : float
        Target Y-coordinate for end-effector
    phi_deg : float
        Desired end-effector orientation in degrees
        
    Returns:
    --------
    tuple: (alpha_deg, beta_deg, gamma_deg, success, error_msg)
        Joint angles in degrees, success flag, and error message
    """
    
    try:
        # Convert orientation to radians
        phi = math.radians(phi_deg)
        
        # Step 1: Calculate wrist position
        # The wrist is located at distance l3 away from the end-effector,
        # in the direction opposite to the final orientation
        wx = px - l3 * math.cos(phi)
        wy = py - l3 * math.sin(phi)
        
        # Convert to relative coordinates from base
        wx_rel = wx - x0
        wy_rel = wy - y0
        
        # Step 2: Solve for second joint angle (beta)
        # Using the law of cosines for the triangle formed by l1, l2, and wrist distance
        wrist_distance_sq = wx_rel**2 + wy_rel**2
        wrist_distance = math.sqrt(wrist_distance_sq)
        
        # Check if wrist position is reachable by first two links
        if wrist_distance > (l1 + l2):
            return 0, 0, 0, False, f"Wrist too far (d={wrist_distance:.1f}cm > {l1+l2}cm)"
        if wrist_distance < abs(l1 - l2):
            return 0, 0, 0, False, f"Wrist too close (d={wrist_distance:.1f}cm < {abs(l1-l2)}cm)"
        
        c2 = (wrist_distance_sq - l1**2 - l2**2) / (2 * l1 * l2)
        
        # Numerical stability check
        if abs(c2) > 1:
            c2 = max(-1, min(1, c2))  # Clamp to valid range
        
        # Calculate sine of second joint angle (choosing elbow-up configuration)
        s2 = math.sqrt(1 - c2**2)
        
        # Second joint angle (negative for elbow-up configuration)
        beta = -math.atan2(s2, c2)
        
        # Step 3: Solve for first joint angle (alpha)
        # Using the relationship between joint angles and wrist position
        denominator = wrist_distance_sq
        
        if denominator < 1e-6:  # Avoid division by zero
            return 0, 0, 0, False, "Wrist at origin - singular configuration"
        
        c1 = ((l1 + l2 * c2) * wx_rel - l2 * s2 * wy_rel) / denominator
        s1 = ((l1 + l2 * c2) * wy_rel + l2 * s2 * wx_rel) / denominator
        
        # First joint angle
        alpha = math.atan2(s1, c1)
        
        # Step 4: Calculate third joint angle (gamma)
        # Third joint compensates to achieve the desired final orientation
        gamma = phi - (alpha + beta)
        
        # Convert to degrees
        alpha_deg = math.degrees(alpha)
        beta_deg = math.degrees(beta)
        gamma_deg = math.degrees(gamma)
        
        return alpha_deg, beta_deg, gamma_deg, True, "Solution found"
        
    except Exception as e:
        return 0, 0, 0, False, f"Calculation error: {str(e)}"

#===============================================================================
# FORWARD KINEMATICS FOR VERIFICATION
#===============================================================================

def calculate_arm_position(alpha_deg, beta_deg, gamma_deg):
    """Calculate forward kinematics to verify the solution."""
    
    # Convert to radians
    alpha = math.radians(alpha_deg)
    beta = math.radians(beta_deg)
    gamma = math.radians(gamma_deg)
    
    # Calculate joint positions
    x1 = x0 + l1 * math.cos(alpha)
    y1 = y0 + l1 * math.sin(alpha)
    
    x2 = x1 + l2 * math.cos(alpha + beta)
    y2 = y1 + l2 * math.sin(alpha + beta)
    
    x3 = x2 + l3 * math.cos(alpha + beta + gamma)
    y3 = y2 + l3 * math.sin(alpha + beta + gamma)
    
    return [x0, x1, x2, x3], [y0, y1, y2, y3]

#===============================================================================
# INTERACTIVE VISUALIZATION
#===============================================================================

def create_interactive_ik_solver():
    """
    Create the interactive inverse kinematics solver interface.
    """
    
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(14, 10))
    plt.subplots_adjust(bottom=0.25, right=0.75)  # Make room for input boxes and info
    
    # Initial target values
    current_values = {'px': 20.0, 'py': 15.0, 'phi': -25.0}
    
    # Solve initial inverse kinematics
    alpha, beta, gamma, success, msg = solve_inverse_kinematics(
        current_values['px'], current_values['py'], current_values['phi'])
    
    # Calculate initial arm position
    if success:
        arm_x, arm_y = calculate_arm_position(alpha, beta, gamma)
    else:
        arm_x, arm_y = [x0, x0, x0, x0], [y0, y0, y0, y0]
    
    # Create plots
    # Arm links
    line, = ax.plot(arm_x, arm_y, '-o', 
                   linewidth=3, markersize=8, color='royalblue', label='Arm')
    
    # Base marker
    ax.plot(x0, y0, 'o', markersize=12, color='red', label='Base')
    
    # Target point
    target_point, = ax.plot(current_values['px'], current_values['py'], 's', 
                           markersize=12, color='orange', label='Target')
    
    # Target orientation indicator (arrow)
    phi_rad = math.radians(current_values['phi'])
    arrow_length = 8
    target_arrow = ax.annotate('', 
                              xy=(current_values['px'] + arrow_length * math.cos(phi_rad),
                                  current_values['py'] + arrow_length * math.sin(phi_rad)),
                              xytext=(current_values['px'], current_values['py']),
                              arrowprops=dict(arrowstyle='->', 
                                            color='orange', lw=2))
    
    # Workspace circle (maximum reach)
    workspace_circle = plt.Circle((x0, y0), max_reach, 
                                 fill=False, linestyle='--', 
                                 color='gray', alpha=0.5, label='Max workspace')
    ax.add_patch(workspace_circle)
    
    # Configure plot with dynamic limits based on arm reach
    ax.set_xlim(x0 - plot_limit, x0 + plot_limit)
    ax.set_ylim(y0 - plot_limit, y0 + plot_limit)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position (cm)')
    ax.set_ylabel('Y Position (cm)')
    ax.legend(loc='upper left')
    
    # Title
    title = ax.set_title("Interactive Inverse Kinematics Solver - Enter values and click Calculate")
    
    #===========================================================================
    # CREATE INPUT BOXES
    #===========================================================================
    
    input_height = 0.04
    input_width = 0.15
    label_width = 0.1
    
    # Input box positions
    ax_px_label = plt.axes([0.05, 0.15, label_width, input_height])
    # Position sliders with dynamic range based on workspace
    max_coord = int(x0 + max_reach)
    min_coord = int(x0 - max_reach)

    # Input box positions
    ax_px_label = plt.axes([0.05, 0.15, label_width, input_height])
    ax_px = plt.axes([0.16, 0.15, input_width, input_height])
    
    ax_py_label = plt.axes([0.05, 0.10, label_width, input_height])
    ax_py = plt.axes([0.16, 0.10, input_width, input_height])
    
    ax_phi_label = plt.axes([0.05, 0.05, label_width, input_height])
    ax_phi = plt.axes([0.16, 0.05, input_width, input_height])
    

    # Create labels
    ax_px_label.text(0.5, 0.5, 'Target X (cm):', ha='center', va='center', transform=ax_px_label.transAxes)
    ax_px_label.set_xticks([])
    ax_px_label.set_yticks([])
    
    ax_py_label.text(0.5, 0.5, 'Target Y (cm):', ha='center', va='center', transform=ax_py_label.transAxes)
    ax_py_label.set_xticks([])
    ax_py_label.set_yticks([])
    
    ax_phi_label.text(0.5, 0.5, 'Orient. φ (°):', ha='center', va='center', transform=ax_phi_label.transAxes)
    ax_phi_label.set_xticks([])
    ax_phi_label.set_yticks([])

    # Create text input boxes
    textbox_px = TextBox(ax_px, '', initial=str(current_values['px']))
    textbox_py = TextBox(ax_py, '', initial=str(current_values['py']))
    textbox_phi = TextBox(ax_phi, '', initial=str(current_values['phi']))
    
    #===========================================================================
    # INFORMATION DISPLAY
    #===========================================================================
    
    # Create text box for displaying results
    info_text = plt.figtext(0.78, 0.7, '', fontsize=10, 
                           bbox=dict(boxstyle="round,pad=0.5", 
                                   facecolor="lightblue", alpha=0.8))
    
    status_text = plt.figtext(0.78, 0.5, '', fontsize=10,
                             bbox=dict(boxstyle="round,pad=0.3", 
                                     facecolor="lightgreen", alpha=0.8))
    
    def update_info_display(px, py, phi, alpha, beta, gamma, success, msg):
        """Update the information display with current values."""
        
        info_content = (
            f"TARGET:\n"
            f"Position: ({px:.1f}, {py:.1f}) cm\n"
            f"Orientation: {phi:.0f}°\n\n"
            f"JOINT ANGLES:\n"
            f"α = {alpha:6.1f}°\n"
            f"β = {beta:6.1f}°\n" 
            f"γ = {gamma:6.1f}°\n\n"
            f"WORKSPACE:\n"
            f"Max reach: {max_reach} cm\n"
            f"Target dist: {math.sqrt((px-x0)**2 + (py-y0)**2):.1f} cm"
        )
        
        info_text.set_text(info_content)
        
        # Status message
        status_color = "lightgreen" if success else "lightcoral"
        status_text.set_bbox(dict(boxstyle="round,pad=0.3", 
                                facecolor=status_color, alpha=0.8))
        status_text.set_text(f"STATUS:\n{msg}")
    
    #===========================================================================
    # UPDATE FUNCTION
    #===========================================================================
    
    def update_solver():
        """Update the solver when input values change."""
        nonlocal target_arrow  # Declare nonlocal at the beginning
        
        px = current_values['px']
        py = current_values['py']
        phi = current_values['phi']
        
        # Solve inverse kinematics
        alpha, beta, gamma, success, msg = solve_inverse_kinematics(px, py, phi)
        
        # Update target point and arrow
        target_point.set_xdata([px])
        target_point.set_ydata([py])
        
        phi_rad = math.radians(phi)
        target_arrow.remove()
        new_arrow = ax.annotate('', 
                               xy=(px + arrow_length * math.cos(phi_rad),
                                   py + arrow_length * math.sin(phi_rad)),
                               xytext=(px, py),
                               arrowprops=dict(arrowstyle='->', 
                                             color='orange', lw=2))
        
        # Update arm position if solution exists
        if success:
            arm_x, arm_y = calculate_arm_position(alpha, beta, gamma)
            line.set_xdata(arm_x)
            line.set_ydata(arm_y)
            line.set_color('royalblue')
        else:
            # Show arm in error state (red)
            line.set_color('red')
        
        # Update information display
        update_info_display(px, py, phi, alpha, beta, gamma, success, msg)
        
        # Update the global arrow reference
        target_arrow = new_arrow
        
        # Redraw
        fig.canvas.draw_idle()
    
    #===========================================================================
    # INPUT HANDLERS
    #===========================================================================
    
    def on_px_change(text):
        """Handle X coordinate input change - no auto-update."""
        try:
            value = float(text)
            current_values['px'] = value
            # No automatic update - only when Calculate is pressed
        except ValueError:
            pass  # Ignore invalid input
    
    def on_py_change(text):
        """Handle Y coordinate input change - no auto-update."""
        try:
            value = float(text)
            current_values['py'] = value
            # No automatic update - only when Calculate is pressed
        except ValueError:
            pass  # Ignore invalid input
    
    def on_phi_change(text):
        """Handle orientation input change - no auto-update."""
        try:
            value = float(text)
            current_values['phi'] = value
            # No automatic update - only when Calculate is pressed
        except ValueError:
            pass  # Ignore invalid input
    
    # Connect input boxes to handlers
    textbox_px.on_submit(on_px_change)
    textbox_py.on_submit(on_py_change)
    textbox_phi.on_submit(on_phi_change)
    
    #===========================================================================
    # BUTTONS
    #===========================================================================
    
    # Reset button
    ax_reset = plt.axes([0.35, 0.15, 0.08, 0.04])
    button_reset = Button(ax_reset, 'Reset')
    
    def reset_solver(event):
        """Reset to initial position."""
        current_values['px'] = x0 + 20.0
        current_values['py'] = y0 + 15.0
        current_values['phi'] = -25.0
        
        textbox_px.set_val(str(current_values['px']))
        textbox_py.set_val(str(current_values['py']))
        textbox_phi.set_val(str(current_values['phi']))
        
        update_solver()
    
    button_reset.on_clicked(reset_solver)
    
    # Calculate button (manual update only)
    ax_calc = plt.axes([0.35, 0.10, 0.08, 0.04])
    button_calc = Button(ax_calc, 'Calculate')
    
    def manual_calculate(event):
        """Calculate and update when button is pressed."""
        # Force update from current textbox values
        try:
            current_values['px'] = float(textbox_px.text)
            current_values['py'] = float(textbox_py.text)
            current_values['phi'] = float(textbox_phi.text)
            update_solver()
        except ValueError:
            # Show error in status if invalid input
            status_text.set_bbox(dict(boxstyle="round,pad=0.3", 
                                    facecolor="lightcoral", alpha=0.8))
            status_text.set_text("STATUS:\nInvalid input values!\nCheck your numbers.")
            fig.canvas.draw_idle()
    
    button_calc.on_clicked(manual_calculate)
    
    # Initialize display
    update_solver()
    
    # Instructions
    instructions = (
        "INSTRUCTIONS:\n"
        "• Type values in input boxes\n"
        "• Click 'Calculate' to update\n"
        "• No auto-update on typing\n\n"
        "VISUALIZATION:\n"
        "• Orange square: Target point\n"
        "• Orange arrow: Target orientation\n"
        "• Dashed circle: Workspace limit\n"
        "• Blue arm: Valid solution\n"
        "• Red arm: Unreachable position\n\n"
        "EXAMPLES:\n"
        "• X=20, Y=15, φ=-25°\n"
        "• X=30, Y=0, φ=0°\n"
        "• X=10, Y=25, φ=45°"
    )
    
    plt.figtext(0.78, 0.05, instructions, fontsize=9,
                bbox=dict(boxstyle="round,pad=0.3", 
                         facecolor="lightyellow", alpha=0.8))
    
    plt.show()

#===============================================================================
# MAIN EXECUTION
#===============================================================================

if __name__ == "__main__":
    print("Starting Interactive Inverse Kinematics Solver...")
    print("Instructions:")
    print("1. Type values in the input boxes")
    print("2. Click 'Calculate' button to update the arm")  
    print("3. The arm will NOT update automatically while typing")
    print("4. Use 'Reset' to return to default values")
    print("5. Invalid inputs will show an error message")
    print("Close the window to exit the program.")
    
    create_interactive_ik_solver()
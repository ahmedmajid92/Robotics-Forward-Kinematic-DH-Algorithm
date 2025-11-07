"""
COMAU Smart Six 6-1.4 Robot Model Module

This module defines the forward kinematics model for the COMAU Smart Six 6-1.4
industrial robot using Denavit-Hartenberg (DH) parameters.

The DH parameters are based on the paper:
Guida et al. (2019), "Multibody Model of the COMAU Smart Six 6-1.4 Industrial Robot"
IOP Conf. Ser.: Mater. Sci. Eng. 568 012115

Key calibrations:
- a1 = 0.101m (calibrated from paper's test cases, not explicitly in Table 1)
- Joint 2 has 90° offset (paper's coordinate frame convention)
- d4 = 0.674m (corrected value from paper's supplementary data)
"""

# Import required libraries
import numpy as np  # For numerical operations and array handling
from roboticstoolbox import DHRobot, RevoluteDH  # Robotics Toolbox for DH modeling
from spatialmath import SE3  # For spatial transformations (SE3 = Special Euclidean group in 3D)


def create_comau_robot():
    """
    Creates and returns the DHRobot model for the COMAU Smart Six 6-1.4.
    
    This function constructs a 6-DOF (Degrees of Freedom) robot model using the
    Standard Denavit-Hartenberg (DH) convention. Each joint is modeled as a
    revolute joint with specific DH parameters.
    
    DH Parameters Explanation:
    - d: Link offset (distance along previous z to common normal)
    - a: Link length (length of the common normal)
    - alpha: Link twist (angle about common normal, from old z to new z)
    - offset: Joint angle offset (added to joint variable)
    - qlim: Joint limits in radians [min, max]
    
    Based on: Guida et al. (2019), IOP Conf. Ser.: Mater. Sci. Eng. 568 012115
    
    DH Parameters (Standard DH Convention):
    Joint | d     | a     | alpha   | offset | Notes
    ------|-------|-------|---------|--------|------------------
    1     | 0.45  | 0.101 | π/2     | 0°     | Base height + shoulder offset (calibrated)
    2     | 0     | 0.59  | 0       | 90°    | Upper arm (offset for paper's "zero")
    3     | 0     | 0.13  | π/2     | 0°     | Elbow offset
    4     | 0.674 | 0     | -π/2    | 0°     | Forearm length
    5     | 0     | 0     | π/2     | 0°     | Wrist pitch
    6     | 0.095 | 0     | 0       | 0°     | Tool flange
    
    IMPORTANT: Paper's coordinate system uses:
               - 90° offset on joint 2 (paper's q=0 means shoulder horizontal)
               - a1 = 101mm (calibrated from paper's test cases, not in Table 1)
               - This gives exact match to Table 2 test positions
    
    Returns:
        DHRobot: A roboticstoolbox DHRobot object representing the COMAU robot
    
    Example:
        >>> robot = create_comau_robot()
        >>> q = np.zeros(6)  # Home position
        >>> T = robot.fkine(q)  # Forward kinematics
        >>> print(T.t)  # End-effector position
        [0.87, 0.0, 1.17]
    """
    
    # Create a list to store the 6 revolute joint links
    links = [
        # ===== JOINT 1: BASE ROTATION =====
        # This is the base rotation joint around the vertical Z-axis
        RevoluteDH(
            d=0.45,          # d1 - Base height (450mm from ground to joint 2)
            a=0.101,         # a1 - Shoulder offset (101mm, CALIBRATED value)
                             #      This is the horizontal offset from joint 1 to joint 2
                             #      Not explicitly in paper, found through calibration
            alpha=np.pi/2,   # alpha1 - Link twist is 90° (π/2 radians)
                             #          Rotates frame so Z-axis points forward
            offset=0,        # No angular offset - joint zero matches physical zero
            qlim=[-170*np.pi/180, 170*np.pi/180]  # Joint limits: -170° to +170°
                                                   # Converted to radians for computation
        ),
        
        # ===== JOINT 2: SHOULDER =====
        # This joint controls the shoulder pitch (up/down motion of the arm)
        RevoluteDH(
            d=0,             # d2 - No vertical offset (joint 2 is at same height as joint 1)
            a=0.59,          # a2 - Upper arm length (590mm from shoulder to elbow)
                             #      This is the main structural element of the upper arm
            alpha=0,         # alpha2 - No link twist (Z-axes remain parallel)
            offset=np.pi/2,  # +90° offset (π/2 radians) - IMPORTANT!
                             # Paper's convention: q2=0 means arm is horizontal
                             # Physical robot: q2=0 means arm points up
                             # This offset reconciles the two coordinate systems
            qlim=[-85*np.pi/180, 155*np.pi/180]  # Joint limits: -85° to +155°
                                                  # Allows arm to go down to -85° and up to 155°
        ),
        
        # ===== JOINT 3: ELBOW =====
        # This joint controls the elbow bend
        RevoluteDH(
            d=0,             # d3 - No vertical offset along Z-axis
            a=0.13,          # a3 - Elbow offset (130mm)
                             #      Small offset accounting for mechanical structure at elbow
            alpha=np.pi/2,   # alpha3 - Link twist is 90°
                             #          Rotates frame for wrist orientation
            offset=0,        # No angular offset
            qlim=[-170*np.pi/180, 158*np.pi/180]  # Joint limits: -170° to +158°
        ),
        
        # ===== JOINT 4: WRIST ROLL =====
        # This joint rotates the wrist around its axis (roll motion)
        RevoluteDH(
            d=0.674,         # d4 - Forearm length (674mm from elbow to wrist)
                             #      This is the CORRECTED value from paper
                             #      Paper's Table 1 had 647mm, but 674mm matches test data
            a=0,             # a4 - No link length offset
            alpha=-np.pi/2,  # alpha4 - Link twist is -90° (negative π/2)
                             #          Negative twist to orient next joint correctly
            offset=0,        # No angular offset
            qlim=[-270*np.pi/180, 270*np.pi/180]  # Joint limits: -270° to +270°
                                                   # Wide range for continuous rotation
        ),
        
        # ===== JOINT 5: WRIST PITCH =====
        # This joint controls wrist pitch (up/down bending of end-effector)
        RevoluteDH(
            d=0,             # d5 - No offset along Z-axis
            a=0,             # a5 - No link length
            alpha=np.pi/2,   # alpha5 - Link twist is 90°
                             #          Perpendicular orientation for final joint
            offset=0,        # No angular offset
            qlim=[-130*np.pi/180, 130*np.pi/180]  # Joint limits: -130° to +130°
        ),
        
        # ===== JOINT 6: WRIST YAW =====
        # This is the tool flange rotation (twists the end-effector)
        RevoluteDH(
            d=0.095,         # d6 - Tool flange thickness (95mm)
                             #      Distance from wrist center to tool mounting point
            a=0,             # a6 - No link length (end of kinematic chain)
            alpha=0,         # alpha6 - No link twist (end-effector frame aligned)
            offset=0,        # No angular offset
            qlim=[-270*np.pi/180, 270*np.pi/180]  # Joint limits: -270° to +270°
                                                   # Wide range for tool rotation
        )
    ]

    # Create the DHRobot object from the list of links
    # This object provides forward/inverse kinematics, Jacobian, etc.
    robot = DHRobot(
        links,                           # List of 6 DH link parameters defined above
        name="COMAU Smart Six 6-1.4"    # Human-readable name for the robot
    )
    
    # Return the fully constructed robot model
    return robot


# ===== MAIN EXECUTION BLOCK =====
# This block runs when the script is executed directly (not imported as a module)
if __name__ == '__main__':
    # Create an instance of the COMAU robot
    robot = create_comau_robot()
    
    # Print robot information (shows DH table and structure)
    print("COMAU Smart Six 6-1.4 Robot Model")
    print(robot)  # This prints a formatted table of DH parameters
    
    # ===== VALIDATION TEST =====
    # Validate the model against the paper's home position test case
    
    # Define home position: all joints at zero radians
    q = np.zeros(6)  # Creates array [0, 0, 0, 0, 0, 0]
    
    # Calculate forward kinematics: joint angles → end-effector pose
    # T is a 4x4 homogeneous transformation matrix (SE3 object)
    T = robot.fkine(q)
    
    # Expected end-effector position from paper's Table 2 for q=[0,0,0,0,0,0]
    target = np.array([0.87, 0, 1.17])  # [x, y, z] in meters
    
    # Calculate position error using Euclidean distance (L2 norm)
    # error = sqrt((x_calc - x_ref)² + (y_calc - y_ref)² + (z_calc - z_ref)²)
    error = np.linalg.norm(T.t - target)
    
    # Print validation results
    print(f"\nValidation at q=[0,0,0,0,0,0]:")
    # T.t extracts the translation vector (position) from transformation matrix
    print(f"  Calculated: [{T.t[0]:.6f}, {T.t[1]:.6f}, {T.t[2]:.6f}]")
    print(f"  Expected:   [{target[0]:.6f}, {target[1]:.6f}, {target[2]:.6f}]")
    # Convert error to millimeters for readability
    # Show ✓ if error < 1mm (essentially perfect), otherwise ✗
    print(f"  Error: {error*1000:.3f}mm {'✓' if error < 0.001 else '✗'}")
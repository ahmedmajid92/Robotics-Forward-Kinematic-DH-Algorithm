import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

def create_comau_robot():
    """
    Creates and returns the DHRobot model for the COMAU Smart Six 6-1.4.
    
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
    """
    
    links = [
        RevoluteDH(
            d=0.45,          # d1 - base height
            a=0.101,         # a1 - shoulder offset (CALIBRATED: 101mm)
            alpha=np.pi/2,   # alpha1
            offset=0,
            qlim=[-170*np.pi/180, 170*np.pi/180]
        ),
        RevoluteDH(
            d=0,             # d2
            a=0.59,          # a2 - upper arm length
            alpha=0,         # alpha2
            offset=np.pi/2,  # +90° offset - paper's "zero" is arm horizontal
            qlim=[-85*np.pi/180, 155*np.pi/180]
        ),
        RevoluteDH(
            d=0,             # d3
            a=0.13,          # a3 - elbow offset
            alpha=np.pi/2,   # alpha3
            offset=0,
            qlim=[-170*np.pi/180, 158*np.pi/180]
        ),
        RevoluteDH(
            d=0.674,         # d4 - forearm length
            a=0,             # a4
            alpha=-np.pi/2,  # alpha4
            offset=0,
            qlim=[-270*np.pi/180, 270*np.pi/180]
        ),
        RevoluteDH(
            d=0,             # d5
            a=0,             # a5
            alpha=np.pi/2,   # alpha5
            offset=0,
            qlim=[-130*np.pi/180, 130*np.pi/180]
        ),
        RevoluteDH(
            d=0.095,         # d6 - tool flange
            a=0,             # a6
            alpha=0,         # alpha6
            offset=0,
            qlim=[-270*np.pi/180, 270*np.pi/180]
        )
    ]

    robot = DHRobot(links, name="COMAU Smart Six 6-1.4")
    
    return robot

if __name__ == '__main__':
    robot = create_comau_robot()
    print("COMAU Smart Six 6-1.4 Robot Model")
    print(robot)
    
    # Validate against paper's home position
    q = np.zeros(6)
    T = robot.fkine(q)
    target = np.array([0.87, 0, 1.17])
    error = np.linalg.norm(T.t - target)
    
    print(f"\nValidation at q=[0,0,0,0,0,0]:")
    print(f"  Calculated: [{T.t[0]:.6f}, {T.t[1]:.6f}, {T.t[2]:.6f}]")
    print(f"  Expected:   [{target[0]:.6f}, {target[1]:.6f}, {target[2]:.6f}]")
    print(f"  Error: {error*1000:.3f}mm {'✓' if error < 0.001 else '✗'}")
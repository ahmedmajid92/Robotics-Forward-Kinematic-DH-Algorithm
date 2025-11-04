import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, RevoluteMDH

def test_different_conventions():
    """
    Test different DH parameter interpretations to match paper
    """
    
    print("="*90)
    print("Testing Different DH Conventions")
    print("="*90)
    
    # Paper's expected result at q=0: [0.87, 0, 1.17]
    target = np.array([0.87, 0, 1.17])
    
    print("\nTest 1: Paper's exact parameters (a1=0)")
    links1 = [
        RevoluteDH(d=0.45, a=0, alpha=np.pi/2),      # Joint 1
        RevoluteDH(d=0, a=0.59, alpha=0),             # Joint 2
        RevoluteDH(d=0, a=0.13, alpha=np.pi/2),       # Joint 3
        RevoluteDH(d=0.674, a=0, alpha=-np.pi/2),     # Joint 4
        RevoluteDH(d=0, a=0, alpha=np.pi/2),          # Joint 5
        RevoluteDH(d=0.095, a=0, alpha=0)             # Joint 6
    ]
    robot1 = DHRobot(links1, name="Test1")
    T1 = robot1.fkine(np.zeros(6))
    print(f"Position: {T1.t}")
    print(f"Error: {np.linalg.norm(T1.t - target)*1000:.1f}mm")
    
    print("\nTest 2: With a1=0.15 (your current model)")
    links2 = [
        RevoluteDH(d=0.45, a=0.15, alpha=np.pi/2),    # Joint 1
        RevoluteDH(d=0, a=0.59, alpha=0),             # Joint 2
        RevoluteDH(d=0, a=0.13, alpha=np.pi/2),       # Joint 3
        RevoluteDH(d=0.674, a=0, alpha=-np.pi/2),     # Joint 4
        RevoluteDH(d=0, a=0, alpha=np.pi/2),          # Joint 5
        RevoluteDH(d=0.095, a=0, alpha=0)             # Joint 6
    ]
    robot2 = DHRobot(links2, name="Test2")
    T2 = robot2.fkine(np.zeros(6))
    print(f"Position: {T2.t}")
    print(f"Error: {np.linalg.norm(T2.t - target)*1000:.1f}mm")
    
    print("\nTest 3: Trying different joint 2 offset")
    links3 = [
        RevoluteDH(d=0.45, a=0, alpha=np.pi/2),
        RevoluteDH(d=0, a=0.59, alpha=0, offset=-np.pi/2),  # -90° offset
        RevoluteDH(d=0, a=0.13, alpha=np.pi/2),
        RevoluteDH(d=0.674, a=0, alpha=-np.pi/2),
        RevoluteDH(d=0, a=0, alpha=np.pi/2),
        RevoluteDH(d=0.095, a=0, alpha=0)
    ]
    robot3 = DHRobot(links3, name="Test3")
    T3 = robot3.fkine(np.zeros(6))
    print(f"Position: {T3.t}")
    print(f"Error: {np.linalg.norm(T3.t - target)*1000:.1f}mm")
    
    print("\nTest 4: Check if paper uses Modified DH")
    # Modified DH has different transformation order
    links4 = [
        RevoluteMDH(a=0, alpha=np.pi/2, d=0.45),
        RevoluteMDH(a=0.59, alpha=0, d=0),
        RevoluteMDH(a=0.13, alpha=np.pi/2, d=0),
        RevoluteMDH(a=0, alpha=-np.pi/2, d=0.674),
        RevoluteMDH(a=0, alpha=np.pi/2, d=0),
        RevoluteMDH(a=0, alpha=0, d=0.095)
    ]
    robot4 = DHRobot(links4, name="Test4_MDH")
    T4 = robot4.fkine(np.zeros(6))
    print(f"Position: {T4.t}")
    print(f"Error: {np.linalg.norm(T4.t - target)*1000:.1f}mm")
    
    print("\nTest 5: Paper might define 'zero' at different config")
    # Maybe paper's "zero" is actually shoulder down (-90°)
    T5 = robot1.fkine(np.deg2rad([0, -90, 0, 0, 0, 0]))
    print(f"Position at q2=-90°: {T5.t}")
    print(f"Error: {np.linalg.norm(T5.t - target)*1000:.1f}mm")
    
    print("\nTest 6: Maybe paper's 'zero' has positive q2")
    T6 = robot1.fkine(np.deg2rad([0, 90, 0, 0, 0, 0]))
    print(f"Position at q2=90°: {T6.t}")
    print(f"Error: {np.linalg.norm(T6.t - target)*1000:.1f}mm")
    
    print("\nTest 7: Search for correct offset on joint 2")
    best_error = float('inf')
    best_offset = 0
    for offset_deg in range(-180, 181, 15):
        offset_rad = np.deg2rad(offset_deg)
        links_test = [
            RevoluteDH(d=0.45, a=0, alpha=np.pi/2),
            RevoluteDH(d=0, a=0.59, alpha=0, offset=offset_rad),
            RevoluteDH(d=0, a=0.13, alpha=np.pi/2),
            RevoluteDH(d=0.674, a=0, alpha=-np.pi/2),
            RevoluteDH(d=0, a=0, alpha=np.pi/2),
            RevoluteDH(d=0.095, a=0, alpha=0)
        ]
        robot_test = DHRobot(links_test, name="Test")
        T_test = robot_test.fkine(np.zeros(6))
        error = np.linalg.norm(T_test.t - target)
        if error < best_error:
            best_error = error
            best_offset = offset_deg
            best_pos = T_test.t
    
    print(f"Best offset: {best_offset}° → Position: {best_pos}, Error: {best_error*1000:.1f}mm")
    
    print("="*90)

if __name__ == '__main__':
    test_different_conventions()
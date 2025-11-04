import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH

def find_correct_a1():
    """
    Find the exact a1 value that produces the paper's results.
    """
    target = np.array([0.87, 0, 1.17])
    
    print("="*90)
    print("Calibrating a1 parameter")
    print(f"Target position at q=[0,0,0,0,0,0]: {target}")
    print("="*90)
    
    best_error = float('inf')
    best_a1 = 0
    
    # Search for optimal a1
    for a1_mm in range(0, 201, 1):  # 0mm to 200mm in 1mm steps
        a1 = a1_mm / 1000.0
        
        links = [
            RevoluteDH(d=0.45, a=a1, alpha=np.pi/2),
            RevoluteDH(d=0, a=0.59, alpha=0, offset=np.pi/2),
            RevoluteDH(d=0, a=0.13, alpha=np.pi/2),
            RevoluteDH(d=0.674, a=0, alpha=-np.pi/2),
            RevoluteDH(d=0, a=0, alpha=np.pi/2),
            RevoluteDH(d=0.095, a=0, alpha=0)
        ]
        
        robot = DHRobot(links, name="Test")
        T = robot.fkine(np.zeros(6))
        pos = T.t
        error = np.linalg.norm(pos - target)
        
        if error < best_error:
            best_error = error
            best_a1 = a1
            best_pos = pos
    
    print(f"\n✓ Optimal a1 found: {best_a1*1000:.1f}mm ({best_a1:.4f}m)")
    print(f"  Resulting position: [{best_pos[0]:.6f}, {best_pos[1]:.6f}, {best_pos[2]:.6f}]")
    print(f"  Target position:    [{target[0]:.6f}, {target[1]:.6f}, {target[2]:.6f}]")
    print(f"  Error: {best_error*1000:.3f}mm")
    print("="*90 + "\n")
    
    return best_a1

if __name__ == '__main__':
    optimal_a1 = find_correct_a1()
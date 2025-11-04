import numpy as np
from comau_model import create_comau_robot

def find_qs_configuration():
    """
    Find the correct joint configuration for q_s (Singular/Stretched).
    Target from Table 2: [0.45, 0, 0.87]
    """
    robot = create_comau_robot()
    target = np.array([0.45, 0.0, 0.87])
    
    print("="*90)
    print("Searching for paper's q_s (Singular) configuration")
    print(f"Target position: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    print("="*90)
    
    best_error = float('inf')
    best_config = None
    
    # Search through joint space
    for q2 in range(-90, 181, 15):
        for q3 in range(-90, 91, 15):
            q = np.deg2rad([0, q2, q3, 0, 0, 0])
            T = robot.fkine(q)
            pos = T.t
            error = np.linalg.norm(pos - target)
            
            if error < 0.05:  # Within 50mm
                print(f"q=[0, {q2:4d}, {q3:4d}, 0, 0, 0]° "
                      f"→ [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]  Error: {error*1000:5.1f}mm")
            
            if error < best_error:
                best_error = error
                best_config = ([0, q2, q3, 0, 0, 0], pos)
    
    print("-"*90)
    print(f"\n✓ Best configuration found:")
    print(f"  q = [0, {best_config[0][1]:4d}, {best_config[0][2]:4d}, 0, 0, 0]°")
    print(f"  Position: [{best_config[1][0]:.3f}, {best_config[1][1]:.3f}, {best_config[1][2]:.3f}]")
    print(f"  Target:   [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    print(f"  Error: {best_error*1000:.1f}mm")
    print("="*90 + "\n")
    
    return best_config[0]

if __name__ == '__main__':
    qs = find_qs_configuration()
import numpy as np
from comau_model import create_comau_robot

def find_all_missing_configs():
    """
    Find correct joint configurations for q_s and q_n with updated model (a1=0.101).
    """
    robot = create_comau_robot()
    
    # Target positions from paper's Table 2
    targets = {
        "q_s": np.array([0.45, 0.0, 0.87]),
        "q_n": np.array([1.19, 0.0, 0.501])
    }
    
    print("="*90)
    print("Searching for correct configurations with calibrated model (a1=0.101)")
    print("="*90)
    
    for name, target in targets.items():
        print(f"\n{name}: Target = [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
        print("-"*90)
        
        best_error = float('inf')
        best_config = None
        
        # Comprehensive search
        for q1 in range(0, 91, 15):
            for q2 in range(-90, 181, 15):
                for q3 in range(-90, 91, 15):
                    for q4 in range(0, 91, 30):
                        for q5 in range(0, 91, 30):
                            for q6 in [0]:  # Paper likely uses q6=0
                                q = np.deg2rad([q1, q2, q3, q4, q5, q6])
                                T = robot.fkine(q)
                                pos = T.t
                                error = np.linalg.norm(pos - target)
                                
                                if error < 0.05:  # Within 50mm
                                    print(f"  q=[{q1:3d}, {q2:4d}, {q3:4d}, {q4:3d}, {q5:3d}, {q6:3d}]° "
                                          f"→ [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]  "
                                          f"Error: {error*1000:5.1f}mm")
                                
                                if error < best_error:
                                    best_error = error
                                    best_config = ([q1, q2, q3, q4, q5, q6], pos)
        
        print(f"\n  ✓ Best: q=[{best_config[0][0]:3d}, {best_config[0][1]:4d}, {best_config[0][2]:4d}, "
              f"{best_config[0][3]:3d}, {best_config[0][4]:3d}, {best_config[0][5]:3d}]°")
        print(f"    Position: [{best_config[1][0]:.3f}, {best_config[1][1]:.3f}, {best_config[1][2]:.3f}]")
        print(f"    Error: {best_error*1000:.1f}mm")
    
    print("\n" + "="*90)

if __name__ == '__main__':
    find_all_missing_configs()
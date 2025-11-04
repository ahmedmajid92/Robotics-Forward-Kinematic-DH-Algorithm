import numpy as np
from comau_model import create_comau_robot

def find_paper_qn():
    """
    Find the joint configuration that produces the paper's q_n position.
    Target from Table 2: [1.19, 0, 0.501]
    """
    robot = create_comau_robot()
    target = np.array([1.19, 0.0, 0.501])
    
    print("="*90)
    print("Searching for paper's q_n configuration")
    print(f"Target position: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    print("="*90)
    
    best_error = float('inf')
    best_config = None
    
    # Systematic search through joint space
    for q1 in range(0, 91, 15):
        for q2 in range(0, 91, 15):
            for q3 in range(-90, 1, 15):
                for q4 in range(0, 91, 30):
                    for q5 in range(0, 91, 30):
                        for q6 in range(0, 91, 45):
                            q = np.deg2rad([q1, q2, q3, q4, q5, q6])
                            T = robot.fkine(q)
                            pos = T.t
                            error = np.linalg.norm(pos - target)
                            
                            if error < 0.05:  # Within 50mm
                                print(f"q=[{q1:3d}, {q2:3d}, {q3:4d}, {q4:3d}, {q5:3d}, {q6:3d}]° "
                                      f"→ [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]  "
                                      f"Error: {error*1000:5.1f}mm")
                            
                            if error < best_error:
                                best_error = error
                                best_config = ([q1, q2, q3, q4, q5, q6], pos)
    
    print("-"*90)
    print("\n✓ Best match found:")
    print(f"  q = [{best_config[0][0]:3d}, {best_config[0][1]:3d}, {best_config[0][2]:4d}, "
          f"{best_config[0][3]:3d}, {best_config[0][4]:3d}, {best_config[0][5]:3d}]°")
    print(f"  Position: [{best_config[1][0]:.3f}, {best_config[1][1]:.3f}, {best_config[1][2]:.3f}]")
    print(f"  Error: {best_error*1000:.1f}mm")
    print("="*90 + "\n")
    
    return best_config[0]

if __name__ == '__main__':
    qn = find_paper_qn()
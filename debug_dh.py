import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from comau_model import create_comau_robot

def test_dh_parameters():
    """
    Debug script to test different DH parameter interpretations
    and find which matches the paper's results.
    """
    
    print("="*90)
    print("DH Parameter Debugging for COMAU Smart Six 6-1.4")
    print("="*90)
    
    # Paper's DH parameters from Table 1
    print("\nPaper's DH Parameters (Table 1):")
    print("Joint | theta | d     | a     | alpha")
    print("------|-------|-------|-------|-------")
    print("1     | 0     | 0.45  | 0     | π/2")
    print("2     | 0     | 0     | 0.59  | 0")
    print("3     | 0     | 0     | 0.13  | π/2")
    print("4     | 0     | 0.647 | 0     | -π/2")
    print("5     | 0     | 0     | 0     | π/2")
    print("6     | 0     | 0.095 | 0     | 0")
    
    # Test Configuration 1: Standard DH with NO offsets
    print("\n" + "="*90)
    print("TEST 1: Standard DH - NO offsets, NO adjustments")
    print("="*90)
    
    links1 = [
        RevoluteDH(d=0.45, a=0, alpha=np.pi/2, offset=0),
        RevoluteDH(d=0, a=0.59, alpha=0, offset=0),
        RevoluteDH(d=0, a=0.13, alpha=np.pi/2, offset=0),
        RevoluteDH(d=0.647, a=0, alpha=-np.pi/2, offset=0),
        RevoluteDH(d=0, a=0, alpha=np.pi/2, offset=0),
        RevoluteDH(d=0.095, a=0, alpha=0, offset=0)
    ]
    robot1 = DHRobot(links1, name="Test1")
    
    # Test zero configuration
    q_zero = np.zeros(6)
    T1 = robot1.fkine(q_zero)
    print(f"q=[0,0,0,0,0,0] → Position: [{T1.t[0]:.3f}, {T1.t[1]:.3f}, {T1.t[2]:.3f}]")
    print(f"Expected: [0.870, 0.000, -0.270]")
    print(f"Error: {np.linalg.norm(T1.t - np.array([0.87, 0, -0.27])):.4f}m")
    
    # Test stretched configuration
    q_stretch = np.deg2rad([0, 90, 0, 0, 0, 0])
    T1s = robot1.fkine(q_stretch)
    print(f"\nq=[0,90,0,0,0,0] → Position: [{T1s.t[0]:.3f}, {T1s.t[1]:.3f}, {T1s.t[2]:.3f}]")
    print(f"Expected: [1.465, 0.000, 0.580]")
    print(f"Error: {np.linalg.norm(T1s.t - np.array([1.465, 0, 0.58])):.4f}m")
    
    # Test Configuration 2: With a1 = 0.15
    print("\n" + "="*90)
    print("TEST 2: Standard DH - WITH a1=0.15 (shoulder offset)")
    print("="*90)
    
    links2 = [
        RevoluteDH(d=0.45, a=0.15, alpha=np.pi/2, offset=0),
        RevoluteDH(d=0, a=0.59, alpha=0, offset=0),
        RevoluteDH(d=0, a=0.13, alpha=np.pi/2, offset=0),
        RevoluteDH(d=0.647, a=0, alpha=-np.pi/2, offset=0),
        RevoluteDH(d=0, a=0, alpha=np.pi/2, offset=0),
        RevoluteDH(d=0.095, a=0, alpha=0, offset=0)
    ]
    robot2 = DHRobot(links2, name="Test2")
    
    T2 = robot2.fkine(q_zero)
    print(f"q=[0,0,0,0,0,0] → Position: [{T2.t[0]:.3f}, {T2.t[1]:.3f}, {T2.t[2]:.3f}]")
    print(f"Expected: [0.870, 0.000, -0.270]")
    print(f"Error: {np.linalg.norm(T2.t - np.array([0.87, 0, -0.27])):.4f}m")
    
    T2s = robot2.fkine(q_stretch)
    print(f"\nq=[0,90,0,0,0,0] → Position: [{T2s.t[0]:.3f}, {T2s.t[1]:.3f}, {T2s.t[2]:.3f}]")
    print(f"Expected: [1.465, 0.000, 0.580]")
    print(f"Error: {np.linalg.norm(T2s.t - np.array([1.465, 0, 0.58])):.4f}m")
    
    # Test Configuration 3: Paper might use -90° for "down" shoulder position
    print("\n" + "="*90)
    print("TEST 3: Paper's 'zero' might be robot's -90° on joint 2")
    print("="*90)
    
    q_paper_zero = np.deg2rad([0, -90, 0, 0, 0, 0])
    T3 = robot2.fkine(q_paper_zero)
    print(f"q=[0,-90,0,0,0,0] → Position: [{T3.t[0]:.3f}, {T3.t[1]:.3f}, {T3.t[2]:.3f}]")
    print(f"Expected: [0.870, 0.000, -0.270]")
    print(f"Error: {np.linalg.norm(T3.t - np.array([0.87, 0, -0.27])):.4f}m")
    
    # Test different shoulder angles for stretched config
    print("\n" + "="*90)
    print("TEST 4: Finding correct 'stretched' joint angles")
    print("="*90)
    
    target_stretch = np.array([1.465, 0, 0.58])
    best_error = float('inf')
    best_config = None
    
    for q2 in [-90, -45, 0, 45, 90]:
        for q3 in [-90, -45, 0, 45, 90]:
            q_test = np.deg2rad([0, q2, q3, 0, 0, 0])
            T_test = robot2.fkine(q_test)
            error = np.linalg.norm(T_test.t - target_stretch)
            
            if error < best_error:
                best_error = error
                best_config = (q2, q3, T_test.t)
            
            if error < 0.1:  # Print close matches
                print(f"q2={q2:4.0f}°, q3={q3:4.0f}° → [{T_test.t[0]:.3f}, {T_test.t[1]:.3f}, {T_test.t[2]:.3f}] Error={error:.4f}m")
    
    print(f"\nBest match: q2={best_config[0]:.0f}°, q3={best_config[1]:.0f}°")
    print(f"Position: [{best_config[2][0]:.3f}, {best_config[2][1]:.3f}, {best_config[2][2]:.3f}]")
    print(f"Error: {best_error:.4f}m")
    
    print("\n" + "="*90)
    print("SUMMARY")
    print("="*90)
    print("\nThe paper likely uses:")
    print("1. a1 = 0.15m (shoulder offset)")
    print("2. 'Home' position at q=[0,-90°,0,0,0,0] (arm pointing down)")
    print("3. 'Stretched' needs specific q2/q3 combination")
    

def find_qn_configuration():
    """
    Comprehensive search for q_n configuration
    Target: [0.615, 0.615, 0.501]
    """
    robot = create_comau_robot()
    
    print("="*90)
    print("Comprehensive Search for q_n (Generic) Configuration")
    print("="*90)
    print(f"Target position: [0.615, 0.615, 0.501]")
    print("\nPhase 1: Grid search across all joint combinations...")
    print("-"*90)
    
    target = np.array([0.615, 0.615, 0.501])
    best_error = float('inf')
    best_config = None
    
    # Grid search with finer resolution around promising areas
    for q1 in [45]:  # Keep q1 at 45° as specified
        for q2 in range(-90, 91, 15):
            for q3 in range(-90, 91, 15):
                for q4 in [0, 45, 90]:
                    for q5 in [0, 45]:
                        q_test = np.deg2rad([q1, q2, q3, q4, q5, 0])
                        T = robot.fkine(q_test)
                        pos = T.t
                        error = np.linalg.norm(pos - target)
                        
                        if error < 0.1:  # Within 100mm
                            print(f"q=[{q1:3.0f}, {q2:3.0f}, {q3:3.0f}, {q4:3.0f}, {q5:3.0f}, 0]° "
                                  f"→ [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]  Error: {error*1000:5.1f}mm")
                        
                        if error < best_error:
                            best_error = error
                            best_config = ([q1, q2, q3, q4, q5, 0], pos)
    
    print("-"*90)
    print(f"\nPhase 1 Best: q={best_config[0]} → Error: {best_error*1000:.1f}mm")
    
    # Phase 2: Fine-tune around the best configuration
    print("\nPhase 2: Fine-tuning around best configuration...")
    print("-"*90)
    
    base_q = best_config[0]
    
    for dq2 in range(-10, 11, 2):
        for dq3 in range(-10, 11, 2):
            for dq5 in range(-10, 11, 2):
                q_test = np.deg2rad([
                    base_q[0],
                    base_q[1] + dq2,
                    base_q[2] + dq3,
                    base_q[3],
                    base_q[4] + dq5,
                    0
                ])
                T = robot.fkine(q_test)
                pos = T.t
                error = np.linalg.norm(pos - target)
                
                if error < 0.05:  # Within 50mm
                    q_deg = [base_q[0], base_q[1]+dq2, base_q[2]+dq3, base_q[3], base_q[4]+dq5, 0]
                    print(f"q=[{q_deg[0]:3.0f}, {q_deg[1]:3.0f}, {q_deg[2]:3.0f}, "
                          f"{q_deg[3]:3.0f}, {q_deg[4]:3.0f}, {q_deg[5]:3.0f}]° "
                          f"→ [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]  Error: {error*1000:5.1f}mm")
                
                if error < best_error:
                    best_error = error
                    best_config = ([base_q[0], base_q[1]+dq2, base_q[2]+dq3, base_q[3], base_q[4]+dq5, 0], pos)
    
    print("-"*90)
    print(f"\n✓ FINAL BEST CONFIGURATION:")
    print(f"  Joint angles: [{best_config[0][0]:.1f}°, {best_config[0][1]:.1f}°, "
          f"{best_config[0][2]:.1f}°, {best_config[0][3]:.1f}°, "
          f"{best_config[0][4]:.1f}°, {best_config[0][5]:.1f}°]")
    print(f"  Position: [{best_config[1][0]:.3f}, {best_config[1][1]:.3f}, {best_config[1][2]:.3f}]")
    print(f"  Target:   [0.615, 0.615, 0.501]")
    print(f"  Error: {best_error*1000:.1f}mm")
    
    if best_error > 0.03:
        print(f"\n⚠ WARNING: Best error ({best_error*1000:.1f}mm) exceeds 30mm tolerance.")
        print("  The paper's q_n configuration may have a typo or use different conventions.")
    
    print("="*90 + "\n")
    
    return best_config[0]


if __name__ == '__main__':
    test_dh_parameters()
    best_angles = find_qn_configuration()
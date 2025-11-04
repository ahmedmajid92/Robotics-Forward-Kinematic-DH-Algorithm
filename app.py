from flask import Flask, render_template, request, jsonify
import numpy as np
from comau_model import create_comau_robot

# Initialize the Flask application
app = Flask(__name__)

# Create a global instance of the robot model to avoid reloading it on every request
robot = create_comau_robot()
print("COMAU Smart Six 6-1.4 robot model loaded.")
print(robot)

@app.route('/')
def index():
    """
    Serves the main HTML page.
    """
    return render_template('index.html')

@app.route('/fkine', methods=['POST'])
def fkine():
    """
    Forward kinematics endpoint.
    Input: JSON with q1-q6 in radians
    Output: 4x4 transformation matrices for all 7 frames (base + 6 joints)
    """
    data = request.get_json()
    q = np.array([
        data.get('q1', 0),
        data.get('q2', 0),
        data.get('q3', 0),
        data.get('q4', 0),
        data.get('q5', 0),
        data.get('q6', 0)
    ])
    
    try:
        # Get all link transforms
        all_transforms = robot.fkine_all(q)
        
        # Convert to list of 4x4 matrices
        transforms_list = [T.A.tolist() for T in all_transforms]
        
        # Ensure we have exactly 7 transforms (base + 6 links)
        if len(transforms_list) > 7:
            transforms_list = transforms_list[:7]
        elif len(transforms_list) < 7:
            # Add identity matrix as base if needed
            transforms_list.insert(0, np.eye(4).tolist())
        
        return jsonify(transforms=transforms_list)
    
    except Exception as e:
        import traceback
        print(f"Error in FK calculation: {e}")
        print(traceback.format_exc())
        return jsonify(error=str(e)), 500

if __name__ == '__main__':
    # Run the app in debug mode, which provides helpful error messages
    app.run(debug=True)
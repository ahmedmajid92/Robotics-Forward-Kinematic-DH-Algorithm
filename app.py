"""
Flask Web Application for COMAU Smart Six 6-1.4 Robot Simulator

This module provides a web-based interface for visualizing and controlling
the COMAU Smart Six 6-1.4 robot. It serves both the HTML interface and
provides a RESTful API endpoint for forward kinematics calculations.

The application uses:
- Flask: Web framework for serving pages and handling API requests
- NumPy: Numerical operations for joint angle arrays
- comau_model: Custom module containing the robot's DH parameters

API Endpoints:
- GET  /       : Serves the main HTML page with 3D visualization
- POST /fkine  : Calculates forward kinematics for given joint angles
"""

# Import required libraries
from flask import Flask, render_template, request, jsonify
import numpy as np
from comau_model import create_comau_robot

# ===== FLASK APPLICATION INITIALIZATION =====
# Create a Flask application instance
# __name__ tells Flask where to look for templates, static files, etc.
app = Flask(__name__)

# ===== ROBOT MODEL INITIALIZATION =====
# Create a single global instance of the robot model
# This is done once at startup to avoid recreating the model on every request
# Global instance improves performance by reusing the same robot object
robot = create_comau_robot()

# Print confirmation that robot model is loaded (appears in console)
print("COMAU Smart Six 6-1.4 robot model loaded.")
# Print the robot's DH parameter table for verification
print(robot)


# ===== ROUTE 1: HOME PAGE =====
@app.route('/')
def index():
    """
    Serves the main HTML page with 3D robot visualization.
    
    This route handles GET requests to the root URL (/) and returns
    the index.html template which contains the interactive 3D interface.
    
    Returns:
        str: Rendered HTML content from templates/index.html
        
    Example:
        Browser navigation to http://127.0.0.1:5000/ will trigger this function
    """
    # render_template() looks for 'index.html' in the 'templates/' folder
    # Flask automatically handles the file path resolution
    return render_template('index.html')


# ===== ROUTE 2: FORWARD KINEMATICS API =====
@app.route('/fkine', methods=['POST'])
def fkine():
    """
    Forward kinematics calculation endpoint.
    
    This RESTful API endpoint accepts joint angles and returns transformation
    matrices for all robot frames (base + 6 joints = 7 total frames).
    
    Request Format (JSON):
        {
            "q1": 0.0,  // Joint 1 angle in radians
            "q2": 0.0,  // Joint 2 angle in radians
            "q3": 0.0,  // Joint 3 angle in radians
            "q4": 0.0,  // Joint 4 angle in radians
            "q5": 0.0,  // Joint 5 angle in radians
            "q6": 0.0   // Joint 6 angle in radians
        }
    
    Response Format (JSON):
        {
            "transforms": [
                [[r11, r12, r13, x],   // 4x4 transformation matrix for frame 0
                 [r21, r22, r23, y],
                 [r31, r32, r33, z],
                 [0,   0,   0,   1]],
                // ... 6 more 4x4 matrices for frames 1-6
            ]
        }
    
    Error Response (JSON):
        {
            "error": "Error description string"
        }
    
    Returns:
        JSON: List of 7 transformation matrices or error message
        
    HTTP Status Codes:
        200: Success - returns transformation matrices
        500: Internal Server Error - returns error message
        
    Example:
        POST http://127.0.0.1:5000/fkine
        Body: {"q1":0,"q2":0,"q3":0,"q4":0,"q5":0,"q6":0}
    """
    # ===== PARSE REQUEST DATA =====
    # Extract JSON data from the POST request body
    # request.get_json() automatically parses JSON string to Python dictionary
    data = request.get_json()
    
    # ===== BUILD JOINT ANGLE ARRAY =====
    # Extract joint angles from the received data
    # data.get('q1', 0) returns the value for 'q1', or 0 if 'q1' is missing
    # This provides default values and prevents KeyError exceptions
    q = np.array([
        data.get('q1', 0),  # Joint 1: Base rotation (default: 0 radians)
        data.get('q2', 0),  # Joint 2: Shoulder pitch (default: 0 radians)
        data.get('q3', 0),  # Joint 3: Elbow bend (default: 0 radians)
        data.get('q4', 0),  # Joint 4: Wrist roll (default: 0 radians)
        data.get('q5', 0),  # Joint 5: Wrist pitch (default: 0 radians)
        data.get('q6', 0)   # Joint 6: Wrist yaw (default: 0 radians)
    ])
    # Result: q is a NumPy array of shape (6,) containing joint angles in radians
    
    # ===== CALCULATE FORWARD KINEMATICS =====
    try:
        # Attempt to calculate forward kinematics
        # This is wrapped in try-except to catch any computation errors
        
        # robot.fkine_all(q) calculates transformation matrices from base to each joint
        # Returns a list of SE3 objects (spatial transformation matrices)
        # Each SE3 object represents the pose (position + orientation) of a frame
        all_transforms = robot.fkine_all(q)
        
        # ===== CONVERT TO SERIALIZABLE FORMAT =====
        # Convert SE3 objects to plain Python lists for JSON serialization
        # T.A extracts the 4x4 NumPy array from the SE3 object
        # .tolist() converts NumPy array to nested Python lists
        transforms_list = [T.A.tolist() for T in all_transforms]
        
        # ===== ENSURE CORRECT NUMBER OF FRAMES =====
        # The robot should have exactly 7 frames (base + 6 joints)
        # Handle edge cases where the number might differ
        
        if len(transforms_list) > 7:
            # If we somehow got more than 7 transforms, keep only first 7
            # This shouldn't happen but provides safety
            transforms_list = transforms_list[:7]
            
        elif len(transforms_list) < 7:
            # If we got fewer than 7 transforms, add identity matrix as base
            # Identity matrix represents the world/base frame (no transformation)
            # np.eye(4) creates a 4x4 identity matrix: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
            transforms_list.insert(0, np.eye(4).tolist())
        
        # ===== RETURN SUCCESS RESPONSE =====
        # jsonify() converts Python dictionary to JSON format
        # Flask automatically sets Content-Type: application/json header
        return jsonify(transforms=transforms_list)
    
    # ===== ERROR HANDLING =====
    except Exception as e:
        # Catch any exception that occurs during forward kinematics calculation
        
        # Import traceback module for detailed error information
        import traceback
        
        # Print error message to console (useful for debugging)
        print(f"Error in FK calculation: {e}")
        
        # Print full stack trace to console (shows exactly where error occurred)
        print(traceback.format_exc())
        
        # Return error response to client
        # jsonify(error=str(e)) creates JSON: {"error": "error message"}
        # The second parameter 500 sets HTTP status code to "500 Internal Server Error"
        return jsonify(error=str(e)), 500


# ===== APPLICATION ENTRY POINT =====
# This block only runs when the script is executed directly (not imported)
if __name__ == '__main__':
    # Start the Flask development server
    # Parameters:
    #   debug=True: Enables debug mode which provides:
    #     - Automatic reloading when code changes
    #     - Detailed error messages in browser
    #     - Interactive debugger for exceptions
    #   Default host: 127.0.0.1 (localhost - only accessible from this computer)
    #   Default port: 5000
    #
    # To access: Open browser and navigate to http://127.0.0.1:5000
    #
    # WARNING: Never use debug=True in production! It's a security risk.
    app.run(debug=True)
    
    # Alternative production configuration (commented out):
    # app.run(host='0.0.0.0', port=5000, debug=False)
    #   host='0.0.0.0': Makes server accessible from other devices on network
    #   port=5000: Port number for the web server
    #   debug=False: Disables debug mode for security
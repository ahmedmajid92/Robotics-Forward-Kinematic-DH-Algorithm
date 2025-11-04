// --- Scene Setup ---
const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x263238);
const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
camera.position.set(2.5, 2, 2.5);
camera.lookAt(0, 0.5, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(container.clientWidth, container.clientHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container.appendChild(renderer.domElement);

// --- Lighting ---
const ambientLight = new THREE.AmbientLight(0x404040, 1.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 1.2);
directionalLight.position.set(5, 10, 5);
directionalLight.castShadow = true;
directionalLight.shadow.camera.near = 0.1;
directionalLight.shadow.camera.far = 50;
scene.add(directionalLight);

const pointLight = new THREE.PointLight(0xffffff, 0.5);
pointLight.position.set(-5, 5, -5);
scene.add(pointLight);

// --- Grid and Axes ---
const gridHelper = new THREE.GridHelper(5, 20, 0x37474f, 0x37474f);
scene.add(gridHelper);

const axesHelper = new THREE.AxesHelper(0.8);
scene.add(axesHelper);

// --- Robot Representation with Cylinders ---
const linkFrames = [];
const linkCylinders = [];
const jointSpheres = [];

// Link dimensions (radius, approximate lengths)
const linkDimensions = [
    { radius: 0.06, length: 0.45 },  // Base
    { radius: 0.05, length: 0.59 },  // Upper arm
    { radius: 0.04, length: 0.13 },  // Elbow
    { radius: 0.035, length: 0.674 }, // Forearm
    { radius: 0.03, length: 0.05 },  // Wrist 1
    { radius: 0.025, length: 0.095 }  // Wrist 2
];

// Function to create robotic gripper hand
function createRoboticGripper() {
    const gripperGroup = new THREE.Group();
    
    // Base/Mounting plate
    const baseGeometry = new THREE.CylinderGeometry(0.045, 0.045, 0.02, 20);
    const baseMaterial = new THREE.MeshPhongMaterial({ 
        color: 0x455A64,
        shininess: 80
    });
    const base = new THREE.Mesh(baseGeometry, baseMaterial);
    base.castShadow = true;
    gripperGroup.add(base);
    
    // Palm/Wrist connector
    const palmGeometry = new THREE.BoxGeometry(0.06, 0.08, 0.04);
    const palmMaterial = new THREE.MeshPhongMaterial({ 
        color: 0x546E7A,
        shininess: 70
    });
    const palm = new THREE.Mesh(palmGeometry, palmMaterial);
    palm.position.z = 0.06;
    palm.castShadow = true;
    gripperGroup.add(palm);
    
    // Gripper jaws (left and right)
    const jawMaterial = new THREE.MeshPhongMaterial({ 
        color: 0x37474F,
        shininess: 90
    });
    
    // Left jaw
    const leftJawGroup = new THREE.Group();
    
    // Jaw base
    const jawBaseGeometry = new THREE.BoxGeometry(0.025, 0.04, 0.02);
    const leftJawBase = new THREE.Mesh(jawBaseGeometry, jawMaterial);
    leftJawBase.castShadow = true;
    leftJawGroup.add(leftJawBase);
    
    // Jaw finger
    const jawFingerGeometry = new THREE.BoxGeometry(0.02, 0.08, 0.015);
    const leftFinger = new THREE.Mesh(jawFingerGeometry, jawMaterial);
    leftFinger.position.y = 0.06;
    leftFinger.castShadow = true;
    leftJawGroup.add(leftFinger);
    
    // Gripper pad
    const padGeometry = new THREE.BoxGeometry(0.022, 0.04, 0.008);
    const padMaterial = new THREE.MeshPhongMaterial({ 
        color: 0x263238,
        shininess: 50
    });
    const leftPad = new THREE.Mesh(padGeometry, padMaterial);
    leftPad.position.set(0, 0.08, 0.01);
    leftPad.castShadow = true;
    leftJawGroup.add(leftPad);
    
    leftJawGroup.position.set(-0.035, 0.02, 0.09);
    gripperGroup.add(leftJawGroup);
    
    // Right jaw (mirror of left)
    const rightJawGroup = leftJawGroup.clone();
    rightJawGroup.position.set(0.035, 0.02, 0.09);
    gripperGroup.add(rightJawGroup);
    
    // Guide rails
    const railGeometry = new THREE.CylinderGeometry(0.008, 0.008, 0.08, 8);
    const railMaterial = new THREE.MeshPhongMaterial({ 
        color: 0x78909C,
        shininess: 100
    });
    
    const leftRail = new THREE.Mesh(railGeometry, railMaterial);
    leftRail.rotation.x = Math.PI / 2;
    leftRail.position.set(-0.025, 0.04, 0.09);
    leftRail.castShadow = true;
    gripperGroup.add(leftRail);
    
    const rightRail = leftRail.clone();
    rightRail.position.set(0.025, 0.04, 0.09);
    gripperGroup.add(rightRail);
    
    // Actuator housing
    const actuatorGeometry = new THREE.BoxGeometry(0.05, 0.03, 0.03);
    const actuatorMaterial = new THREE.MeshPhongMaterial({ 
        color: 0xFF6F00,
        shininess: 80
    });
    const actuator = new THREE.Mesh(actuatorGeometry, actuatorMaterial);
    actuator.position.set(0, 0.025, 0.045);
    actuator.castShadow = true;
    gripperGroup.add(actuator);
    
    // Small details - screws
    const screwGeometry = new THREE.CylinderGeometry(0.004, 0.004, 0.008, 8);
    const screwMaterial = new THREE.MeshPhongMaterial({ 
        color: 0x212121,
        shininess: 100
    });
    
    const screwPositions = [
        [-0.025, 0, 0.065],
        [0.025, 0, 0.065],
        [-0.015, 0.04, 0.09],
        [0.015, 0.04, 0.09]
    ];
    
    screwPositions.forEach(pos => {
        const screw = new THREE.Mesh(screwGeometry, screwMaterial);
        screw.rotation.x = Math.PI / 2;
        screw.position.set(pos[0], pos[1], pos[2]);
        screw.castShadow = true;
        gripperGroup.add(screw);
    });
    
    return gripperGroup;
}

// Create frames and visual representations
for (let i = 0; i < 7; i++) {
    const jointGroup = new THREE.Group();
    
    // Add frame axes
    const axesSize = i === 6 ? 0.15 : 0.08;
    const frameAxes = new THREE.AxesHelper(axesSize);
    jointGroup.add(frameAxes);
    
    // Add joint sphere
    const sphereRadius = i === 0 ? 0.08 : (i === 6 ? 0.05 : 0.045);
    const sphereGeometry = new THREE.SphereGeometry(sphereRadius, 20, 20);
    const sphereMaterial = new THREE.MeshPhongMaterial({ 
        color: i === 6 ? 0x4CAF50 : (i === 0 ? 0x2196F3 : 0xFF5722),
        shininess: 100
    });
    const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    sphere.castShadow = true;
    jointGroup.add(sphere);
    jointSpheres.push(sphere);
    
    // Add robotic gripper at end-effector
    if (i === 6) {
        const gripper = createRoboticGripper();
        gripper.rotation.x = Math.PI / 2;  // Orient gripper forward
        gripper.position.z = 0.05;
        jointGroup.add(gripper);
    }
    
    linkFrames.push(jointGroup);
    scene.add(jointGroup);

    // Create cylindrical links between joints
    if (i > 0 && i < 7) {
        const dim = linkDimensions[i-1];
        const cylinderGeometry = new THREE.CylinderGeometry(dim.radius, dim.radius, 1, 16);
        const cylinderMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x607D8B,
            shininess: 60
        });
        const cylinder = new THREE.Mesh(cylinderGeometry, cylinderMaterial);
        cylinder.castShadow = true;
        cylinder.receiveShadow = true;
        linkCylinders.push(cylinder);
        scene.add(cylinder);
    }
}

// --- Controls ---
const sliders = {
    q1: document.getElementById('q1'),
    q2: document.getElementById('q2'),
    q3: document.getElementById('q3'),
    q4: document.getElementById('q4'),
    q5: document.getElementById('q5'),
    q6: document.getElementById('q6'),
};

const valueSpans = {
    q1: document.getElementById('q1_val'),
    q2: document.getElementById('q2_val'),
    q3: document.getElementById('q3_val'),
    q4: document.getElementById('q4_val'),
    q5: document.getElementById('q5_val'),
    q6: document.getElementById('q6_val'),
};

function rad2deg(rad) {
    return (rad * 180 / Math.PI).toFixed(2);
}

function getJointValues() {
    return {
        q1: parseFloat(sliders.q1.value),
        q2: parseFloat(sliders.q2.value),
        q3: parseFloat(sliders.q3.value),
        q4: parseFloat(sliders.q4.value),
        q5: parseFloat(sliders.q5.value),
        q6: parseFloat(sliders.q6.value),
    };
}

function resetJoints() {
    Object.keys(sliders).forEach(key => {
        sliders[key].value = 0;
        valueSpans[key].textContent = '0.00';
    });
    updateRobotPose();
}

function loadPreset(name) {
    const presets = {
        'home': [0, 0, 0, 0, 0, 0],
        'rotation': [90, 0, 0, 0, 0, 0],
        'singular': [0, 45, -60, 0, 60, 0],
        'numerical': [0, -45, 0, 0, 60, 0]
    };
    
    const config = presets[name];
    if (!config) return;
    
    ['q1', 'q2', 'q3', 'q4', 'q5', 'q6'].forEach((key, i) => {
        const radValue = config[i] * Math.PI / 180;
        sliders[key].value = radValue;
        valueSpans[key].textContent = config[i].toFixed(2);
    });
    
    updateRobotPose();
}

async function updateRobotPose() {
    const q = getJointValues();

    try {
        const response = await fetch('/fkine', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(q),
        });
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        
        if (data.error) {
            console.error('Backend error:', data.error);
            return;
        }

        if (!data.transforms || data.transforms.length !== 7) {
            console.error('Expected 7 transforms, got:', data.transforms?.length);
            return;
        }

        // Update frame positions and orientations
        data.transforms.forEach((matrixData, i) => {
            if (i >= linkFrames.length) return;
            
            const matrix = new THREE.Matrix4();
            const flatMatrix = matrixData.flat();
            matrix.fromArray(flatMatrix);
            matrix.transpose();
            
            linkFrames[i].position.setFromMatrixPosition(matrix);
            linkFrames[i].rotation.setFromRotationMatrix(matrix);
        });

        // Update cylindrical links between frames
        for (let i = 0; i < linkCylinders.length; i++) {
            const startPos = linkFrames[i].position;
            const endPos = linkFrames[i + 1].position;
            
            // Position cylinder at midpoint
            const midpoint = new THREE.Vector3().addVectors(startPos, endPos).multiplyScalar(0.5);
            linkCylinders[i].position.copy(midpoint);
            
            // Calculate length and orientation
            const direction = new THREE.Vector3().subVectors(endPos, startPos);
            const length = direction.length();
            linkCylinders[i].scale.y = length;
            
            // Rotate cylinder to align with link direction
            if (length > 0.001) {
                const axis = new THREE.Vector3(0, 1, 0);
                linkCylinders[i].quaternion.setFromUnitVectors(axis, direction.normalize());
            }
        }

        // Update end-effector position display
        const eePos = linkFrames[6].position;
        document.getElementById('ee_x').textContent = eePos.x.toFixed(3);
        document.getElementById('ee_y').textContent = eePos.y.toFixed(3);
        document.getElementById('ee_z').textContent = eePos.z.toFixed(3);

    } catch (error) {
        console.error('Error fetching kinematics:', error);
    }
}

// Add event listeners
Object.keys(sliders).forEach(key => {
    sliders[key].addEventListener('input', () => {
        valueSpans[key].textContent = rad2deg(sliders[key].value);
        updateRobotPose();
    });
});

// --- Animation Loop ---
let controls;

function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    renderer.render(scene, camera);
}

// --- Initial Setup ---
window.addEventListener('resize', () => {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}, false);

window.addEventListener('load', () => {
    if (typeof THREE.OrbitControls !== 'undefined') {
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.target.set(0, 0.5, 0);
    }
    
    updateRobotPose();
    animate();
});
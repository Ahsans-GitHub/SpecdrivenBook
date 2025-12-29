export interface Question {
  id: string;
  text: string;
  options: string[];
  correctIndex: number;
  feedback: {
    correct: string;
    incorrect: string;
  };
}

export interface QuizData {
  chapterId: string;
  questions: Question[];
}

const quizData: Record<string, QuizData> = {
  'chapter2': {
    chapterId: 'chapter2',
    questions: [
      {
        id: 'q1',
        text: 'What is the primary communication pattern for continuous high-frequency data like sensor streams?',
        options: ['Services (Client/Server)', 'Actions (Goal/Result)', 'Topics (Publish/Subscribe)', 'Parameters'],
        correctIndex: 2,
        feedback: {
          correct: 'Correct! Topics are optimized for one-to-many, continuous asynchronous data streams like 100Hz IMU data. This ensures the robot always has the latest state without waiting for a request.',
          incorrect: 'Incorrect. Services are blocking/synchronous and meant for quick requests. Topics are the standard for streaming sensor data.'
        }
      },
      {
        id: 'q2',
        text: 'Which ROS 2 concept allows you to change a value like "max_speed" at runtime without recompiling code?',
        options: ['Messages', 'Parameters', 'Topics', 'Namespaces'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes! Parameters are typed key-value pairs used for runtime configuration. They are essential for tuning robot behavior in different environments.',
          incorrect: 'No. Messages define data structure, and topics move data. Parameters specifically handle configuration values.'
        }
      },
      {
        id: 'q3',
        text: 'Why is "colcon build" used instead of simply running python scripts in a production robot environment?',
        options: ['To compile C++ nodes and link dependencies/resources', 'To make the code run faster', 'To download internet packages', 'It is only for documentation'],
        correctIndex: 0,
        feedback: {
          correct: 'Right. Colcon processes the package.xml, handles dependencies, and installs scripts/resources to the install/ folder, creating a portable and reproducible environment.',
          incorrect: 'While performance is a factor for C++, colcon\'s primary job is build system orchestration and dependency management.'
        }
      },
      {
        id: 'q4',
        text: 'What is the primary purpose of a ROS 2 Launch file?',
        options: ['To compile the code', 'To start and configure multiple nodes simultaneously', 'To visualize the robot in 3D', 'To manage git branches'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. Launch files (usually Python-based) orchestrate the startup of the entire system, managing dependencies and parameters.',
          incorrect: 'Compilation is handled by Colcon. Visualization is handled by Rviz. Launch files are for process orchestration.'
        }
      },
      {
        id: 'q5',
        text: 'In a defensive ROS 2 node, what should a subscriber do if it receives "NaN" (Not a Number) from an IMU sensor?',
        options: ['Crash the node immediately', 'Pass it directly to the motor controller', 'Log a warning and discard/filter the message', 'Ignore the IMU forever'],
        correctIndex: 2,
        feedback: {
          correct: 'Safety first! Never pass invalid data to actuators. Logging the failure helps developers while filtering prevents the robot from falling or moving erratically.',
          incorrect: 'Passing NaN to motors causes undefined behavior and likely hardware damage. Graceful filtering is the defensive standard.'
        }
      },
      {
        id: 'q6',
        text: 'What does DDS stand for in the context of ROS 2?',
        options: ['Data Distribution Service', 'Digital Device System', 'Direct Driver Simulation', 'Distributed Data Socket'],
        correctIndex: 0,
        feedback: {
          correct: 'Correct. DDS is the industrial-grade decentralized communication standard that ROS 2 uses for discovery and message passing.',
          incorrect: 'DDS is the middleware standard. It provides the "plumbing" for ROS 2 nodes to find each other.'
        }
      },
      {
        id: 'q7',
        text: 'Which QoS (Quality of Service) policy is best for a high-bandwidth video stream where speed is more important than missing a single frame?',
        options: ['Reliable', 'Best Effort', 'Transient Local', 'Volatile'],
        correctIndex: 1,
        feedback: {
          correct: 'Best Effort is ideal here. It sends data without retries, minimizing latency. If a frame is lost, the next one arrives immediately.',
          incorrect: 'Reliable QoS would try to re-send lost video frames, causing a massive backlog and latency (the "lag" effect).'
        }
      },
      {
        id: 'q8',
        text: 'In a Python package structure, which file contains the list of dependencies and metadata?',
        options: ['setup.py', 'package.xml', 'CMakeLists.txt', 'README.md'],
        correctIndex: 1,
        feedback: {
          correct: 'Exactly. package.xml is the manifest that tells ROS 2 what other packages are required to run your code.',
          incorrect: 'setup.py handles installation logic, but package.xml is the formal dependency manifest for the ROS ecosystem.'
        }
      },
      {
        id: 'q9',
        text: 'What happens when you "remap" a topic during launch?',
        options: ['You delete the old topic', 'You change the topic name the node uses without changing its code', 'You increase the bandwidth', 'You change the message type'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes! Remapping allows modularity. A generic "camera" node can be used for "left_eye" or "right_eye" topics just by remapping in launch.',
          incorrect: 'Remapping is just a name change in the graph; it doesn\'t affect the code or the message structure.'
        }
      },
      {
        id: 'q10',
        text: 'Which state in a Lifecycle Node indicates the node is active and processing data?',
        options: ['Unconfigured', 'Inactive', 'Active', 'Finalized'],
        correctIndex: 2,
        feedback: {
          correct: 'Correct. The "Active" state means the node is fully functional. Lifecycle nodes allow deterministic system startup sequences.',
          incorrect: 'Inactive nodes are configured but not running callbacks. Active nodes are the ones doing the work.'
        }
      },
      {
        id: 'q11',
        text: 'What is the standard command to check the list of all active topics on a robot?',
        options: ['ros2 topic list', 'ros2 node info', 'ros2 msg show', 'ros2 run'],
        correctIndex: 0,
        feedback: {
          correct: 'Correct. "ros2 topic list" is the most common introspection tool used by roboticists.',
          incorrect: 'Node info shows node details. Msg show shows message structure. Topic list shows the graph buses.'
        }
      },
      {
        id: 'q12',
        text: 'Why should you avoid synchronous (blocking) service calls in a balance loop?',
        options: ['They use too much RAM', 'They can cause deadlocks if the server is in the same thread', 'They are illegal in Python', 'They make the robot too fast'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. If a balance loop waits for a service that never responds, the robot will fall because it stopped processing motor updates.',
          incorrect: 'It\'s about timing and deadlocks. Asynchronous calls keep the execution flow non-blocking and safe.'
        }
      },
      {
        id: 'q13',
        text: 'Which file defines the entry points (executables) for a Python ROS 2 package?',
        options: ['package.xml', 'setup.py', 'MANIFEST.in', 'config.yaml'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes! setup.py uses "console_scripts" to map terminal commands to specific Python functions.',
          incorrect: 'package.xml is for dependencies. setup.py is where the installation and executable mapping occurs.'
        }
      },
      {
        id: 'q14',
        text: 'What is the default language for writing ROS 2 launch files?',
        options: ['XML', 'YAML', 'Python', 'C++'],
        correctIndex: 2,
        feedback: {
          correct: 'Python is the default and most powerful choice, allowing for complex logic and conditional launching.',
          incorrect: 'While XML and YAML are supported, Python is the standard for its flexibility and logic capabilities.'
        }
      },
      {
        id: 'q15',
        text: 'What is the purpose of SROS 2?',
        options: ['Faster Simulation', 'Security (Encryption and Authentication) for ROS 2', 'Support for old ROS 1 nodes', 'Super ROS mode'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. SROS 2 provides tools to enable DDS security features like encryption to prevent robot hijacking.',
          incorrect: 'It\'s all about security. As robots move into public spaces, SROS 2 becomes a mandatory defensive layer.'
        }
      }
    ]
  },
  'chapter3': {
    chapterId: 'chapter3',
    questions: [
      {
        id: 'c3q1',
        text: 'Which XML-based format is the ROS 2 standard for describing a robot\'s kinematics and joint structure?',
        options: ['SDF', 'URDF', 'JSON', 'YAML'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! URDF (Unified Robot Description Format) is the tree-based standard for ROS 2 kinematics.',
          incorrect: 'Incorrect. SDF is the Gazebo standard. URDF is the core ROS standard for kinematics.'
        }
      },
      {
        id: 'c3q2',
        text: 'What is the "Kraken" effect in robot simulation?',
        options: ['A water-based robot model', 'Violent vibrations caused by unstable physics steps', 'A high-speed communication protocol', 'The robot falling over'],
        correctIndex: 1,
        feedback: {
          correct: 'Exactly. It usually happens when the physics time step is too large for the joint complexity, causing mathematical divergence.',
          incorrect: 'No. While falling over is common, the "Kraken" refers to physics-induced unstable oscillations.'
        }
      },
      {
        id: 'c3q3',
        text: 'Why should collision geometry be simpler than visual geometry?',
        options: ['To save disk space', 'To make the robot look better', 'To drastically improve physics computation speed', 'It should not; they should be identical'],
        correctIndex: 2,
        feedback: {
          correct: 'Correct! Simple primitives (boxes, cylinders) are much faster to calculate for contact than complex high-poly meshes.',
          incorrect: 'Incorrect. Identical collision meshes will likely crash or crawl the simulation speed due to complex contact point math.'
        }
      },
      {
        id: 'c3q4',
        text: 'What is the primary coordinate system difference between Unity and ROS 2?',
        options: ['Unity is metric, ROS is imperial', 'Unity is Left-Handed (Y-up), ROS is Right-Handed (Z-up)', 'There is no difference', 'ROS is 2D, Unity is 3D'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! This shift requires careful coordinate transformation to prevent the robot appearing on its side when visualized in Unity.',
          incorrect: 'Incorrect. Both are metric and 3D, but the handedness and "up" axis differ (Z-up in ROS vs Y-up in Unity).'
        }
      },
      {
        id: 'c3q5',
        text: 'In simulation, what does "Ground Truth" mean?',
        options: ['A floor texture', 'The actual, perfect state of the robot (position/velocity) without sensor noise', 'The robot\'s moral compass', 'Documentation files'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes! Simulation knows exactly where the robot is, which is perfect for verifying if your sensors (like a noisy GPS) are lying.',
          incorrect: 'Incorrect. Ground truth refers to the idealized, noise-free state data available only in sim.'
        }
      },
      {
        id: 'c3q6',
        text: 'What is the purpose of Xacro in URDF generation?',
        options: ['To speed up rendering', 'To use variables, math, and macros to create cleaner, modular robot descriptions', 'To simulate water', 'To compress files'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. Xacro (XML Macros) allows you to define a "finger" once and reuse it 10 times, making the description maintainable.',
          incorrect: 'Xacro is a pre-processor for URDF. It makes writing complex robots like humanoids much easier.'
        }
      },
      {
        id: 'c3q7',
        text: 'What happens if a URDF link has 0.0 mass but is connected to an active joint?',
        options: ['The robot floats', 'The physics engine returns NaN and crashes', 'The robot moves faster', 'Nothing changes'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. Physics engines (F=ma) cannot handle zero mass when forces are applied. It leads to division by zero.',
          incorrect: 'Inertia is non-negotiable. Even a small link must have at least a tiny mass for the solver to work.'
        }
      },
      {
        id: 'c3q8',
        text: 'Which simulator is most commonly used for open-source ROS 2 development?',
        options: ['MuJoCo', 'Gazebo', 'Unity', 'PyBullet'],
        correctIndex: 1,
        feedback: {
          correct: 'Gazebo is the industry standard for open-source robotics, offering seamless ROS 2 integration through plugins.',
          incorrect: 'While others are used for RL, Gazebo is the standard general-purpose simulator for ROS 2.'
        }
      },
      {
        id: 'c3q9',
        text: 'What is a "Plugin" in Gazebo?',
        options: ['A visual skin', 'A shared library that adds custom logic or emulates hardware sensors', 'A way to buy models', 'A data logger'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes. Plugins are used to simulate specific hardware like a RealSense camera or to bridge data to ROS 2 topics.',
          incorrect: 'Plugins add functionality. Without plugins, a Gazebo robot is just a static bunch of shapes.'
        }
      },
      {
        id: 'c3q10',
        text: 'In SDF, what does the <mu> tag control?',
        options: ['Color', 'Friction coefficient', 'Mass', 'Transparency'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. <mu> defines the friction between surfaces. Tuning this is critical for humanoid walking stability.',
          incorrect: 'It is the friction parameter. Mu is the standard symbol for friction in physics.'
        }
      },
      {
        id: 'c3q11',
        text: 'Why is "use_sim_time" important when running ROS nodes with a simulator?',
        options: ['It makes the code run faster', 'It syncs the node\'s clock with the simulator\'s clock instead of the real wall clock', 'It saves battery', 'It is only for C++'],
        correctIndex: 1,
        feedback: {
          correct: 'Exactly. If ROS thinks it\'s 12:00 PM but the simulator is paused at 10:00 AM, transforms (TF) will fail due to time drift.',
          incorrect: 'Syncing time is mandatory. Without it, the robot\'s perception and control will be out of phase.'
        }
      },
      {
        id: 'c3q12',
        text: 'What is a "Digital Twin"?',
        options: ['A second physical robot', 'A high-fidelity virtual model that perfectly mirrors a physical robot\'s state and behavior', 'A clone of the programmer', 'A backup of the code'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. A digital twin allows you to test scenarios in sim that are exactly identical to what the physical robot will face.',
          incorrect: 'It\'s a virtual mirror. The goal is 1:1 fidelity between sim and reality.'
        }
      },
      {
        id: 'c3q13',
        text: 'Which joint type allows 360-degree continuous rotation (like a wheel)?',
        options: ['Fixed', 'Prismatic', 'Revolute', 'Continuous'],
        correctIndex: 3,
        feedback: {
          correct: 'Correct. While "revolute" has limits, "continuous" joints are for wheels or endless rotating parts.',
          incorrect: 'Prismatic is for sliding. Revolute has limits. Continuous is for wheels.'
        }
      },
      {
        id: 'c3q14',
        text: 'What is "Restitution" in a physics engine?',
        options: ['Friction', 'Bounciness (how much energy is kept after a collision)', 'The robot\'s weight', 'Communication delay'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. High restitution makes a ball bounce; low restitution makes a robot foot "thud" on the floor.',
          incorrect: 'It measures bounciness. For humanoids, we usually want low restitution for stable footsteps.'
        }
      },
      {
        id: 'c3q15',
        text: 'What is the benefit of using Unity for visualization over Rviz?',
        options: ['Photorealistic rendering and complex HRI (Human-Robot Interaction) scenes', 'It uses less RAM', 'It is built by the ROS community', 'It only works on Linux'],
        correctIndex: 0,
        feedback: {
          correct: 'Correct. Unity provides the visual fidelity needed for synthetic data generation and human-in-the-loop tests.',
          incorrect: 'Unity is a game engine. It\'s much heavier than Rviz but offers vastly better visuals and interactive tools.'
        }
      }
    ]
  },
  'chapter4': {
    chapterId: 'chapter4',
    questions: [
      {
        id: 'c4q1',
        text: 'What is the primary advantage of NVIDIA Isaac Sim over Gazebo?',
        options: ['It is easier to install', 'It runs on MacOS', 'Massive GPU-accelerated parallel simulation', 'It does not require a GPU'],
        correctIndex: 2,
        feedback: {
          correct: 'Correct! Isaac Sim uses the GPU to simulate thousands of robots in parallel, which is essential for modern RL.',
          incorrect: 'Incorrect. Isaac Sim requires a powerful NVIDIA GPU and focuses on parallelized, high-fidelity simulation.'
        }
      },
      {
        id: 'c4q2',
        text: 'Which data format is the core of NVIDIA Omniverse and Isaac Sim?',
        options: ['XML', 'USD (Universal Scene Description)', 'CSV', 'STL'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes! USD is the industrial standard for 3D scene description used in Isaac Sim, allowing for collaborative non-destructive editing.',
          incorrect: 'No. USD is the core. It allows for hierarchical, scalable scene management beyond what XML/URDF can do.'
        }
      },
      {
        id: 'c4q3',
        text: 'What is "Domain Randomization" used for in Isaac Sim?',
        options: ['To make the simulation look pretty', 'To bridge the Sim-to-Real gap by varying physical parameters during training', 'To randomize the robot\'s name', 'To slow down the training process'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! By training in a range of "messy" virtual worlds, the robot becomes robust enough for the unpredictable real world.',
          incorrect: 'It is a technical strategy for robustness. Training on "sandpaper" and "ice" in sim makes the robot handle real carpet better.'
        }
      },
      {
        id: 'c4q4',
        text: 'What does "Zero-Copy" transport (using NITROS in Isaac ROS) achieve?',
        options: ['Saves disk space', 'Reduces perception latency by passing memory pointers instead of copying data', 'Deletes old log files', 'Increases security'],
        correctIndex: 1,
        feedback: {
          correct: 'Exactly. Minimizing latency is critical for humanoid balance. Zero-copy bypasses the CPU for faster GPU-to-GPU data movement.',
          incorrect: 'Zero-copy is about speed and efficiency. In robotics, latency is the enemy of stability.'
        }
      },
      {
        id: 'c4q5',
        text: 'Why is a "Reward Function" dangerous if not designed with defensive penalties?',
        options: ['It might cost real money', 'The AI might "cheat" (Reward Hacking) to maximize points without learning the task', 'It makes the robot too happy', 'It deletes the code'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes! A robot might learn to "vibrate" its legs to get velocity points if you don\'t penalize excessive torque.',
          incorrect: 'This is "Reward Hacking." You must penalize falls and energy waste to get a natural, safe gait.'
        }
      },
      {
        id: 'c4q6',
        text: 'What is "Isaac ROS" intended for?',
        options: ['Cloud simulation only', 'Hardware-accelerated perception packages for Jetson Orin and RTX PCs', 'Writing Python scripts', '3D Modeling'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. Isaac ROS provides optimized packages for tasks like VSLAM, stereo depth, and object detection on NVIDIA hardware.',
          incorrect: 'It\'s about hardware acceleration. It makes ROS 2 perception run 10x faster on a Jetson Orin.'
        }
      },
      {
        id: 'c4q7',
        text: 'Which tool is used to generate millions of labeled images for AI training in Isaac Sim?',
        options: ['Replicator (Synthetic Data Generation)', 'OmniGraph', 'PhysX', 'TensorRT'],
        correctIndex: 0,
        feedback: {
          correct: 'Correct! Replicator automates the creation of diverse, perfectly labeled datasets, saving thousands of hours of manual labeling.',
          incorrect: 'Replicator is for data. TensorRT is for inference. OmniGraph is for logic.'
        }
      },
      {
        id: 'c4q8',
        text: 'What is TensorRT?',
        options: ['A new type of motor', 'A high-performance deep learning inference optimizer and runtime', 'A simulator', 'A database'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes. TensorRT compiles neural networks to run at maximum speed on specific NVIDIA GPUs (like the Orin).',
          incorrect: 'It\'s an optimizer. In robotics, you use it to make your YOLO or VLA models run fast enough for real-time control.'
        }
      },
      {
        id: 'c4q9',
        text: 'What is "System Identification" (SysID) in the context of Sim-to-Real?',
        options: ['Identifying the robot\'s name', 'Measuring physical properties of the real robot to update the simulation parameters', 'Buying a new GPU', 'Changing the OS'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. SysID ensures the sim mass and friction match the physical robot, reducing the "Gap."',
          incorrect: 'It\'s an engineering process to make the simulation a more accurate "Digital Twin."'
        }
      },
      {
        id: 'c4q10',
        text: 'Which NVIDIA hardware is the primary target for deployed Physical AI humanoid brains?',
        options: ['GeForce 1050', 'Jetson Orin series', 'Apple M3', 'Raspberry Pi'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. The Jetson Orin provides the power-efficient AI compute needed for onboard humanoid control.',
          incorrect: 'Jetson Orin is the industry standard for edge robotics. It handles the high-DoF math and vision pipelines locally.'
        }
      },
      {
        id: 'c4q11',
        text: 'What does "Vectorized Simulation" (in Isaac Lab) allow you to do?',
        options: ['Draw robots as vectors', 'Simulate thousands of robots simultaneously in a single GPU memory space', 'Send data over WiFi', 'Move joints in a straight line'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. This is how Isaac Sim trains humanoids to walk in hours instead of weeks.',
          incorrect: 'It\'s about massive parallelism. "Vectorized" means the GPU treats 1000 robots as a single math operation.'
        }
      },
      {
        id: 'c4q12',
        text: 'What is the "Omniverse" in relation to Isaac Sim?',
        options: ['A type of VR goggles', 'The foundational platform for collaborative 3D simulation and USD workflows', 'A social media for robots', 'A brand of sensors'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes. Omniverse is the "Engine" that Isaac Sim runs on, providing the rendering and USD management.',
          incorrect: 'It\'s the platform. Isaac Sim is an application built on the Omniverse platform.'
        }
      },
      {
        id: 'c4q13',
        text: 'What is "PhysX"?',
        options: ['A camera brand', 'NVIDIA\'s high-fidelity physics engine for simulating rigid bodies and articulation', 'A ROS 2 command', 'A Python library'],
        correctIndex: 1,
        feedback: {
          correct: 'Exactly. PhysX handles the collisions and joint constraints that make the robot walk realistically.',
          incorrect: 'It\'s the physics engine. It calculates "Gravity + Friction + Force = Motion."'
        }
      },
      {
        id: 'c4q14',
        text: 'Which precision level is typically used for TensorRT optimizations on Jetson to save power?',
        options: ['FP64', 'FP32', 'INT8 or FP16', 'Binary'],
        correctIndex: 2,
        feedback: {
          correct: 'Correct. Quantizing to INT8 or FP16 provides massive speedups with negligible accuracy loss for most robot vision tasks.',
          incorrect: 'Robots use INT8/FP16 for speed. High precision (FP64) is too slow for real-time edge AI.'
        }
      },
      {
        id: 'c4q15',
        text: 'In RL training, what happens during the "Exploration" phase?',
        options: ['The robot looks for a map', 'The AI tries random actions to discover which ones lead to rewards', 'The robot goes on vacation', 'The code is deleted'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. Exploration is how the AI discovers that "lifting a leg" might help it move forward.',
          incorrect: 'Exploration is trial and error. The AI must try "bad" things to learn what "good" things look like.'
        }
      }
    ]
  },
  'chapter5': {
    chapterId: 'chapter5',
    questions: [
      {
        id: 'c5q1',
        text: 'What does VLA stand for in the context of autonomous robotics?',
        options: ['Virtual Logic Agent', 'Vision-Language-Action', 'Variable Linear Actuator', 'Visual Localization Accuracy'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! VLA models (like RT-2) connect visual perception and language understanding directly to physical control actions.',
          incorrect: 'VLA is the integration of Vision, Language, and Action. It allows robots to "See, Reason, and Do."'
        }
      },
      {
        id: 'c5q2',
        text: 'To stay upright, a humanoid must keep its ZMP (Zero Moment Point) inside which boundary?',
        options: ['The Pelvis', 'The Support Polygon (area under/between the feet)', 'The Head', 'The Camera View'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! As long as the ZMP is within the support polygon, the robot is statically stable and will not tip over.',
          incorrect: 'Balance is determined by the contact points with the ground. The area between the feet is the "Safety Zone" for balance.'
        }
      },
      {
        id: 'c5q3',
        text: 'What is the primary role of a Large Language Model (LLM) in a humanoid "brain"?',
        options: ['Controlling high-speed motor voltages', 'High-level task planning and semantic reasoning', 'Rendering 3D graphics', 'Calculating joint friction coefficients'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes! LLMs serve as high-level planners, breaking complex human requests like "Clean the room" into actionable robot steps.',
          incorrect: 'LLMs are too slow for millisecond motor control. They handle the "Thinking" while RL handles the "Moving."'
        }
      },
      {
        id: 'c5q4',
        text: 'Why is a "Safety Filter" mandatory when using LLM-generated robot commands?',
        options: ['To save battery power', 'To prevent AI hallucinations from causing dangerous physical movements or collisions', 'To improve speech quality', 'To make the robot more polite'],
        correctIndex: 1,
        feedback: {
          correct: 'Safety first! LLMs can hallucinate unsafe actions (e.g., "Walk through that glass window"). A classical safety layer must validate commands against a costmap.',
          incorrect: 'Safety filters are a critical defense against AI non-determinism. You never let a black-box AI have 100% control without a watchdog.'
        }
      },
      {
        id: 'c5q5',
        text: 'What is the "Linear Inverted Pendulum Model" (LIPM) used for?',
        options: ['Designing robot fingers', 'Simplifying the complex physics of humanoid balance into a manageable mathematical model', 'Simulating fluids', 'Programming voice synthesis'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! LIPM is a fundamental simplification that treats the robot as a mass on a stick, making balance math solvable in real-time.',
          incorrect: 'LIPM is the "Gold Standard" for simplified walking and balance control in humanoid robotics.'
        }
      },
      {
        id: 'c5q6',
        text: 'In bipedal walking, what is the "Double Support" phase?',
        options: ['When the robot is carrying two objects', 'When both feet are in contact with the ground simultaneously', 'When two humans are helping the robot', 'A backup power system'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. Double support is the most stable phase of walking. It occurs between steps when the weight is transferring.',
          incorrect: 'It refers to foot contact. It\'s the "safe" part of a step compared to the unstable "Single Support" phase.'
        }
      },
      {
        id: 'c5q7',
        text: 'What is a "Kinematic Singularity" in a robot arm?',
        options: ['A black hole in the code', 'A joint configuration where the math fails and the arm loses a direction of movement', 'A broken motor', 'A very fast movement'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes. Singularities happen when joints align (like a fully straight arm). The math produces "infinite" values, causing the arm to lock or jerk.',
          incorrect: 'It\'s a mathematical trap. Defensive programming means keeping joints away from their "locked" positions.'
        }
      },
      {
        id: 'c5q8',
        text: 'Which technology is best for providing a humanoid robot with high-quality local speech-to-text (STT)?',
        options: ['MP3 files', 'OpenAI Whisper', 'SQLite', 'DirectX'],
        correctIndex: 1,
        feedback: {
          correct: 'Whisper is the standard for robust, multi-lingual STT that can run locally on edge hardware like the Jetson Orin.',
          incorrect: 'Whisper is an AI model for speech. It allows the robot to "Hear" and understand intent without needing an internet connection.'
        }
      },
      {
        id: 'c5q9',
        text: 'In the Capstone Project, what is the purpose of a "Behavior Tree"?',
        options: ['To grow virtual plants', 'To manage complex, branching logic and error recovery for autonomous tasks', 'To store images', 'To calculate gravity'],
        correctIndex: 1,
        feedback: {
          correct: 'Exactly. Behavior Trees allow the robot to handle "If-Then-Else" scenarios like "If I drop the object, go back and search for it.',
          incorrect: 'Behavior Trees are the "Executive" of the robot. They coordinate the interaction, perception, and control layers.'
        }
      },
      {
        id: 'c5q10',
        text: 'What is "Compliance" in robotic manipulation?',
        options: ['Following all laws', 'The ability of a robot arm to "give" or be flexible when it hits an obstacle', 'Staying within the support polygon', 'A type of software license'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. Compliance prevents the robot from breaking itself or the object it is touching. It makes HRI safe for humans.',
          incorrect: 'Compliance is "Softness." A compliant robot arm feels like a human arm, not a rigid steel bar.'
        }
      },
      {
        id: 'c5q11',
        text: 'Which Degrees of Freedom (DoF) count is standard for a high-performance humanoid arm?',
        options: ['1 DoF', '3 DoF', '7 DoF', '50 DoF'],
        correctIndex: 2,
        feedback: {
          correct: '7 DoF is the standard. It mimics the human arm (Shoulder:3, Elbow:1, Wrist:3), allowing the robot to reach a point from any angle.',
          incorrect: '7 DoF is the magic number. It provides "Redundancy," meaning the robot can move its elbow without moving its hand.'
        }
      },
      {
        id: 'c5q12',
        text: 'What is a "Deictic Gesture" in HRI?',
        options: ['A dance move', 'A pointing gesture used to identify an object ("Pick *that* up")',
'A wave', 'A smile'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. Deictic gestures are critical for resolving ambiguity. "That" + "Pointing" = Clear Intent.',
          incorrect: 'It refers to pointing. It\'s how humans identify objects in 3D space without knowing their names.'
        }
      },
      {
        id: 'c5q13',
        text: 'How does a robot calculate "Distance Remaining" during an Action goal?',
        options: ['By asking the user', 'By comparing current GPS/Odometry to the target coordinates', 'It guesses', 'It doesn\'t need to know'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. This feedback is sent back to the user or the high-level planner to monitor progress.',
          incorrect: 'Odometry is the key. The robot uses its "Internal Map" to see how far it has traveled.'
        }
      },
      {
        id: 'c5q14',
        text: 'What is "Prompt Injection" in the context of conversational robotics?',
        options: ['Giving the robot a vaccine', 'A malicious user command designed to override the robot\'s safety persona', 'A fast way to type', 'A type of hardware port'],
        correctIndex: 1,
        feedback: {
          correct: 'Safety first! Users might try to "Hack" the robot brain by saying "Ignore your safety rules." Sanitization is mandatory.',
          incorrect: 'It\'s a cyber-security threat. We must protect the robot\'s "Executive Brain" from being tricked by text commands.'
        }
      },
      {
        id: 'c5q15',
        text: 'What is the "Capture Point" (DCM)?',
        options: ['A photo spot', 'The point on the ground where the robot should step to come to a complete stop', 'The robot\'s home base', 'The center of the camera'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. The Capture Point is a dynamic balance concept. If you can step on it, you won\'t fall.',
          incorrect: 'It\'s a balance metric. It tells the robot where to put its foot to "Capture" its momentum and stop falling.'
        }
      }
    ]
  },
  'chapter7': {
    chapterId: 'chapter7',
    questions: [
      {
        id: 'c7q1',
        text: 'What is the "Reality Gap" in humanoid development?',
        options: ['The distance between the robot and user', 'The difference between simulated physics and real-world behavior', 'The time it takes to build a robot', 'The cost of hardware'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! Bridging this gap is the central challenge. Success requires domain randomization and accurate system identification.',
          incorrect: 'No. The reality gap is why code that works in sim often fails on the real floor.'
        }
      },
      {
        id: 'c7q2',
        text: 'In an integrated humanoid behavior loop, which layer is responsible for translating voice into JSON function calls?',
        options: ['Control Layer', 'Interaction Layer (Whisper/LLM)', 'Perception Layer', 'Planning Layer'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes! The Interaction Layer handles the high-level "Brain" that interprets human intent.',
          incorrect: 'Control is for the muscles (motors). Interaction is for the mind (LLM/Whisper).'
        }
      },
      {
        id: 'c7q3',
        text: 'Which technology provides the hardware-accelerated "Gems" used for high-performance vision on the Jetson Orin?',
        options: ['Gazebo', 'Isaac ROS', 'MoveIt 2', 'Turtlesim'],
        correctIndex: 1,
        feedback: {
          correct: 'Exactly. Isaac ROS packages use the Jetson\'s GPU and NvMedia hardware to process vision with minimal latency.',
          incorrect: 'Isaac ROS is the performance standard for NVIDIA-based robotics perception.'
        }
      },
      {
        id: 'c7q4',
        text: 'What is the purpose of a Behavior Tree in the final Capstone project?',
        options: ['To store training datasets', 'To manage states and handle multi-step error recovery logic', 'To render 3D vegetation', 'To calculate battery voltage'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! Behavior Trees provide the "Executive Logic" to handle complex, branching tasks like "Grasp, and if you fail, search again.',
          incorrect: 'BTs are for decision making. They are much more robust than simple "If-Else" chains for complex humanoid tasks.'
        }
      },
      {
        id: 'c7q5',
        text: 'Why should a balance controller be isolated in a dedicated Real-time Executor?',
        options: ['To save power', 'To prevent high-level AI reasoning (like LLMs) from blocking critical 500Hz motor updates', 'To make the code shorter', 'To hide the control logic'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! Any jitter in the balance loop will make the robot fall. Real-time isolation guarantees the motors get updated on time.',
          incorrect: 'Timing is everything. You cannot let a "Thinking" process delay a "Balancing" process.'
        }
      },
      {
        id: 'c7q6',
        text: 'What is "Perception Age" and why does it matter?',
        options: ['How old the camera is', 'The total latency from when a photon hits the sensor to when the data reaches the controller', 'The robot\'s lifespan', 'The time it takes to train an AI'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. If perception age is 100ms, the robot is seeing where objects *were* 100ms ago. This must be compensated for in dynamic tasks.',
          incorrect: 'It is the latency measurement. In high-speed robotics, even 50ms of "Perception Age" can cause a missed catch or a collision.'
        }
      },
      {
        id: 'c7q7',
        text: 'Which precision format is preferred for deploying YOLO detection on a Jetson Orin to maximize FPS?',
        options: ['FP64', 'FP32', 'INT8 (via TensorRT)', 'String'],
        correctIndex: 2,
        feedback: {
          correct: 'Correct. INT8 precision provides the fastest inference speeds on NVIDIA hardware with minimal impact on accuracy.',
          incorrect: 'Speed is key. INT8 allows the Jetson to process multiple camera feeds simultaneously.'
        }
      },
      {
        id: 'c7q8',
        text: 'What is "Multi-modal Consensus" in defensive perception?',
        options: ['Using two cameras', 'Requiring multiple sensors (e.g. Lidar AND Camera) to agree before believing an object exists', 'Asking the user for help', 'Running the same AI twice'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. This prevents "Ghost Detections." If the camera sees a person but the Lidar sees nothing, it might be a poster or a reflection.',
          incorrect: 'Consensus is a safety strategy. It prevents the robot from reacting to AI hallucinations or sensor noise.'
        }
      },
      {
        id: 'c7q9',
        text: 'What is the role of NVBlox in the Isaac ROS pipeline?',
        options: ['To play music', 'To build a high-resolution 3D occupancy map of the environment in real-time', 'To simulate joints', 'To write log files'],
        correctIndex: 1,
        feedback: {
          correct: 'Yes. NVBlox uses depth data to build a 3D "Memory" of the room, allowing the robot to navigate safely.',
          incorrect: 'NVBlox is for 3D mapping. It turns flat images into a 3D world the robot can understand.'
        }
      },
      {
        id: 'c7q10',
        text: 'In the Capstone, what should the robot do if the "Battery Low" signal is received mid-task?',
        options: ['Finish the task first', 'Fail-Safe: Interrupt the task, safely stow arms, and navigate to the charger', 'Ignore it', 'Shut down immediately'],
        correctIndex: 1,
        feedback: {
          correct: 'Safety first! A humanoid that dies mid-walk will fall and break. Low battery must be a high-priority interrupt.',
          incorrect: 'Robots must be self-preserving. Navigating to a charger is a foundational autonomous behavior.'
        }
      },
      {
        id: 'c7q11',
        text: 'What is the "Support Polygon" for a humanoid standing on one leg?',
        options: ['The whole room', 'The area of the single foot in contact with the ground', 'A triangle between the head and feet', 'There is no support polygon'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. When on one leg, the polygon is tiny. This is why "Single Support" is the most dangerous phase of walking.',
          incorrect: 'The support polygon is strictly defined by the points of contact. One foot = very little room for error.'
        }
      },
      {
        id: 'c7q12',
        text: 'What is "Domain Randomization" during simulation training?',
        options: ['To make the training look more like a video game', 'To ensure the AI policy is robust to unknown real-world friction and mass variations', 'To save money', 'To make the simulation slower'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct! By training on "1000 different worlds," the AI learns to generalize to the "1 real world.',
          incorrect: 'It\'s a Sim-to-Real strategy. Without randomization, the AI becomes "overfit" to the perfect simulator physics.'
        }
      },
      {
        id: 'c7q13',
        text: 'What is the primary risk of "Prompt Injection" in a robot?',
        options: ['The robot gets a virus', 'An unauthorized user tricks the robot into bypassing safety constraints (e.g. "Run at full speed")',
'The robot stops talking', 'The battery drains faster'],
        correctIndex: 1,
        feedback: {
          correct: 'Correct. Defending against malicious language input is as important as defending against bad sensor data.',
          incorrect: 'It is a security risk. Language models are powerful but can be manipulated by clever users.'
        }
      },
      {
        id: 'c7q14',
        text: 'Which ROS 2 tool allows you to visualize the robot\'s internal "Brain" graph?',
        options: ['Rviz', 'RQT Graph', 'Gazebo', 'Turtlesim'],
        correctIndex: 1,
        feedback: {
          correct: 'RQT Graph shows you every node and how data is flowing between them. It is essential for debugging integration issues.',
          incorrect: 'Rviz is for 3D data. RQT Graph is for the logical structure of the software.'
        }
      },
      {
        id: 'c7q15',
        text: 'Final Mastery Check: What is the most important rule in Physical AI engineering?',
        options: ['Make it fast', 'Make it look human', 'Safety and Robustness through Defensive Programming', 'Use the newest AI model'],
        correctIndex: 2,
        feedback: {
          correct: 'Correct. In the physical world, errors have consequences. Every line of code must be written with safety and hardware integrity in mind.',
          incorrect: 'Safety is #1. Speed and aesthetics are secondary to a robot that doesn\'t break itself or hurt others.'
        }
      }
    ]
  }
};

export const getQuizData = (chapterId: string): QuizData | null => {
  return quizData[chapterId] || null;
};
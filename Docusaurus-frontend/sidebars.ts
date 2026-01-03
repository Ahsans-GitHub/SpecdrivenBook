import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  mainSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapter 1: Introduction to Physical AI',
      collapsible: true,
      collapsed: false,
      items: [
        'chapter1/lesson1',
        'chapter1/lesson2',
        'chapter1/lesson3',
        'chapter1/lesson4',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: ROS 2 Fundamentals',
      link: {
        type: 'doc',
        id: 'chapter2/chapter2-overview',
      },
      items: [
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          link: {
            type: 'doc',
            id: 'chapter2/module1-overview',
          },
          items: [
            {
              type: 'doc',
              id: 'chapter2/module1/lesson1',
              label: 'ROS 2 architecture and core concepts',
            },
            {
              type: 'doc',
              id: 'chapter2/module1/lesson2',
              label: 'Nodes, topics, services, and actions',
            },
            {
              type: 'doc',
              id: 'chapter2/module1/lesson3',
              label: 'Building ROS 2 packages with Python',
            },
            {
              type: 'doc',
              id: 'chapter2/module1/lesson4',
              label: 'Launch files and parameter management',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 3: Robot Simulation with Gazebo',
      link: {
        type: 'doc',
        id: 'chapter3/chapter3-overview',
      },
      items: [
        {
          type: 'category',
          label: 'Module 2: The Digital Twin (Gazebo & Unity)',
          link: {
            type: 'doc',
            id: 'chapter3/module2-overview',
          },
          items: [
            {
              type: 'doc',
              id: 'chapter3/module2/lesson1',
              label: 'Gazebo simulation environment setup',
            },
            {
              type: 'doc',
              id: 'chapter3/module2/lesson2',
              label: 'URDF and SDF robot description formats',
            },
            {
              type: 'doc',
              id: 'chapter3/module2/lesson3',
              label: 'Physics simulation and sensor simulation',
            },
            {
              type: 'doc',
              id: 'chapter3/module2/lesson4',
              label: 'Introduction to Unity for robot visualization',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 4: NVIDIA Isaac Platform',
      link: {
        type: 'doc',
        id: 'chapter4/chapter4-overview',
      },
      items: [
        {
          type: 'category',
          label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
          link: {
            type: 'doc',
            id: 'chapter4/module3-overview',
          },
          items: [
            {
              type: 'doc',
              id: 'chapter4/module3/lesson1',
              label: 'NVIDIA Isaac SDK and Isaac Sim',
            },
            {
              type: 'doc',
              id: 'chapter4/module3/lesson2',
              label: 'AI-powered perception and manipulation',
            },
            {
              type: 'doc',
              id: 'chapter4/module3/lesson3',
              label: 'Reinforcement learning for robot control',
            },
            {
              type: 'doc',
              id: 'chapter4/module3/lesson4',
              label: 'Sim-to-real transfer techniques',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 5: Humanoid Robot Development',
      link: {
        type: 'doc',
        id: 'chapter5/chapter5-overview',
      },
      items: [
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          link: {
            type: 'doc',
            id: 'chapter5/module4-overview',
          },
          items: [
            {
              type: 'doc',
              id: 'chapter5/module4/lesson1',
              label: 'Humanoid robot kinematics and dynamics',
            },
            {
              type: 'doc',
              id: 'chapter5/module4/lesson2',
              label: 'Bipedal locomotion and balance control',
            },
            {
              type: 'doc',
              id: 'chapter5/module4/lesson3',
              label: 'Manipulation and grasping with humanoid hands',
            },
            {
              type: 'doc',
              id: 'chapter5/module4/lesson4',
              label: 'Natural human-robot interaction design',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Chapter 6 - Conversational Robotics: VLA & Multimodal AI',
      items: [
        'chapter6/lesson1',
        'chapter6/lesson2',
        'chapter6/lesson3',
      ],
    },
    {
      type: 'category',
      label: 'Chapter 7 - Capstone Project & Assessments',
      link: {
        type: 'doc',
        id: 'chapter7/index',
      },
      items: [
        {
          type: 'doc',
          id: 'chapter7/assessment1',
          label: 'ROS 2 package development project',
        },
        {
          type: 'doc',
          id: 'chapter7/assessment2',
          label: 'Gazebo simulation implementation',
        },
        {
          type: 'doc',
          id: 'chapter7/assessment3',
          label: 'Isaac-based perception pipeline',
        },
        {
          type: 'doc',
          id: 'chapter7/assessment4',
          label: 'Capstone: Simulated humanoid robot with conversational AI',
        },
      ],
    },
    'hardware',
  ],
};

export default sidebars;

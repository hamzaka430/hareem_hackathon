import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Physical AI & Humanoid Robotics Textbook Sidebar Configuration
 * Organized into comprehensive chapters covering the full curriculum
 */
const sidebars: SidebarsConfig = {
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: '1. What is Physical AI?',
      collapsed: false,
      items: [
        'chapter-01/what-is-physical-ai',
        'chapter-01/embodied-intelligence',
        'chapter-01/ai-in-physical-world',
      ],
    },
    {
      type: 'category',
      label: '2. Foundations of Robotics',
      items: [
        'chapter-02/robotics-fundamentals',
        'chapter-02/sensors-and-actuators',
        'chapter-02/robot-kinematics',
      ],
    },
    {
      type: 'category',
      label: '3. Humanoid Robotics',
      items: [
        'chapter-03/humanoid-overview',
        'chapter-03/bipedal-locomotion',
        'chapter-03/humanoid-design',
      ],
    },
    {
      type: 'category',
      label: '4. Robot Operating System (ROS2)',
      items: [
        'chapter-04/ros2-introduction',
        'chapter-04/ros2-architecture',
        'chapter-04/ros2-nodes-topics',
      ],
    },
    {
      type: 'category',
      label: '5. Computer Vision for Robots',
      items: [
        'chapter-05/vision-fundamentals',
        'chapter-05/object-detection',
        'chapter-05/depth-perception',
      ],
    },
    {
      type: 'category',
      label: '6. Motion Planning & Control',
      items: [
        'chapter-06/motion-planning',
        'chapter-06/path-planning-algorithms',
        'chapter-06/control-systems',
      ],
    },
    {
      type: 'category',
      label: '7. Reinforcement Learning for Robotics',
      items: [
        'chapter-07/rl-fundamentals',
        'chapter-07/robot-learning',
        'chapter-07/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: '8. Simulation Environments',
      items: [
        'chapter-08/simulation-overview',
        'chapter-08/gazebo-simulation',
        'chapter-08/isaac-sim',
      ],
    },
    {
      type: 'category',
      label: '9. Natural Language & Robots',
      items: [
        'chapter-09/nlp-for-robots',
        'chapter-09/voice-commands',
        'chapter-09/llm-integration',
      ],
    },
    {
      type: 'category',
      label: '10. Real-World Applications',
      items: [
        'chapter-10/industrial-robotics',
        'chapter-10/service-robots',
        'chapter-10/future-trends',
      ],
    },
  ],
};

export default sidebars;

import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link'
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

type LessonOutcome = {
  label: string;
  link: string;
  definition: string;
}

type FeatureItem = {
  title: string;
  imageSrc: string;
  description: ReactNode;
  mainLink: string; // The primary link for the module
  subsections?: { subheading: string; content: ReactNode; }[];
  lessonOutcomes?: LessonOutcome[];
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS2 & Robotics Middleware',
    imageSrc: '/img/ros2.png',
    description: (
      <>
        Master the neural nervous system of robots. Dive deep into nodes, topics, and services 
        to orchestrate complex humanoid behaviors.
      </>
    ),
    mainLink: '/docs/chapter2/module1-detailing',
    subsections: [
      {
        subheading: 'ROS2 Core Concepts',
        content: 'Understand the foundational elements of ROS2, including its distributed architecture, nodes, topics, and services for inter-process communication.',
      },
      {
        subheading: 'Robotics Middleware',
        content: 'Explore the role of middleware in robotics, enabling seamless integration of hardware and software components for complex robotic systems.',
      },
      {
        subheading: 'Behavior Orchestration',
        content: 'Learn how to orchestrate complex humanoid behaviors using ROS2, from basic movements to advanced task execution.',
      },
    ],
    lessonOutcomes: [
      { label: 'ROS2 Basics', link: '/docs/chapter2/module1/lesson1', definition: 'Fundamentals of ROS2 framework.' },
      { label: 'Nodes & Topics', link: '/docs/chapter2/module1/lesson2', definition: 'Inter-process communication mechanisms.' },
      { label: 'Services & Actions', link: '/docs/chapter2/module1/lesson3', definition: 'Request-response and long-running task patterns.' },
      { label: 'Advanced ROS2', link: '/docs/chapter2/module1/lesson4', definition: 'Dive deeper into custom interfaces and advanced tools.' },
    ],
  },
  {
    title: 'Digital Twin & Physics Sim',
    imageSrc: '/img/digital.png',
    description: (
      <>
        Bridge the sim-to-real gap. Leverage Gazebo and Unity to create high-fidelity 
        virtual labs for safe and scalable AI training.
      </>
    ),
    mainLink: '/docs/chapter3/module2-detailing',
    subsections: [
      {
        subheading: 'Sim-to-Real Gap',
        content: 'Bridge the gap between virtual simulations and real-world robot performance, ensuring robust and reliable operation.',
      },
      {
        subheading: 'Virtual Labs (Gazebo/Unity)',
        content: 'Leverage high-fidelity simulation environments like Gazebo and Unity to create safe and scalable virtual labs for AI training.',
      },
      {
        subheading: 'Physics-Based Simulation',
        content: 'Understand the principles of physics-based simulation for accurate modeling of robotic dynamics and interactions.',
      },
    ],
    lessonOutcomes: [
      { label: 'Simulation Setup', link: '/docs/chapter3/module2/lesson1', definition: 'Setting up your first simulation environment.' },
      { label: 'Gazebo & Unity', link: '/docs/chapter3/module2/lesson2', definition: 'Comparing and utilizing different simulation platforms.' },
      { label: 'Sim-to-Real Transfer', link: '/docs/chapter3/module2/lesson3', definition: 'Techniques for transferring learned policies to real robots.' },
      { label: 'Advanced Simulation', link: '/docs/chapter3/module2/lesson4', definition: 'Complex environments and multi-robot simulations.' },
    ],
  },
  {
    title: 'AI Brain (Isaac)',
    imageSrc: '/img/edge.png',
    description: (
      <>
        Deploy intelligence to the edge. Explore hardware requirements, quantum-crypto trends, 
        and real-time inference on NVIDIA Jetson platforms.
      </>
    ),
    mainLink: '/docs/chapter4/module3-detailing',
    subsections: [
      {
        subheading: 'Edge AI Deployment',
        content: 'Deploy intelligence directly to the edge, optimizing AI models for real-time inference on robotic hardware.',
      },
      {
        subheading: 'Hardware Requirements',
        content: 'Explore the hardware requirements for AI-powered robots, including embedded systems and specialized processing units.',
      },
      {
        subheading: 'NVIDIA Isaac Platform',
        content: 'Utilize the NVIDIA Isaac platform for accelerated AI development, simulation, and deployment in robotics.',
      },
    ],
    lessonOutcomes: [
      { label: 'Isaac Platform Overview', link: '/docs/chapter4/module3/lesson1', definition: 'Introduction to NVIDIA Isaac SDK and Sim.' },
      { label: 'Reinforcement Learning', link: '/docs/chapter4/module3/lesson2', definition: 'Applying RL techniques in robotic control.' },
      { label: 'Model Deployment', link: '/docs/chapter4/module3/lesson3', definition: 'Deploying trained models to edge devices.' },
      { label: 'Isaac ROS Integration', link: '/docs/chapter4/module3/lesson4', definition: 'Combining Isaac with ROS for advanced robotics.' },
    ],
  },
  {
    title: 'VLA & Cognition',
    imageSrc: '/img/humanoid.png', // Reusing humanoid image for now
    description: (
      <>
        Explore the frontier of embodied AI. Understand Visual-Language-Action (VLA) models and 
        the cognitive architectures driving next-generation humanoid robots.
      </>
    ),
    mainLink: '/docs/chapter5/module4-detailing',
    subsections: [
      {
        subheading: 'Visual-Language-Action (VLA)',
        content: 'Understand how VLA models integrate vision, language, and action to enable more intelligent and versatile robotic behaviors.',
      },
      {
        subheading: 'Cognitive Architectures',
        content: 'Explore advanced cognitive architectures that drive next-generation humanoid robots, enabling reasoning and decision-making.',
      },
      {
        subheading: 'Embodied AI',
        content: 'Delve into the frontier of embodied AI, where AI systems learn and interact with the physical world through a robotic body.',
      },
    ],
    lessonOutcomes: [
      { label: 'VLA Fundamentals', link: '/docs/chapter5/module4/lesson1', definition: 'Core concepts of Visual-Language-Action models.' },
      { label: 'Cognitive Architectures', link: '/docs/chapter5/module4/lesson2', definition: 'Building intelligent decision-making systems.' },
      { label: 'Ethical AI in Robotics', link: '/docs/chapter5/module4/lesson3', definition: 'Considering the ethical implications of advanced robotics.' },
      { label: 'Human-Robot Interaction', link: '/docs/chapter5/module4/lesson4', definition: 'Designing intuitive and safe interactions with humanoids.' },
    ],
  },
];

function Feature({title, imageSrc, description, mainLink, subsections, lessonOutcomes}: FeatureItem) {
  const {siteConfig} = useDocusaurusContext(); // Needed for useBaseUrl
  return (
    <div className={clsx('col col--6')}>
      <div className={clsx("card-demo", styles.featureCard)}>
        <div className="card">
          <div className="card__header">
            <div className="text--center">
              <Link to={mainLink}>
                <img src={useBaseUrl(imageSrc)} className={styles.featureSvg} alt={title} />
              </Link>
            </div>
            <Heading as="h3" className="text--center">
              <Link to={mainLink} style={{textDecoration: 'none', color: 'inherit'}}>{title}</Link>
            </Heading>
          </div>
          <div className="card__body">
            <p className={styles.mainDescription}>{description}</p>
            {subsections && subsections.length > 0 && (
              <div className={styles.subsections}>
                {subsections.map((section, idx) => (
                  <div key={idx} className={styles.subsectionItem}>
                    <Heading as="h4">{section.subheading}</Heading>
                    <p>{section.content}</p>
                  </div>
                ))}
              </div>
            )}
            {lessonOutcomes && lessonOutcomes.length > 0 && (
              <div className={styles.lessonOutcomes}>
                <Heading as="h4">Learning Outcomes:</Heading>
                <ul>
                  {lessonOutcomes.map((outcome, idx) => (
                    <li key={idx}>
                      <Heading as="h4"><Link to={outcome.link}>{outcome.label}</Link></Heading>
                      <p className={styles.lessonDefinition}>{outcome.definition}</p>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>
          <div className="card__footer">
            <Link className="button button--secondary button--sm" to={mainLink}>
              Learn More
            </Link>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function ModuleCards(): ReactNode {
  return (
    <section className={styles.moduleCards}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
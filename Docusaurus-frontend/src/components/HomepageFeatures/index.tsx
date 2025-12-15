import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import Link from '@docusaurus/Link'
import useBaseUrl from '@docusaurus/useBaseUrl';

type FeatureItem = {
  title: string;
  imageSrc: string;
  description: ReactNode;
  link: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS2 & Robotics Middleware',
    imageSrc: '/img/ros2-category.png',
    description: (
      <>
        Master the neural nervous system of robots. Dive deep into nodes, topics, and services 
        to orchestrate complex humanoid behaviors.
      </>
    ),
    link: '/docs/chapter2',
  },
  {
    title: 'Digital Twin & Physics Sim',
    imageSrc: '/img/digital-twin-category.png',
    description: (
      <>
        Bridge the sim-to-real gap. Leverage Gazebo and Unity to create high-fidelity 
        virtual labs for safe and scalable AI training.
      </>
    ),
    link: '/docs/chapter3',
  },
  {
    title: 'Hardware & Edge AI',
    imageSrc: '/img/edge-ai-category.png',
    description: (
      <>
        Deploy intelligence to the edge. Explore hardware requirements, quantum-crypto trends, 
        and real-time inference on NVIDIA Jetson platforms.
      </>
    ),
    link: '/docs/hardware-requirements',
  },
];

function Feature({title, imageSrc, description, link}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Link to={link}>
          <img src={useBaseUrl(imageSrc)} className={styles.featureSvg} alt={title} style={{cursor: 'pointer'}} />
        </Link>
      </div>
      <div className="text--center padding-horiz--md">
        <Link to={link} style={{textDecoration: 'none', color: 'inherit'}}>
          <Heading as="h3">{title}</Heading>
        </Link>
        <p>{description}</p>
        <Link className="button button--secondary button--sm" to={link}>
          Learn More
        </Link>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
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

import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import ModuleCards from '@site/src/components/ModuleCards';
import Heading from '@theme/Heading';
import useBaseUrl from '@docusaurus/useBaseUrl';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={clsx("container", styles.heroContainer)}>
                            <div className={styles.heroMainContent}>
                              {/* <div style={{marginBottom: '1rem'}}>
                                <span className="badge badge--secondary margin-bottom--md">✨ 2025 Edition Updated</span>
                              </div> */}
                              <img src={useBaseUrl("/img/humanoid.png")} alt="Humanoid Book Icon" className={styles.heroBookIcon} style={{maxWidth: '150px', marginBottom: '1rem'}} />
                              <Heading as="h1" className="hero__title">
                                {siteConfig.title}
                              </Heading>
                              <p className="hero__subtitle">{siteConfig.tagline}</p>
                              <div className={styles.buttons}>
                                <Link
                                  className="button button--secondary button--lg"
                                  to="/docs/">
                                  Start Your Journey - 60min ⏱️
                                </Link>
                              </div>
                            </div>
                            <div className={clsx(styles.quickLinksContainer)} style={{marginTop: '3rem'}}>
                              <Heading as="h2" className={styles.quickLinksTitle} style={{color: 'white', marginBottom: '1rem'}}>Jump to Core Modules</Heading>
                              <div className={clsx(styles.quickLinksList)} style={{display: 'flex', gap: '10px', flexWrap: 'wrap', justifyContent: 'center'}}>
                                <Link className="button button--outline button--secondary" to="/docs/chapter2/module1-detailing">Module 1: ROS 2 Nervous System</Link>
                                <Link className="button button--outline button--secondary" to="/docs/chapter3/module2-detailing">Module 2: Digital Twin (Sim)</Link>
                                <Link className="button button--outline button--secondary" to="/docs/chapter4/module3-detailing">Module 3: AI Brain (Isaac)</Link>
                                <Link className="button button--outline button--secondary" to="/docs/chapter5/module4-detailing">Module 4: VLA & Cognition</Link>
                              </div>
                            </div>      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI Textbook | ${siteConfig.title}`}
      description="A comprehensive guide to Physical AI and Humanoid Robotics, covering ROS 2, Simulation, NVIDIA Isaac, and VLA systems.">
      <HomepageHeader />
      <main>
        <ModuleCards />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}

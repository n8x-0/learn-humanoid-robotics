import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/preface/intro">
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className={styles.feature}>
              <h3>📚 Comprehensive Content</h3>
              <p>
                Complete 13-week course covering Physical AI, ROS 2, Digital Twins, 
                NVIDIA Isaac, Humanoid Development, and Conversational Robotics.
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.feature}>
              <h3>🤖 Interactive Chatbot</h3>
              <p>
                Ask questions about the textbook content with our RAG-powered chatbot. 
                Get answers with citations, or ask about specific selected text.
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className={styles.feature}>
              <h3>🧪 Hands-On Labs</h3>
              <p>
                Each chapter includes practical labs, exercises, and checkpoint quizzes 
                to reinforce your learning.
              </p>
            </div>
          </div>
        </div>
        <div className="row" style={{marginTop: '2rem'}}>
          <div className="col col--6">
            <div className={styles.feature}>
              <h3>🎯 Learning Path</h3>
              <ul>
                <li>Foundations (Weeks 1-2)</li>
                <li>ROS 2 Nervous System (Weeks 3-5)</li>
                <li>Digital Twin (Weeks 6-7)</li>
                <li>NVIDIA Isaac Platform (Weeks 8-10)</li>
                <li>Humanoid Development (Weeks 11-12)</li>
                <li>Conversational Robotics (Week 13)</li>
              </ul>
            </div>
          </div>
          <div className="col col--6">
            <div className={styles.feature}>
              <h3>🚀 Quick Start</h3>
              <ol>
                <li>Read the <Link to="/docs/preface/intro">Preface</Link> to understand how to use this book</li>
                <li>Check the <Link to="/docs/appendix/hardware">Hardware Tracks</Link> for setup requirements</li>
                <li>Start with <Link to="/docs/foundations/intro">Foundations</Link></li>
                <li>Use the chatbot (💬 button) to ask questions as you read</li>
              </ol>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive textbook on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}


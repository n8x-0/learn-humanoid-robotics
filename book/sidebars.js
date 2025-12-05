/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'preface/intro',
      label: 'Preface & How to Use This Book',
    },
    {
      type: 'category',
      label: 'Foundations (Weeks 1–2)',
      items: [
        'foundations/intro',
        'foundations/week1',
        'foundations/week2',
      ],
    },
    {
      type: 'category',
      label: 'ROS 2 Nervous System (Weeks 3–5)',
      items: [
        'ros2/intro',
        'ros2/week3',
        'ros2/week4',
        'ros2/week5',
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin (Weeks 6–7)',
      items: [
        'digital-twin/intro',
        'digital-twin/week6',
        'digital-twin/week7',
      ],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac Platform (Weeks 8–10)',
      items: [
        'isaac/intro',
        'isaac/week8',
        'isaac/week9',
        'isaac/week10',
      ],
    },
    {
      type: 'category',
      label: 'Humanoid Development (Weeks 11–12)',
      items: [
        'humanoid-dev/intro',
        'humanoid-dev/week11',
        'humanoid-dev/week12',
      ],
    },
    {
      type: 'category',
      label: 'Conversational Robotics (Week 13)',
      items: [
        'conversational-robotics/intro',
        'conversational-robotics/week13',
      ],
    },
    {
      type: 'doc',
      id: 'capstone/intro',
      label: 'Capstone Guide + Rubrics',
    },
    {
      type: 'category',
      label: 'Appendix',
      items: [
        'appendix/hardware',
        'appendix/glossary',
      ],
    },
  ],
};

module.exports = sidebars;


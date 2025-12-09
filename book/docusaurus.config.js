// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook on Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://n8x-0.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages. If your repo is https://github.com/username/repo-name, then baseUrl should be '/repo-name/'
  // For user/organization sites, baseUrl should be '/'
  baseUrl: '/learn-humanoid-robotics/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'n8x-0', // Usually your GitHub org/user name.
  projectName: 'learn-humanoid-robotics', // Usually your repo name.

  onBrokenLinks: 'throw',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to set "zh-Hans" as value.
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: 'https://github.com/n8x-0/learn-humanoid-robotics/tree/main/book/',
        },
        blog: false, // Disable blog
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

        themeConfig:
      /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
      ({
        apiBaseUrl: process.env.RAG_API_BASE_URL || 'http://localhost:8000',
        // Replace with your project's social card
        image: 'img/docusaurus-social-card.jpg',      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            to: '/',
            label: 'Home',
            position: 'right',
          },
          {
            href: 'https://github.com/n8x-0/learn-humanoid-robotics',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Chapters',
            items: [
              {
                label: 'Foundations',
                to: '/docs/foundations/intro',
              },
              {
                label: 'ROS 2',
                to: '/docs/ros2/intro',
              },
              {
                label: 'Digital Twin',
                to: '/docs/digital-twin/intro',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Glossary',
                to: '/docs/appendix/glossary',
              },
              {
                label: 'Hardware Tracks',
                to: '/docs/appendix/hardware',
              },
            ],
          },
          {
            title: 'Author',
            items: [
              {
                label: 'Syed Shayan Ali',
                href: 'https://github.com/n8x-0',
              },
              {
                label: 'Repository',
                href: 'https://github.com/n8x-0/learn-humanoid-robotics',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Syed Shayan Ali. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;


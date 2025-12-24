// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const { themes: prismThemes } = require('prism-react-renderer');

// const lightCodeTheme = require('@docusaurus/theme-prismjs');
// const darkCodeTheme = require('@docusaurus/theme-prismjs');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI Robotics Book',
  tagline: 'A comprehensive guide to Physical AI and embodied intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://speckitplus-book-rag-chatbot.vercel.app/',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<username>.github.io/<repo-name>'
  baseUrl: '/',

  // GitHub pages deployment config.
  organizationName: 'your-organization', // Usually your GitHub org/user name.
  projectName: 'giaic-hackathon-speckit-plus', // Usually your repo name.

  onBrokenLinks: 'ignore',
  // onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
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
          editUrl:
            'https://github.com/Aqsa-owais/speckitplus-book-rag-chatbot.git',
        },
        blog: false, // Disable blog functionality
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
        pages: {
          // Enable the pages plugin to process root index.md as homepage
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        logo: {
          src: 'https://cdn3d.iconscout.com/3d/premium/thumb/ai-book-3d-icon-png-download-7900686.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Physical AI Robotics Book',
          },
          {
            to: '/docs/intro',
            label: 'Home',
            position: 'left',
          },
          {
            to: '/docs/module-1-ros/',
            label: 'Module 1',
            position: 'left',
          },
          {
            to: '/docs/module-2-digital-twin/',
            label: 'Module 2',
            position: 'left',
          },
          {
            to: '/docs/module-3-ai-brain/',
            label: 'Module 3',
            position: 'left',
          },
          {
            to: '/docs/module-4-vla/',
            label: 'Module 4',
            position: 'left',
          },
          {
            href: 'https://github.com/Aqsa-owais/speckitplus-book-rag-chatbot.git',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Aqsa-owais/speckitplus-book-rag-chatbot.git',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

module.exports = config;
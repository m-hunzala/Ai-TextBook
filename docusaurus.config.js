// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Teaching Embodied Intelligence with AI Agents',
  favicon: 'img/favicon.svg',

  customFields: {
    ragBackendUrl: process.env.RAG_BACKEND_URL || 'http://localhost:8000',
    // Dev server configuration for proxying auth requests
    devServer: {
      proxy: {
        '/api/auth': {
          target: process.env.AUTH_API_URL || 'http://localhost:4001',
          changeOrigin: true,
          secure: false,
        },
      },
    },
  },

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  markdown: {
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
    },
  },

  // Set the production url of your site here
  url: 'http://localhost:3000',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Abc', // Usually your GitHub org/user name.
  projectName: 'physical-ai-book-main', // Usually your repo name.

  onBrokenLinks: 'warn',

  // Configure client modules to inject runtime configuration
  clientModules: [
    require.resolve('./src/client-modules/runtime-config-injector.js'),
    require.resolve('./src/client-modules/auth-manager.js'),
  ],

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: 'docs',
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      colorMode: {
        defaultMode: 'dark', // Set dark mode as default
        disableSwitch: false, // Allow switching between modes
        respectPrefersColorScheme: true, // Still respect system preference if available
      },
      // Replace with your project's social card
      image: 'img/logo.svg',
      navbar: {
        logo: {
          alt: 'Physical AI Humanoid Textbook Logo',
          src: 'img/logo.svg',
          srcDark: 'img/logo-dark.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Tutorial',
          },
          {to: '/blog', label: 'Blog', position: 'left'},
          {
            href: 'https://github.com/m-hunzala/physical-ai-book',
            label: 'GitHub',
            position: 'right',
          },
          // Auth buttons will be handled by the AuthManager component in the Navbar theme
        ],
      },
      footer: {
        style: 'dark',
        logo: {
          alt: 'Physical AI Humanoid Textbook Logo',
          src: 'img/logo.svg',
          srcDark: 'img/logo-dark.svg',
          href: '/',
          width: 160,
        },
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Tutorial',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'X',
                href: 'https://x.com/MrHunzala5787',
              },
               {
                label: 'GitHub',
                href: 'https://github.com/m-hunzala',
              },
              {
                label: 'Linkedin',
                href: 'https://www.linkedin.com/in/m-hunzala/',
              },
             
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Blog',
                to: '/blog',
              },
              {
                label: 'Login',
                to: '/login',
              },
              {
                label: 'Instagram',
                href: 'https://www.instagram.com/___hunzala/',
              },
              {
                label: 'Portfolio',
                href: 'https://muhammad-hunzala.vercel.app/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;

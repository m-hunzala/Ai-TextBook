# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Common Development Tasks

-   **Start Development Server**: `npm start`
-   **Build Static Site**: `npm run build`
-   **Deploy Site**: `npm run deploy`
-   **Clear Build Output and Cache**: `npm run clear`
-   **Serve Built Site Locally**: `npm run serve`

## High-Level Code Architecture

This project is a Docusaurus site, a React-based static site generator for documentation.

-   **`docusaurus.config.js`**: Main configuration for the Docusaurus site, defining metadata, plugins, themes, and content sources.
-   **`sidebars.js`**: Configures the navigation sidebars for the documentation.
-   **`blog/`**: Stores markdown/MDX files for blog posts.
-   **`docs/`**: Stores markdown/MDX files for documentation pages, organized into categories (e.g., `tutorial-basics/`, `tutorial-extras/`).
-   **`src/`**: Contains custom React components, CSS, or other client-side assets that extend the Docusaurus theme.
-   **`static/`**: Holds static assets like images.

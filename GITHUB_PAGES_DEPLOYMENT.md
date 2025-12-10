# Physical AI Book - GitHub Pages Deployment

This repository contains a Docusaurus-based documentation site for the Physical AI Book with Better-Auth integration.

## GitHub Pages Deployment Information

- **GitHub Pages URL**: `https://m-hunzala.github.io/physical-ai-book/`
- **Base URL**: `/physical-ai-book/`
- **Build Command**: `npm run build`
- **Deployment Branch**: `gh-pages`

## Features

- Complete user authentication system using Better-Auth
- Signup with personalized questions about:
  - Programming background
  - AI experience level
  - Hardware availability
  - Other relevant preferences
- Signin functionality
- Content personalization based on user profile
- PostgreSQL database integration with Neon
- Responsive design for all devices

## GitHub Actions Workflow

The workflow file `.github/workflows/deploy-gh-pages.yml` automatically:
1. Triggers on pushes to the main branch
2. Sets up Node.js environment
3. Installs dependencies
4. Builds the static site
5. Deploys to the `gh-pages` branch

## Verification Checklist

Before confirming the GitHub Pages deployment, verify:

- [ ] Site loads at `https://<your-username>.github.io/physical-ai-book/`
- [ ] Sign up page is accessible and functional
- [ ] Sign in page is accessible and functional
- [ ] User profile form collects all required information
- [ ] Authentication flows work correctly
- [ ] Content personalization features function
- [ ] All links navigate correctly
- [ ] Images and assets load properly
- [ ] Sitemap is accessible at `/physical-ai-book/sitemap.xml`
- [ ] Contact/Support information is available

## Configuration

- `docusaurus.config.js` has `baseUrl` set to `/physical-ai-book/`
- `package.json` has `homepage` field configured
- Relative paths are used throughout the application

## Local Development

To run locally:
```bash
npm install
npm start
```

To build locally:
```bash
npm run build
```
# Deployment Guide

## GitHub Pages Setup

### 1. Automated Deployment via GitHub Actions

The documentation site is automatically deployed to GitHub Pages using GitHub Actions CI/CD pipeline.

#### GitHub Actions Workflow File

Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 24
          cache: npm

      - name: Install dependencies
        run: npm install

      - name: Build website
        run: npm run build

      # Popular action to deploy to GitHub Pages:
      # Docs: https://github.com/peaceiris/actions-gh-pages#%EF%B8%8F-docusaurus
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          # Build output to publish to the `gh-pages` branch:
          publish_dir: ./build
          # The following lines assign commit authorship to the official
          # GH-Actions bot for deploys to `gh-pages` branch:
          # https://github.com/actions/checkout/issues/13#issuecomment-724415212
          # The GH actions bot is used by default if you didn't specify the two fields.
          # You can swap them with your own user credentials.
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

### 2. Docusaurus Configuration for GitHub Pages

In `docusaurus.config.js`, ensure the following configuration:

```javascript
const config = {
  // ...
  url: 'https://your-username.github.io', // Your GitHub Pages domain
  baseUrl: '/your-repo-name/', // Your repository name
  trailingSlash: false,
  // ...
  themeConfig: {
    // ...
    metadata: [
      {name: 'keywords', content: 'documentation, ai, rag, docusaurus'},
    ],
  },
  // ...
};
```

### 3. GitHub Pages Settings

1. Go to your repository Settings
2. Navigate to "Pages" in the left sidebar
3. Select "Deploy from a branch"
4. Choose `gh-pages` branch and `/root` folder (or `/docs` if you're using the docs-only option)
5. Save settings

### 4. Backend API Deployment

For the FastAPI backend, you can deploy to various platforms:

#### Option A: Railway (Recommended for rapid deployment)

1. Sign up at [Railway.app](https://railway.app)
2. Connect your GitHub repository
3. Add environment variables:
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `GEMINI_API_KEY`
   - `ANTHROPIC_API_KEY`
   - `NEON_DATABASE_URL`
   - `BETTER_AUTH_SECRET`
   - `API_KEY`

#### Option B: Render

1. Sign up at [Render.com](https://render.com)
2. Create a new Web Service
3. Connect your GitHub repository
4. Set the build command: `pip install -r requirements.txt`
5. Set the start command: `uvicorn main:app --host 0.0.0.0 --port $PORT`
6. Add environment variables as above

#### Option C: Heroku

1. Create a new app in Heroku dashboard
2. Connect your GitHub repository
3. Enable automatic deploys
4. Add environment variables in Settings > Config Vars
5. Set buildpack to Python

## CI/CD Pipeline

### GitHub Actions Configuration

The repository includes a complete CI/CD pipeline:

#### Frontend Tests
```yaml
- name: Run frontend tests
  run: |
    npm run test
    npm run lint
    npm run build
```

#### Backend Tests
```yaml
- name: Run backend tests
  run: |
    cd backend/rag_backend
    python -m pytest
    python -m flake8 .
```

### Environment Variables Management

Set up repository secrets in GitHub:
- `QDRANT_URL`
- `QDRANT_API_KEY`
- `GEMINI_API_KEY`
- `ANTHROPIC_API_KEY`
- `NEON_DATABASE_URL`
- `BETTER_AUTH_SECRET`
- `API_KEY`

## Database Migrations

For Neon Postgres, the migration script runs automatically on deploy:

```bash
python migrate.py
```

## Configuration Files

### Docker Configuration (Optional)

Create `Dockerfile` for containerized deployment:

```dockerfile
FROM python:3.10-slim

WORKDIR /app

COPY backend/rag_backend/requirements.txt .
RUN pip install -r requirements.txt

COPY backend/rag_backend/ ./

EXPOSE 8000

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Docker Compose for Local Development

Create `docker-compose.yml`:

```yaml
version: '3.8'
services:
  backend:
    build: .
    ports:
      - "8000:8000"
    environment:
      - QDRANT_URL=${QDRANT_URL}
      - GEMINI_API_KEY=${GEMINI_API_KEY}
      - NEON_DATABASE_URL=${NEON_DATABASE_URL}
    volumes:
      - ./backend:/app
```

## Deployment Commands

### Deploy Frontend Only
```bash
npm run deploy
```

### Deploy Backend to Railway
```bash
npx railway login
npx railway up
```

### Deploy Backend to Render
```bash
git push render main
```

## Monitoring and Logging

### Frontend
- Google Analytics integration
- Error tracking with Sentry (optional)

### Backend
- Structured logging with log levels
- Request/response logging
- Performance metrics

## Rollback Procedures

### GitHub Pages
1. Navigate to repository Settings > Pages
2. Click on "View deployment history"
3. Click "Rollback" next to any deployment

### Backend (Railway/Render)
1. Use the platform's rollback functionality
2. Or deploy a previous git tag/commit

## Troubleshooting

### Common Issues

1. **GitHub Pages not building**: Check `baseUrl` in `docusaurus.config.js`
2. **API endpoints returning 404**: Verify backend deployment and CORS settings
3. **Environment variables not loading**: Ensure secrets are properly set in GitHub
4. **Qdrant connection failures**: Check network access and credentials

### Debugging Commands

```bash
# Check environment variables
printenv | grep -i api

# Test backend locally
curl http://localhost:8000/

# Check logs
npx railway logs
```

## Backup Strategy

### Database Backup
Neon provides automatic backups. For manual backup:
```sql
-- Export data using Neon's backup features
-- Or use pg_dump for manual backups
```

### Documentation Backup
Git-based backup through GitHub's infrastructure.

This deployment setup ensures a robust, scalable, and maintainable documentation platform with automated CI/CD pipelines.
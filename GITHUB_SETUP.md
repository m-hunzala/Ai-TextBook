# GitHub Repository Setup Instructions

## Steps to Push Your Code to GitHub

### 1. Create Repository on GitHub
- Go to https://github.com/new (you may need to sign in first)
- Repository name: `MY_NEW_PROJECT` (or choose your preferred name)
- Choose Public or Private
- **DO NOT** initialize with README, .gitignore, or license (we already have these)
- Click "Create repository"

### 2. After Creating the Repository
Run these commands in your terminal (replace `YOUR_REPO_NAME` with the actual repository name):

```bash
# Add the remote (replace YOUR_REPO_NAME with your actual repo name)
git remote add origin https://github.com/m-hunzala/YOUR_REPO_NAME.git

# Push to GitHub
git push -u origin master
```

Or if your default branch is `main` instead of `master`:
```bash
git branch -M main
git remote add origin https://github.com/m-hunzala/YOUR_REPO_NAME.git
git push -u origin main
```

### Alternative: Using SSH (if you have SSH keys set up)
```bash
git remote add origin git@github.com:m-hunzala/YOUR_REPO_NAME.git
git push -u origin master
```


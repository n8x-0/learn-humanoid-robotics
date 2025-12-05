# GitHub Pages Deployment Guide

## Issue: 404 Error on GitHub Pages

If you're seeing a 404 error, it's likely because GitHub Pages is configured to deploy from `/docs` folder, but we're using GitHub Actions to deploy.

## Solution: Change GitHub Pages Source

1. Go to your repository settings on GitHub
2. Navigate to **Pages** in the left sidebar
3. Under **Build and deployment**:
   - Change **Source** from "Deploy from a branch" to **"GitHub Actions"**
   - Save the changes

## Why This Works

The GitHub Actions workflow (`.github/workflows/deploy-book.yml`) uses the new GitHub Pages deployment method:
- Builds Docusaurus in the `book/` directory
- Uploads the built site as an artifact
- Deploys using `actions/deploy-pages@v4`

This is the recommended method for Docusaurus sites and doesn't require the `/docs` folder.

## Alternative: Deploy from Branch (Not Recommended)

If you prefer to deploy from a branch, you would need to:
1. Build the site locally: `cd book && npm run build`
2. Copy `book/build/*` to `docs/`
3. Commit and push
4. Configure GitHub Pages to deploy from `/docs` folder

But the GitHub Actions method is better because it's automated.

## Configuration

Make sure your `book/docusaurus.config.js` has the correct:
- `url`: Your GitHub Pages URL (e.g., `https://n8x-0.github.io`)
- `baseUrl`: Your repository name with slashes (e.g., `/learn-humanoid-robotics/`)
- `organizationName`: Your GitHub username
- `projectName`: Your repository name

## After Changing to GitHub Actions

1. Push a commit to trigger the workflow
2. Check the **Actions** tab to see the deployment progress
3. Once complete, your site should be live at: `https://n8x-0.github.io/learn-humanoid-robotics/`


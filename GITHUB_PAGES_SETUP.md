# Fix GitHub Pages 404 Error

## The Problem

You're seeing a Jekyll build error because GitHub Pages is still configured to **"Deploy from a branch"** and is trying to build from the `/docs` folder using Jekyll. But we're using **Docusaurus** which is built by GitHub Actions.

## The Solution

### Step 1: Change GitHub Pages Source to GitHub Actions

1. Go to your repository: `https://github.com/n8x-0/learn-humanoid-robotics`
2. Click **Settings** → **Pages** (in the left sidebar)
3. Under **"Build and deployment"**:
   - Find the **"Source"** dropdown
   - Change it from **"Deploy from a branch"** to **"GitHub Actions"**
   - Click **Save**

### Step 2: Verify the Workflow

The GitHub Actions workflow (`.github/workflows/deploy-book.yml`) will:
- Build your Docusaurus site
- Upload it as an artifact
- Deploy it to GitHub Pages

### Step 3: Trigger the Deployment

After changing the source:
1. The workflow will automatically run on the next push
2. Or you can manually trigger it: Go to **Actions** tab → **Deploy Docusaurus to GitHub Pages** → **Run workflow**

### Step 4: Check Deployment Status

1. Go to the **Actions** tab
2. You should see the workflow running
3. Once complete, your site will be live at: `https://n8x-0.github.io/learn-humanoid-robotics/`

## Why This Happens

- **Old method (Deploy from branch)**: GitHub Pages automatically builds Jekyll sites from `/docs` or root
- **New method (GitHub Actions)**: You control the build process - perfect for Docusaurus, React, etc.

## Troubleshooting

If you still see errors:
1. Make sure the workflow file is in `.github/workflows/deploy-book.yml`
2. Check that `book/package.json` has the build script
3. Verify Node.js version (we're using 18, which is stable)
4. Check the Actions tab for detailed error messages

## Current Configuration

- **Repository**: `n8x-0/learn-humanoid-robotics`
- **Site URL**: `https://n8x-0.github.io/learn-humanoid-robotics/`
- **Build Tool**: Docusaurus (via GitHub Actions)
- **Build Output**: `book/build/`


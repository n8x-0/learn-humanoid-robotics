# Image Assets

Place your image files here. They will be accessible at `/img/filename` in your Docusaurus site.

## Required Images

### Logo
- **File**: `logo.svg` (or `logo.png`)
- **Location**: `book/static/img/logo.svg`
- **Usage**: Navbar logo (configured in `docusaurus.config.js`)
- **Recommended size**: 32x32px or 64x64px
- **Format**: SVG (preferred) or PNG

### Favicon
- **File**: `favicon.ico`
- **Location**: `book/static/img/favicon.ico`
- **Usage**: Browser tab icon
- **Format**: ICO or PNG

### Social Card (Optional)
- **File**: `docusaurus-social-card.jpg`
- **Location**: `book/static/img/docusaurus-social-card.jpg`
- **Usage**: Social media preview image
- **Recommended size**: 1200x630px

## How Docusaurus Serves Static Files

Files in `book/static/` are served from the root of your site:
- `book/static/img/logo.svg` → accessible at `/img/logo.svg`
- `book/static/img/favicon.ico` → accessible at `/img/favicon.ico`

## Current Configuration

The logo is configured in `docusaurus.config.js`:
```javascript
logo: {
  alt: 'Physical AI Logo',
  src: 'img/logo.svg',  // This points to /img/logo.svg
},
```

If you use a different filename or format, update the `src` value in the config.


# Quickstart: Docusaurus UI Update for Physical AI Robotics Book

**Feature**: Docusaurus UI Update for Physical AI Robotics Book
**Date**: 2025-12-25
**Branch**: 1-docusaurus-ui-update

## Prerequisites

- Node.js (v16 or higher)
- npm or yarn package manager
- Git
- Basic knowledge of React and Docusaurus

## Setup Instructions

### 1. Clone and Install Dependencies

```bash
# Clone the repository (if not already done)
git clone <repository-url>
cd rag-chatbot

# Install dependencies
npm install
# or
yarn install
```

### 2. Start Development Server

```bash
npm run start
# or
yarn start
```

This will start the Docusaurus development server at http://localhost:3000

### 3. Verify Current Structure

Before implementing changes, verify the current structure:

```bash
# Check docs directory structure
ls -la docs/

# Check sidebar configuration
cat sidebars.js

# Check Docusaurus config
cat docusaurus.config.js
```

## Implementation Steps

### Step 1: Create Custom Components Directory

```bash
mkdir -p src/components
mkdir -p src/css
mkdir -p src/theme
```

### Step 2: Create Tab Navigation Component

Create `src/components/ModuleTabs.js`:

```jsx
import React, { useState } from 'react';
import './ModuleTabs.css';

const ModuleTabs = ({ children }) => {
  const [activeTab, setActiveTab] = useState('home');

  const tabs = [
    { id: 'home', label: 'Home' },
    { id: 'module1', label: 'Module 1' },
    { id: 'module2', label: 'Module 2' },
    { id: 'module3', label: 'Module 3' },
    { id: 'module4', label: 'Module 4' },
  ];

  return (
    <div className="module-tabs-container">
      <div className="tabs-header">
        {tabs.map(tab => (
          <button
            key={tab.id}
            className={`tab-button ${activeTab === tab.id ? 'active' : ''}`}
            onClick={() => setActiveTab(tab.id)}
          >
            {tab.label}
          </button>
        ))}
      </div>
      <div className="tabs-content">
        {React.Children.map(children, child => {
          if (child.props.tabId === activeTab) {
            return child;
          }
          return null;
        })}
      </div>
    </div>
  );
};

export default ModuleTabs;
```

### Step 3: Add Custom CSS

Create `src/css/custom.css` with the specified color palette:

```css
/* Color palette */
:root {
  --ifm-color-primary: #0F172A; /* Dark Navy / Slate */
  --ifm-color-primary-dark: #0F172A;
  --ifm-color-primary-darker: #0F172A;
  --ifm-color-primary-darkest: #0F172A;
  --ifm-color-primary-light: #2563EB; /* Muted Blue */
  --ifm-color-primary-lighter: #38BDF8; /* Soft Sky Blue */
  --ifm-color-primary-lightest: #38BDF8;

  /* Background colors */
  --ifm-background-color: #F8FAFC; /* Light mode */
  --ifm-background-surface-color: #ffffff;

  /* Text colors */
  --ifm-heading-color: #020617;
  --ifm-text-color: #334155;

  /* Dark mode */
  html[data-theme="dark"] {
    --ifm-background-color: #020617; /* Dark mode background */
    --ifm-background-surface-color: #0F172A;
  }
}

/* Typography */
h1, h2, h3, h4, h5, h6 {
  font-family: 'Inter', 'Poppins', system-ui, -apple-system, sans-serif;
}

.markdown p, .markdown li {
  font-family: 'Inter', 'Roboto', system-ui, -apple-system, sans-serif;
}

/* Code blocks */
.code-block {
  font-family: monospace;
  background-color: #f0f0f0;
  border-radius: 8px;
  padding: 12px;
}

/* Rounded corners and shadows */
.card {
  border-radius: 12px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

/* Sidebar improvements */
.menu {
  border-radius: 8px;
}

.menu__list .menu__list {
  border-radius: 8px;
  overflow: hidden;
}

/* Hover effects */
.tab-button:hover,
.menu__link:hover {
  transition: all 0.2s ease-in-out;
}

/* Responsive adjustments */
@media (max-width: 768px) {
  .module-tabs-container {
    flex-direction: column;
  }

  .tabs-header {
    overflow-x: auto;
  }
}
```

### Step 4: Create Custom Footer

Create `src/theme/Footer/index.js`:

```jsx
import React from 'react';
import clsx from 'clsx';
import { useThemeConfig } from '@docusaurus/theme-common';
import styles from './styles.module.css';

function Footer() {
  const { footer } = useThemeConfig();

  if (!footer) {
    return null;
  }

  const { copyright } = footer;

  return (
    <footer
      className={clsx('footer', {
        'footer--dark': footer.style === 'dark',
      })}>
      <div className="container container-fluid">
        {copyright ? (
          <div className="footer__copyright">
            {/* Add only the required copyright text here */}
          </div>
        ) : null}
      </div>
    </footer>
  );
}

export default Footer;
```

### Step 5: Update Docusaurus Configuration

Update `docusaurus.config.js` to include custom styling:

```javascript
// Add to existing docusaurus.config.js
module.exports = {
  // ... existing configuration
  stylesheets: [
    // Add any custom fonts if needed
    'https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600&family=Poppins:wght@400;500;600&display=swap',
  ],
  themeConfig: {
    // ... existing theme config
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
  },
  plugins: [
    // Add any custom plugins if needed
  ],
  themes: [
    // Add custom theme components
  ]
};
```

### Step 6: Run and Test

```bash
# Start the development server
npm run start

# Or build for production to test
npm run build
npm run serve
```

## Verification Steps

1. **Check tab navigation**: Verify that clicking tabs switches content properly
2. **Verify styling**: Check that colors match the specified palette
3. **Test sidebar**: Confirm sidebar updates when switching tabs
4. **Responsive testing**: Test on different screen sizes
5. **Dark mode**: Verify light/dark mode functionality
6. **Footer**: Confirm footer contains only required text

## Common Issues and Solutions

### Issue: Tabs not switching content
**Solution**: Check that the ModuleTabs component is properly wrapping content and that tabId props match the active tab state

### Issue: Styles not applying
**Solution**: Ensure custom CSS is properly imported in the Docusaurus config and that CSS variables are correctly defined

### Issue: Sidebar not updating
**Solution**: Create a custom sidebar component that accepts the active module as a prop and filters items accordingly
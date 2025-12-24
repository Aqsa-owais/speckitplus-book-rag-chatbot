# Quickstart: UI Error Diagnosis & Fix for Physical AI Humanoid Robotics Book

**Feature**: UI Error Diagnosis & Fix for Physical AI Humanoid Robotics Book
**Date**: 2025-12-25
**Branch**: 2-ui-error-fix

## Prerequisites

- Node.js (v16 or higher)
- npm or yarn package manager
- Git
- Basic knowledge of React and Docusaurus

## Setup Instructions

### 1. Clone and Install Dependencies

```bash
# Navigate to the repository (if not already done)
cd D:\rag-chatbot

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

## Debugging Steps

### Step 1: Identify JavaScript Errors

1. Open browser developer tools (F12)
2. Navigate to the Console tab
3. Load the page and look for any error messages
4. Note the specific error messages and file locations

### Step 2: Check Component Integration

1. Verify that the TabProvider is properly wrapped around the main layout in `src/theme/Layout/index.js`
2. Check that ModuleTabs component is properly integrated
3. Ensure all custom components have correct import paths

### Step 3: Verify CSS Application

1. Check that custom CSS is properly loaded
2. Look for any CSS conflicts in the browser developer tools
3. Verify color palette and typography are applied correctly

## Fix Implementation Steps

### Step 1: Fix Import Issues in DocSidebar

Update the import statements in `src/theme/DocSidebar/index.js`:

```javascript
import {
  splitOrThrow,
  useLocalPathname,
  useAnnouncementBar,
  useScrollPosition,
} from '@docusaurus/theme-common/internal';
```

### Step 2: Ensure Proper Context Provider

Verify that `src/theme/Layout/index.js` properly wraps the application:

```javascript
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { TabProvider } from '../../context/TabContext';

export default function Layout(props) {
  return (
    <TabProvider>
      <OriginalLayout {...props} />
    </TabProvider>
  );
}
```

### Step 3: Fix ModuleTabs Component

Ensure the ModuleTabs component properly handles client-side rendering:

```javascript
import React, { useState, useEffect, useContext } from 'react';
import { useTabContext } from '../context/TabContext';

const ModuleTabs = ({ children }) => {
  const { state, switchToModule, switchToHome } = useTabContext();
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  // ... rest of the component logic
};
```

### Step 4: Update Docusaurus Configuration

Update `docusaurus.config.js` to ensure proper navbar configuration:

```javascript
// Add module navigation to navbar
module.exports = {
  // ... existing config
  themeConfig: {
    // ... existing theme config
    navbar: {
      // ... existing navbar config
      items: [
        // ... existing items
        {
          type: 'doc',
          docId: 'module1/index',
          label: 'Module 1',
          position: 'left',
        },
        // Add similar entries for Module 2, Module 3, Module 4
      ],
    },
  },
};
```

### Step 5: Test the Fixes

```bash
# Start the development server
npm run start

# Check for errors in browser console
# Verify module tabs appear in navbar
# Test navigation between modules
# Confirm no "This page crashed" errors
```

## Verification Steps

1. **Check for JavaScript errors**: Open browser console and verify no errors appear
2. **Verify module tabs**: Confirm Home, Module 1-4 tabs appear in navbar
3. **Test navigation**: Click tabs and verify content switches properly
4. **Check styling**: Verify color palette and typography are applied correctly
5. **Sidebar filtering**: Confirm sidebar updates when switching modules
6. **Responsive design**: Test on different screen sizes

## Common Issues and Solutions

### Issue: "This page crashed" errors
**Solution**: Check for React hook errors, missing dependencies, or incorrect imports in custom components

### Issue: Module tabs not appearing
**Solution**: Verify TabProvider is properly wrapping the layout and ModuleTabs component is integrated correctly

### Issue: CSS conflicts
**Solution**: Check for conflicting styles in browser developer tools and adjust specificity as needed

### Issue: Sidebar not updating
**Solution**: Ensure DocSidebar component is properly using the tab context for filtering
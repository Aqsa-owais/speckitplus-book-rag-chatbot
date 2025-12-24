import React from 'react';
import { useLocation } from '@docusaurus/router';
import { useTabContext } from '../context/TabContext';

const Breadcrumb = () => {
  const location = useLocation();
  const { state } = useTabContext();

  // Determine the current module based on the active tab
  const getCurrentModuleLabel = () => {
    switch(state.activeTab) {
      case 'module1':
        return 'Module 1';
      case 'module2':
        return 'Module 2';
      case 'module3':
        return 'Module 3';
      case 'module4':
        return 'Module 4';
      default:
        return null;
    }
  };

  // Get the current page title from the URL or a mapping
  const getCurrentPageTitle = () => {
    // This is a simplified approach - in a real implementation you'd want to
    // map URLs to page titles more robustly
    const pathParts = location.pathname.split('/').filter(part => part);

    if (pathParts.length === 0) return 'Home';

    // Extract the last part of the URL as the page title
    // You might want to implement a more sophisticated mapping
    const lastPart = pathParts[pathParts.length - 1];
    return lastPart
      .replace(/-/g, ' ')
      .replace(/\b\w/g, l => l.toUpperCase()); // Capitalize first letter of each word
  };

  const moduleLabel = getCurrentModuleLabel();
  const pageTitle = getCurrentPageTitle();

  // Only show breadcrumbs when we're in a module view (not home)
  if (state.activeTab === 'home' || !moduleLabel) {
    return null;
  }

  return (
    <nav className="breadcrumb-container" aria-label="Breadcrumb">
      <a href="/" className="breadcrumb-link">Home</a>
      <span className="breadcrumb-separator">›</span>
      <a href={`#${state.activeTab}`} className="breadcrumb-link">{moduleLabel}</a>
      <span className="breadcrumb-separator">›</span>
      <span className="breadcrumb-current">{pageTitle}</span>
    </nav>
  );
};

export default Breadcrumb;
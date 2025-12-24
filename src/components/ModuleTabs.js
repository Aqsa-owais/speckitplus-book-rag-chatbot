import React, { useState, useEffect, useContext } from 'react';
import { useTabContext } from '../context/TabContext';

const ModuleTabs = ({ children }) => {
  const { state, switchToModule, switchToHome } = useTabContext();
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  // Define the tabs
  const tabs = [
    { id: 'home', label: 'Home' },
    { id: 'module1', label: 'Module 1' },
    { id: 'module2', label: 'Module 2' },
    { id: 'module3', label: 'Module 3' },
    { id: 'module4', label: 'Module 4' },
  ];

  const handleTabClick = (tabId) => {
    if (tabId === 'home') {
      switchToHome();
    } else {
      switchToModule(tabId);
    }
  };

  // Function to determine which content to show based on active tab
  const getVisibleContent = () => {
    if (state.activeTab === 'home') {
      // Show all content for home tab
      return React.Children.toArray(children);
    } else {
      // For module tabs, only show content that matches the module
      return React.Children.toArray(children).filter(child => {
        return child.props.moduleId === state.activeTab;
      });
    }
  };

  if (!isClient) {
    // During SSR, show all content (home view) to avoid hydration issues
    return (
      <div className="module-tabs-container">
        <div className="tabs-header">
          {tabs.map(tab => (
            <button
              key={tab.id}
              className={`tab-button ${state.activeTab === tab.id ? 'active' : ''}`}
              onClick={() => handleTabClick(tab.id)}
              aria-selected={state.activeTab === tab.id}
              role="tab"
            >
              {tab.label}
            </button>
          ))}
        </div>
        <div className="tabs-content" role="tabpanel">
          {React.Children.toArray(children)}
        </div>
      </div>
    );
  }

  return (
    <div className="module-tabs-container">
      <div className="tabs-header">
        {tabs.map(tab => (
          <button
            key={tab.id}
            className={`tab-button ${state.activeTab === tab.id ? 'active' : ''}`}
            onClick={() => handleTabClick(tab.id)}
            aria-selected={state.activeTab === tab.id}
            role="tab"
          >
            {tab.label}
          </button>
        ))}
      </div>
      <div className="tabs-content" role="tabpanel">
        {getVisibleContent()}
      </div>
    </div>
  );
};

export default ModuleTabs;
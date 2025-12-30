/**
 * Initialize the global chatbot on all pages
 * This function should be called when the DOM is ready
 */

import React from 'react';
import ReactDOM from 'react-dom/client';
import GlobalChatbot from '../components/GlobalChatbot';

export const initializeGlobalChatbot = () => {
  // Wait for the DOM to be fully loaded
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', mountChatbot);
  } else {
    // DOM is already loaded, mount immediately
    mountChatbot();
  }
};

const mountChatbot = () => {
  // Look for the container we injected via the plugin
  const container = document.getElementById('global-chatbot-root');

  if (container) {
    // Add some styling to position the chatbot appropriately
    container.style.margin = '2rem auto';
    container.style.maxWidth = '1000px';
    container.style.padding = '0 1rem';

    // Create a React root and render the chatbot
    const root = ReactDOM.createRoot(container);
    root.render(
      <React.StrictMode>
        <GlobalChatbot isEmbedded={true} />
      </React.StrictMode>
    );
  }
};

// Initialize when module loads
initializeGlobalChatbot();
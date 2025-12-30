import React, { useEffect } from 'react';
import GlobalChatbot from './GlobalChatbot';

/**
 * Component that injects the chatbot into the page
 * This component can be added to all pages through Docusaurus configuration
 */
const GlobalChatbotInjector = () => {
  useEffect(() => {
    // Any initialization logic can go here if needed
  }, []);

  return (
    <div className="global-chatbot-container">
      <GlobalChatbot isEmbedded={true} />
    </div>
  );
};

export default GlobalChatbotInjector;
import React, { useEffect } from 'react';
import ReactDOM from 'react-dom/client';
import GlobalChatbot from './GlobalChatbot';

/**
 * This component mounts the GlobalChatbot to the global container
 * It's designed to be used with Docusaurus' HTML injection system
 */
const ChatbotInjector = () => {
  useEffect(() => {
    const container = document.getElementById('global-chatbot-root');
    if (container) {
      // Create a React root and render the chatbot
      const root = ReactDOM.createRoot(container);
      root.render(<GlobalChatbot isEmbedded={true} />);

      // Cleanup function
      return () => {
        root.unmount();
      };
    }
  }, []);

  // This component doesn't render anything itself,
  // it just manages mounting the chatbot to the injected div
  return null;
};

export default ChatbotInjector;
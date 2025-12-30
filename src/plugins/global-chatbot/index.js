module.exports = function (context, options) {
  return {
    name: 'global-chatbot-plugin',
    injectHtmlTags() {
      return {
        postBodyTags: [
          `<div id="global-chatbot-root" style="position: fixed; bottom: 20px; right: 20px; z-index: 1000;"></div>`,
          `<script>
            // Global chatbot initialization script
            (function() {
              // Only run if we're on the client side
              if (typeof window !== 'undefined' && typeof document !== 'undefined') {
                // Create the chatbot launcher button
                const launcher = document.createElement('div');
                launcher.id = 'chatbot-launcher';
                launcher.innerHTML = '<button id="chatbot-toggle-btn" style="position: fixed; bottom: 20px; right: 20px; width: 60px; height: 60px; border-radius: 50%; background-color: #3498db; color: white; border: none; font-size: 24px; cursor: pointer; box-shadow: 0 4px 10px rgba(0, 0, 0, 0.2); display: flex; align-items: center; justify-content: center; z-index: 1000;">💬</button>';

                // Add the launcher to the body
                document.body.appendChild(launcher);

                // Track chatbot state
                let isChatbotVisible = false;

                // Toggle chatbot visibility
                document.getElementById('chatbot-toggle-btn').addEventListener('click', function() {
                  isChatbotVisible = !isChatbotVisible;

                  if (isChatbotVisible) {
                    // Show the chatbot
                    showGlobalChatbot();
                  } else {
                    // Hide the chatbot
                    hideGlobalChatbot();
                  }
                });

                function showGlobalChatbot() {
                  const rootElement = document.getElementById('global-chatbot-root');
                  if (rootElement) {
                    rootElement.innerHTML = \`
                      <div id="global-chatbot-container" style="position: fixed; bottom: 90px; right: 20px; width: 400px; height: 600px; background: white; border-radius: 10px; box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15); z-index: 1000; display: flex; flex-direction: column; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;">
                        <div id="chat-header" style="background: #2c3e50; color: white; padding: 15px; display: flex; justify-content: space-between; align-items: center; border-top-left-radius: 10px; border-top-right-radius: 10px;">
                          <h3 style="margin: 0; font-size: 1.2rem;">RAG Chatbot</h3>
                          <div style="display: flex; align-items: center; gap: 10px;">
                            <span id="status-text" style="font-size: 0.8rem; opacity: 0.8;">Ready</span>
                            <button id="chatbot-close-btn" style="background: none; border: none; color: white; font-size: 1.5rem; cursor: pointer; padding: 0; margin: 0; width: 30px; height: 30px; display: flex; align-items: center; justify-content: center; border-radius: 50%; transition: background-color 0.2s;">×</button>
                          </div>
                        </div>

                        <div id="chat-messages" style="flex: 1; padding: 20px; overflow-y: auto; max-height: 300px; background-color: #fafafa; display: flex; flex-direction: column; gap: 10px;">
                          <div class="message bot-message" style="padding: 12px 16px; border-radius: 8px; max-width: 80%; background-color: #ecf0f1; color: #2c3e50;">
                            <strong>Assistant:</strong> Welcome! I'm your RAG assistant. Ask me questions about Physical AI Robotics and the book content.
                          </div>
                        </div>

                        <form id="chat-form" style="display: flex; padding: 15px; border-top: 1px solid #eee; background: white;">
                          <input
                            type="text"
                            id="message-input"
                            placeholder="Ask a question about Physical AI Robotics..."
                            style="flex: 1; padding: 12px 15px; border: 1px solid #ddd; border-radius: 25px; font-size: 16px; outline: none;"
                          />
                          <button
                            type="submit"
                            id="send-button"
                            style="margin-left: 10px; padding: 12px 25px; background-color: #3498db; color: white; border: none; border-radius: 25px; cursor: pointer; font-size: 16px; transition: background-color 0.3s;"
                          >
                            Send
                          </button>
                        </form>
                      </div>
                    \`;

                    // Add event listeners to the new elements
                    document.getElementById('chatbot-close-btn').addEventListener('click', function() {
                      isChatbotVisible = false;
                      hideGlobalChatbot();
                    });

                    document.getElementById('chat-form').addEventListener('submit', async function(e) {
                      e.preventDefault();

                      const input = document.getElementById('message-input');
                      const query = input.value.trim();
                      if (!query) return;

                      // Add user message to chat
                      const messagesContainer = document.getElementById('chat-messages');
                      const userMessage = document.createElement('div');
                      userMessage.className = 'message user-message';
                      userMessage.style.cssText = 'padding: 12px 16px; border-radius: 8px; max-width: 80%; background-color: #3498db; color: white; margin-left: auto; text-align: right;';
                      userMessage.innerHTML = '<strong>You:</strong> ' + query;
                      messagesContainer.appendChild(userMessage);

                      // Clear input and disable button while processing
                      input.value = '';
                      document.getElementById('send-button').disabled = true;
                      document.getElementById('status-text').textContent = 'Processing...';

                      try {
                        const response = await fetch('http://localhost:8000/chat', {
                          method: 'POST',
                          headers: {
                            'Content-Type': 'application/json',
                          },
                          body: JSON.stringify({
                            query: query,
                            top_k: 5
                          })
                        });

                        if (!response.ok) {
                          throw new Error('HTTP error! status: ' + response.status);
                        }

                        const data = await response.json();

                        // Add bot response to chat
                        const botMessage = document.createElement('div');
                        botMessage.className = 'message bot-message';
                        botMessage.style.cssText = 'padding: 12px 16px; border-radius: 8px; max-width: 80%; background-color: #ecf0f1; color: #2c3e50;';
                        botMessage.innerHTML = '<strong>Assistant:</strong> ' + (data.formatted_response || data.response || 'I received your message');
                        messagesContainer.appendChild(botMessage);

                        document.getElementById('status-text').textContent = 'Response received';
                      } catch (error) {
                        console.error('Error:', error);

                        // Add error message to chat
                        const errorMessage = document.createElement('div');
                        errorMessage.className = 'message bot-message';
                        errorMessage.style.cssText = 'padding: 12px 16px; border-radius: 8px; max-width: 80%; background-color: #ecf0f1; color: #2c3e50;';
                        errorMessage.innerHTML = '<strong>Assistant:</strong> Error: ' + error.message + '. Make sure the backend server is running on http://localhost:8000.';
                        messagesContainer.appendChild(errorMessage);

                        document.getElementById('status-text').textContent = 'Error occurred';
                      } finally {
                        document.getElementById('send-button').disabled = false;

                        // Scroll to bottom
                        messagesContainer.scrollTop = messagesContainer.scrollHeight;
                      }
                    });

                    // Scroll to bottom
                    const messagesContainer = document.getElementById('chat-messages');
                    messagesContainer.scrollTop = messagesContainer.scrollHeight;
                  }
                }

                function hideGlobalChatbot() {
                  const container = document.getElementById('global-chatbot-container');
                  if (container) {
                    container.remove();
                  }
                }
              }
            })();
          </script>`
        ],
      };
    },
  };
};
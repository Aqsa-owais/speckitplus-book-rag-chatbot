/**
 * Client-side script to initialize the global chatbot on all pages
 * This script runs in the browser after the page loads
 */

(function() {
  'use strict';

  // Wait for the DOM to be ready
  function onDomReady(callback) {
    if (document.readyState !== 'loading') {
      callback();
    } else {
      document.addEventListener('DOMContentLoaded', callback);
    }
  }

  // Function to initialize the chatbot
  function initializeChatbot() {
    // Look for the container we injected via the plugin
    const container = document.getElementById('global-chatbot-root');

    if (container) {
      // Style the container as a floating chatbot that doesn't interfere with page layout
      container.style.position = 'fixed';
      container.style.bottom = '20px';
      container.style.right = '20px';
      container.style.width = '350px';
      container.style.zIndex = '1000';
      container.style.maxHeight = '60vh';
      container.style.display = 'flex';
      container.style.flexDirection = 'column';
      container.style.fontFamily = "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif";

      // Create the chatbot HTML structure dynamically
      container.innerHTML = `
        <div style="display: flex; flex-direction: column; background: white; border-radius: 10px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); overflow: hidden; max-height: 60vh;">
          <div style="background-color: #2c3e50; color: white; padding: 15px; text-align: center;">
            <h3 style="margin: 0; font-size: 16px;">RAG Chatbot</h3>
          </div>
          <div id="messages-container" style="flex: 1; padding: 15px; overflow-y: auto; max-height: 300px; background-color: #fafafa;">
            <div class="message bot-message" style="margin-bottom: 10px; padding: 10px 12px; border-radius: 8px; max-width: 80%; background-color: #ecf0f1; color: #2c3e50; font-size: 14px;">
              <strong>Assistant:</strong> Welcome! I'm your RAG assistant. Ask me questions about Physical AI Robotics and the book content.
            </div>
          </div>
          <form id="chat-form" style="display: flex; padding: 10px; border-top: 1px solid #eee; background: white;">
            <input
              type="text"
              id="message-input"
              placeholder="Ask question..."
              style="flex: 1; padding: 10px 12px; border: 1px solid #ddd; border-radius: 20px; font-size: 14px; outline: none;"
            />
            <button
              type="submit"
              id="send-button"
              style="margin-left: 8px; padding: 10px 16px; background-color: #3498db; color: white; border: none; border-radius: 20px; cursor: pointer; font-size: 14px; transition: background-color 0.3s;"
            >
              Send
            </button>
          </form>
          <div id="status-bar" style="padding: 8px 10px; text-align: center; color: #7f8c8d; font-size: 12px; background: white; border-top: 1px solid #eee;">
            <span>Status: <span id="status-text">Ready</span></span>
          </div>
        </div>
      `;

      // Add JavaScript functionality to the chatbot
      const form = document.getElementById('chat-form');
      const input = document.getElementById('message-input');
      const messagesContainer = document.getElementById('messages-container');
      const sendButton = document.getElementById('send-button');
      const statusText = document.getElementById('status-text');

      // Function to add a message to the chat
      function addMessage(text, sender) {
        const messageDiv = document.createElement('div');
        messageDiv.className = `message ${sender}-message`;
        messageDiv.style.marginBottom = '15px';
        messageDiv.style.padding = '12px 16px';
        messageDiv.style.borderRadius = '8px';
        messageDiv.style.maxWidth = '80%';

        if (sender === 'user') {
          messageDiv.style.backgroundColor = '#3498db';
          messageDiv.style.color = 'white';
          messageDiv.style.marginLeft = 'auto';
          messageDiv.style.textAlign = 'right';
        } else {
          messageDiv.style.backgroundColor = '#ecf0f1';
          messageDiv.style.color = '#2c3e50';
        }

        messageDiv.innerHTML = `<strong>${sender === 'user' ? 'You:' : 'Assistant:'}</strong> ${text}`;
        messagesContainer.appendChild(messageDiv);

        // Scroll to bottom
        messagesContainer.scrollTop = messagesContainer.scrollHeight;
      }

      // Function to update status
      function updateStatus(text) {
        statusText.textContent = text;
        setTimeout(() => {
          if (statusText.textContent === text) {
            statusText.textContent = 'Ready';
          }
        }, 3000);
      }

      // Form submission handler
      form.addEventListener('submit', async function(e) {
        e.preventDefault();

        const query = input.value.trim();
        if (!query) return;

        // Add user message
        addMessage(query, 'user');
        input.value = '';
        sendButton.disabled = true;
        updateStatus('Processing...');

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
            throw new Error(`HTTP error! status: ${response.status}`);
          }

          const data = await response.json();
          const botResponse = data.formatted_response || data.response || 'I received your message';

          // Add bot response
          addMessage(botResponse, 'bot');
          updateStatus('Response received');
        } catch (error) {
          console.error('Error:', error);
          addMessage(`Error: ${error.message}. Make sure the backend server is running on http://localhost:8000.`, 'bot');
          updateStatus('Error occurred');
        } finally {
          sendButton.disabled = false;
        }
      });
    }
  }

  // Initialize when DOM is ready
  onDomReady(initializeChatbot);

})();
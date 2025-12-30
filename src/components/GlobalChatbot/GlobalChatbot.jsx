import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './global-chatbot.module.css';

const GlobalChatbot = ({ isOpen, onClose, onToggle }) => {
  const [messages, setMessages] = useState([
    { id: 1, text: "Hello! I'm your RAG assistant. Ask me questions about the Physical AI Robotics book content.", sender: 'bot' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [status, setStatus] = useState('Ready');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setStatus('Processing...');

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          top_k: 5
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.formatted_response || data.response || 'I received your message',
        sender: 'bot'
      };

      setMessages(prev => [...prev, botMessage]);
      setStatus('Response received');
    } catch (error) {
      console.error('Error:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: `Error: ${error.message}. Make sure the backend server is running on http://localhost:8000.`,
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
      setStatus('Error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) {
    // Show only the toggle button when chatbot is closed
    return (
      <div className={styles.chatbotLauncher}>
        <button
          className={styles.launchButton}
          onClick={onToggle}
          aria-label="Open chatbot"
        >
          💬
        </button>
      </div>
    );
  }

  return (
    <div className={styles.globalChatbot}>
      <div className={styles.chatHeader}>
        <h3>RAG Chatbot</h3>
        <div className={styles.headerControls}>
          <span className={styles.statusText}>Status: {status}</span>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close chatbot"
          >
            ×
          </button>
        </div>
      </div>

      <div className={styles.chatContainer}>
        <div className={styles.messagesContainer}>
          {messages.map((message) => (
            <div
              key={message.id}
              className={`${styles.message} ${styles[message.sender + 'Message']}`}
            >
              <strong>{message.sender === 'user' ? 'You:' : 'Assistant:'}</strong> {message.text}
            </div>
          ))}
          {isLoading && (
            <div className={`${styles.message} ${styles.botMessage}`}>
              <strong>Assistant:</strong> Thinking...
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <form onSubmit={handleSubmit} className={styles.inputContainer}>
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Ask about Physical AI Robotics..."
            disabled={isLoading}
            className={styles.messageInput}
          />
          <button
            type="submit"
            disabled={isLoading || !inputValue.trim()}
            className={styles.sendButton}
          >
            Send
          </button>
        </form>
      </div>
    </div>
  );
};

export default GlobalChatbot;
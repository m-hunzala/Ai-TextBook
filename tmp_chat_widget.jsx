import React, { useState, useRef, useEffect } from 'react';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Function to handle text selection
  useEffect(() => {
    const handleTextSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, []);

  // Function to scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to handle sending a message
  const handleSend = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    const newMessages = [...messages, userMessage];
    setMessages(newMessages);
    setInputValue('');
    setIsLoading(true);

    try {
      let response;
      if (selectedText) {
        // Send selected text query
        response = await fetch('http://localhost:8000/api/query-selected', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            selected_text: selectedText,
            query: inputValue,
            history: newMessages.filter(msg => msg.sender === 'user').map(msg => ({
              role: 'user',
              content: msg.text
            }))
          })
        });
        setSelectedText(''); // Clear selected text after using it
      } else {
        // Send normal query
        response = await fetch('http://localhost:8000/api/query', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            query: inputValue,
            history: newMessages.filter(msg => msg.sender === 'user').map(msg => ({
              role: 'user',
              content: msg.text
            }))
          })
        });
      }

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();
      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot'
      };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to handle context menu for selected text
  useEffect(() => {
    const handleContextMenu = (e) => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        // Create a custom context menu or use a library for this
        // For now, we'll just store the selected text
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('contextmenu', handleContextMenu);
    return () => {
      document.removeEventListener('contextmenu', handleContextMenu);
    };
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  return (
    <div className="chat-widget">
      {isOpen && (
        <div className="chat-panel">
          <div className="chat-header">
            <h3>AI Assistant</h3>
            <button className="chat-close-btn" onClick={toggleChat}>
              ×
            </button>
          </div>
          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="chat-welcome">
                <p>Hello! I'm your AI assistant for this book. How can I help you today?</p>
                {selectedText && (
                  <div className="selected-text-preview">
                    <small>Selected: "{selectedText.substring(0, 50)}..."</small>
                  </div>
                )}
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`chat-message ${message.sender}-message`}
                >
                  <div className="message-content">{message.text}</div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="chat-message bot-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form className="chat-input-form" onSubmit={handleSend}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Type your message..."
              disabled={isLoading}
              className="chat-input"
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading}
              className="chat-send-btn"
            >
              Send
            </button>
          </form>
        </div>
      )}
      <button className="chat-toggle-btn" onClick={toggleChat}>
        {isOpen ? 'Close' : 'Chat'}
      </button>
    </div>
  );
};

export default ChatWidget;
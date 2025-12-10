import React, { useEffect, useRef, useState } from 'react';
import ClientOnly from '@docusaurus/ClientOnly';
import './ChatWidget.css';

/**
 * Client-only component to avoid using document/window during SSR.
 * Move any DOM or global-window usage into the useEffect below or dynamically import modules there.
 */
function ChatWidgetClient() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  useEffect(() => {
    // Safe: runs only in the browser
    const handleTextSelection = () => {
      const sel = window.getSelection?.().toString().trim();
      if (sel) setSelectedText(sel);
    };
    const handleContextMenu = (e) => {
      // optional: prevent default or show custom menu
    };

    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('contextmenu', handleContextMenu);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('contextmenu', handleContextMenu);
    };
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSend = async (e) => {
    e?.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    const newMessages = [...messages, userMessage];
    setMessages(newMessages);
    setInputValue('');
    setIsLoading(true);

    try {
      let response;
      if (selectedText) {
        response = await fetch('http://localhost:8000/api/query-selected', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            selected_text: selectedText,
            query: inputValue,
            history: newMessages.filter(m => m.sender === 'user').map(msg => ({ role: 'user', content: msg.text }))
          })
        });
        setSelectedText('');
      } else {
        response = await fetch('http://localhost:8000/api/query', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            query: inputValue,
            history: newMessages.filter(m => m.sender === 'user').map(msg => ({ role: 'user', content: msg.text }))
          })
        });
      }

      if (!response.ok) throw new Error(`API request failed: ${response.status}`);
      const data = await response.json();
      setMessages(prev => [...prev, { id: Date.now() + 1, text: data.response, sender: 'bot' }]);
    } catch (err) {
      console.error('Error sending message:', err);
      setMessages(prev => [...prev, { id: Date.now() + 1, text: 'Sorry, I encountered an error.', sender: 'bot' }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div id="tmp-chat-widget" className="chat-widget-root">
      <div className="chat-header" onClick={() => setIsOpen(!isOpen)}>
        Chat
      </div>
      {isOpen && (
        <div className="chat-body">
          <div className="messages">
            {messages.map(m => (
              <div key={m.id} className={`message ${m.sender}`}>{m.text}</div>
            ))}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSend} className="chat-input-form">
            <input
              ref={inputRef}
              value={inputValue}
              onChange={e => setInputValue(e.target.value)}
              placeholder="Type a message..."
            />
            <button type="submit" disabled={isLoading}>Send</button>
          </form>
        </div>
      )}
    </div>
  );
}

export default function TmpChatWidget() {
  return (
    <ClientOnly fallback={null}>
      <ChatWidgetClient />
    </ClientOnly>
  );
}

import React, { useState, useEffect, useRef } from 'react';
import './ChatWidget.css';

const ChatWidget = ({ apiUrl, apiKey }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [useSelectedText, setUseSelectedText] = useState(false);
  const [isSelectingForRAG, setIsSelectingForRAG] = useState(false);
  const [rateLimitMessage, setRateLimitMessage] = useState('');
  const [highlightedSource, setHighlightedSource] = useState(null);
  const [showExpertSkills, setShowExpertSkills] = useState(false);
  const [availableSubagents, setAvailableSubagents] = useState([]);
  const [selectedSubagent, setSelectedSubagent] = useState(null);
  const [subagentQuery, setSubagentQuery] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Function to get available subagents
  useEffect(() => {
    const fetchSubagents = async () => {
      try {
        const response = await fetch(`${apiUrl}/subagents`, {
          headers: {
            'Authorization': `Bearer ${apiKey}`,
            'Content-Type': 'application/json'
          }
        });

        if (response.ok) {
          const data = await response.json();
          setAvailableSubagents(data.subagents);
        }
      } catch (error) {
        console.error('Error fetching subagents:', error);
      }
    };

    fetchSubagents();
  }, [apiUrl, apiKey]);

  // Function to handle text selection on the page for RAG
  useEffect(() => {
    if (typeof window !== 'undefined' && typeof document !== 'undefined') {
      const handleTextSelection = () => {
        if (!isSelectingForRAG) return;

        const selection = window.getSelection();
        if (selection && selection.toString().trim() !== '') {
          const text = selection.toString().trim();
          if (text.length > 10) { // Only capture meaningful selections
            setSelectedText(text);
            setUseSelectedText(true);
            setError('');

            // Show temporary notification to user
            console.log('Selected text captured for RAG:', text.substring(0, 50) + '...');

            // Reset selection mode after capturing
            setIsSelectingForRAG(false);
          }
        }
      };

      document.addEventListener('mouseup', handleTextSelection);

      return () => {
        document.removeEventListener('mouseup', handleTextSelection);
      };
    }
  }, [isSelectingForRAG]);

  // Function to get selected text from the page with additional info
  const getSelectedText = () => {
    if (typeof window === 'undefined' || typeof document === 'undefined') {
      return null;
    }

    const selection = window.getSelection();
    if (!selection || selection.toString().trim() === '') {
      return null;
    }

    const text = selection.toString().trim();
    const range = selection.getRangeAt(0);
    const selectedElement = range.commonAncestorContainer;

    // Try to get the closest heading element as context
    let headingElement = null;
    let current = selectedElement;
    while (current && current !== document.body) {
      if (current.tagName && current.tagName.match(/^H[1-6]$/)) {
        headingElement = current;
        break;
      }
      current = current.parentElement || current.parentNode;
    }

    const docTitle = typeof document !== 'undefined' ? document.title : '';
    const docUrl = typeof window !== 'undefined' ? window.location.href : '';

    return {
      text,
      title: headingElement ? headingElement.textContent : docTitle,
      url: docUrl,
      element: selectedElement
    };
  };

  // State for popover
  const [popover, setPopover] = useState({
    visible: false,
    source: null,
    position: { x: 0, y: 0 }
  });

  // Function to handle source link click
  const handleSourceClick = (e, source) => {
    e.preventDefault();

    // Get the position of the clicked element for the popover
    const rect = e.target.getBoundingClientRect();

    // Update popover state to show the citation
    setPopover({
      visible: true,
      source: source,
      position: {
        x: rect.left + (typeof window !== 'undefined' ? window.scrollX : 0),
        y: rect.top + (typeof window !== 'undefined' ? window.scrollY : 0) - 10
      }
    });

    // Try to find and highlight the text in the document
    const success = highlightTextInDocument(source);

    if (success) {
      setHighlightedSource(source);
      // Clear highlighting after 5 seconds
      setTimeout(() => {
        clearHighlighting();
        setHighlightedSource(null);
      }, 5000);
    } else {
      // If we can't find the exact text, navigate to the URL with anchor if available
      if (source.url && typeof window !== 'undefined' && typeof document !== 'undefined') {
        const [baseUrl, anchor] = source.url.split('#');
        if (anchor) {
          const element = document.getElementById(anchor) || document.querySelector(`[name="${anchor}"]`);
          if (element) {
            element.scrollIntoView({ behavior: 'smooth', block: 'center' });
            setHighlightedSource(source);
            // Clear highlighting after 5 seconds
            setTimeout(() => {
              clearHighlighting();
              setHighlightedSource(null);
            }, 5000);
          } else {
            // If no anchor element found, just navigate to the URL
            window.open(source.url, '_blank');
          }
        } else {
          window.open(source.url, '_blank');
        }
      }
    }
  };

  // Close the popover
  const closePopover = () => {
    setPopover({ visible: false, source: null, position: { x: 0, y: 0 } });
  };

  // Close popover when clicking outside
  useEffect(() => {
    if (typeof document !== 'undefined') {
      const handleClickOutside = (event) => {
        if (popover.visible && !event.target.closest('.source-popover')) {
          closePopover();
        }
        if (showExpertSkills && !event.target.closest('.expert-skill-selector')) {
          setShowExpertSkills(false);
        }
      };

      document.addEventListener('mousedown', handleClickOutside);
      return () => {
        document.removeEventListener('mousedown', handleClickOutside);
      };
    }
  }, [popover.visible, showExpertSkills]);

  // Function to toggle expert skills dropdown
  const toggleExpertSkills = (e) => {
    e.stopPropagation();
    setShowExpertSkills(!showExpertSkills);
  };

  // Function to select a subagent
  const selectSubagent = (subagent) => {
    setSelectedSubagent(subagent);
    setSubagentQuery(inputValue);
    setShowExpertSkills(false);
  };

  // Function to call a subagent
  const callSubagent = async (subagentName, queryText) => {
    setIsLoading(true);
    setError('');

    const newMessage = { id: Date.now(), text: queryText, sender: 'user', timestamp: new Date() };
    setMessages(prev => [...prev, newMessage]);

    try {
      const response = await fetch(`${apiUrl}/subagent-call`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${apiKey}`
        },
        body: JSON.stringify({
          subagent_name: subagentName,
          query: queryText
        })
      });

      if (response.status === 401) {
        throw new Error('Authentication failed for subagent call. Please check your API key.');
      }
      if (response.status === 403) {
        throw new Error('Access forbidden for subagent call. Please check your authentication credentials.');
      }
      if (!response.ok) {
        throw new Error(`Subagent API request failed with status ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: `Expert assistance from ${subagentName}:`,
        sender: 'bot',
        subagentResult: data.result,
        subagentName: subagentName,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
      setInputValue('');
      setSubagentQuery('');
      setSelectedSubagent(null);
    } catch (err) {
      console.error('Error calling subagent:', err);
      const errorMessage = {
        id: Date.now() + 1,
        text: `Error calling ${selectedSubagent?.name}: ${err.message}`,
        sender: 'bot',
        isError: true,
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to highlight text in the document
  const highlightTextInDocument = (source) => {
    if (typeof document === 'undefined' || typeof NodeFilter === 'undefined') {
      return false;
    }

    const textToFind = source.snippet ? source.snippet.replace(/\.{3}$/, '').trim() : '';

    if (!textToFind) return false;

    // Remove any highlighting spans we may have added before
    clearHighlighting();

    // Search for the text in the document
    const walker = document.createTreeWalker(
      document.body,
      NodeFilter.SHOW_TEXT,
      {
        acceptNode: function(node) {
          return node.nodeValue && node.nodeValue.trim() !== ''
            ? NodeFilter.FILTER_ACCEPT
            : NodeFilter.FILTER_REJECT;
        }
      }
    );

    const textNodes = [];
    let node;

    // Collect all text nodes
    while (node = walker.nextNode()) {
      textNodes.push(node);
    }

    // Look for the text in text nodes
    for (let textNode of textNodes) {
      const nodeValue = textNode.nodeValue;
      const index = nodeValue.toLowerCase().indexOf(textToFind.toLowerCase());

      if (index !== -1) {
        // Found the text, now highlight it
        const range = document.createRange();
        range.setStart(textNode, index);
        range.setEnd(textNode, index + textToFind.length);

        // Create a highlight element
        const highlightElement = document.createElement('mark');
        highlightElement.className = 'chat-widget-highlight';
        highlightElement.style.backgroundColor = '#ffeb3b';
        highlightElement.style.padding = '2px 4px';
        highlightElement.style.borderRadius = '3px';
        highlightElement.style.animation = 'pulse 1.5s infinite';

        // Surround the content with the highlight
        range.surroundContents(highlightElement);

        // Scroll the highlighted element into view
        highlightElement.scrollIntoView({ behavior: 'smooth', block: 'center' });

        return true;
      }
    }

    return false;
  };

  // Function to clear highlighting
  const clearHighlighting = () => {
    if (typeof document === 'undefined') {
      return;
    }

    const highlightedElements = document.querySelectorAll('.chat-widget-highlight');
    highlightedElements.forEach(el => {
      const parent = el.parentNode;
      while (el.firstChild) {
        parent.insertBefore(el.firstChild, el);
      }
      parent.removeChild(el);
    });
  };

  // Handle capturing selected text
  const handleCaptureSelectedText = () => {
    if (typeof window === 'undefined') {
      return;
    }

    const selection = window.getSelection();
    if (selection && selection.toString().trim() !== '') {
      // If there's already selected text, use it directly
      const text = selection.toString().trim();
      if (text.length > 10) {
        setSelectedText(text);
        setUseSelectedText(true);
        setError('');

        // Show confirmation to user
        console.log('Captured selected text:', text.substring(0, 50) + '...');
      } else {
        setError('Please select more text (at least 10 characters)');
        setTimeout(() => setError(''), 3000);
      }
    } else {
      // Enable selection mode - user needs to select text on the page
      setIsSelectingForRAG(true);
      setError('Please select text on the page now. Click "Answer from selected text" again after selecting.');
      setTimeout(() => setError(''), 5000);
    }
  };

  // Toggle the chat widget
  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 100);
    }
  };

  // Scroll to bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to send query to the API
  const sendMessage = async (text) => {
    if (!text.trim() || isLoading) return;

    setIsLoading(true);
    setError('');
    setRateLimitMessage('');

    const newMessage = { id: Date.now(), text, sender: 'user', timestamp: new Date() };
    setMessages(prev => [...prev, newMessage]);

    try {
      let response;
      
      if (useSelectedText && selectedText) {
        // Use selected text mode
        response = await fetch(`${apiUrl}/answer`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${apiKey}`
          },
          body: JSON.stringify({
            query: text,
            selected_text: selectedText
          })
        });
      } else {
        // Standard RAG mode - first get retrieved context, then generate answer
        // Actually, for simplicity and efficiency, we'll use the same /answer endpoint
        // which can handle both modes. The backend already supports this.
        response = await fetch(`${apiUrl}/answer`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${apiKey}`
          },
          body: JSON.stringify({
            query: text
          })
        });
      }

      if (response.status === 429) {
        setRateLimitMessage('Rate limit exceeded. Please try again in a moment.');
        setIsLoading(false);
        return;
      }

      if (response.status === 401) {
        throw new Error('Authentication failed. Please check your API key.');
      }
      if (response.status === 403) {
        throw new Error('Access forbidden. Please check your authentication credentials.');
      }
      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      // Process sources to extract and format document links
      const sources = (data.retrieved_context || []).map((context, index) => {
        const metadata = context.metadata || {};

        // Construct document URL with proper path
        let docUrl = '';
        if (metadata.source) {
          // Create URL based on Docusaurus structure
          const cleanSource = metadata.source.replace(/\.mdx?$/, '');
          docUrl = `/${cleanSource}`;

          // If it's in docs directory, format as Docusaurus docs
          if (cleanSource.startsWith('docs/')) {
            docUrl = `/docs/${cleanSource.replace('docs/', '')}`;
          }
        } else if (metadata.url) {
          docUrl = metadata.url;
        } else {
          docUrl = typeof window !== 'undefined' ? window.location.origin : ''; // fallback
        }

        return {
          id: index,
          url: docUrl,
          title: metadata.title || metadata.source || 'Documentation',
          snippet: context.text ? context.text.substring(0, 100) + '...' : '',
          score: context.score || 0
        };
      });

      // Check if the response contains subagent results (for auto-detected subagent queries)
      const botMessage = {
        id: Date.now() + 1,
        text: data.answer || 'No response generated',
        sender: 'bot',
        sources: sources,
        timestamp: new Date()
      };

      // Add subagent data if present in trace
      if (data.trace && data.trace.subagent_used && data.trace.subagent_result) {
        botMessage.subagentResult = data.trace.subagent_result;
        botMessage.subagentName = data.trace.subagent_used;
      }

      setMessages(prev => [...prev, botMessage]);
      
      // Reset selected text mode after sending
      if (useSelectedText) {
        setUseSelectedText(false);
        setSelectedText('');
      }
    } catch (err) {
      console.error('Error sending message:', err);
      const errorMessage = {
        id: Date.now() + 1,
        text: `Error: ${err.message || 'Failed to get response'}`,
        sender: 'bot',
        isError: true,
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setInputValue('');
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();

    if (selectedSubagent) {
      callSubagent(selectedSubagent.name, inputValue || subagentQuery);
    } else {
      sendMessage(inputValue);
    }
  };

  const clearChat = () => {
    setMessages([]);
    setError('');
    setRateLimitMessage('');
  };

  return (
    <>
      {/* Chat Widget Button */}
      <button className="chat-widget-button" onClick={toggleChat}>
        💬
      </button>

      {/* Chat Widget Panel */}
      {isOpen && (
        <div className="chat-widget-panel">
          <div className="chat-widget-header">
            <div className="chat-header-title">AI Assistant {isSelectingForRAG && <span className="selecting-indicator">🔍</span>}</div>
            <div className="chat-header-actions">
              <div className="expert-skill-selector">
                <button
                  className="expert-skill-button"
                  onClick={toggleExpertSkills}
                  title="Use expert skill"
                >
                  Use Expert Skill
                </button>
                {showExpertSkills && (
                  <div className="expert-skill-dropdown">
                    {availableSubagents.map((subagent, index) => (
                      <div
                        key={index}
                        className="expert-skill-option"
                        onClick={() => selectSubagent(subagent)}
                      >
                        <span className="expert-skill-option-title">{subagent.name}</span>
                        <span className="expert-skill-option-description">{subagent.description}</span>
                      </div>
                    ))}
                  </div>
                )}
              </div>
              <button
                className={`selected-text-toggle ${useSelectedText ? 'active' : isSelectingForRAG ? 'selecting' : ''}`}
                onClick={handleCaptureSelectedText}
                title="Use selected text for answering"
              >
                {isSelectingForRAG ? 'Selecting Text...' : 'Answer from selected text'}
              </button>
              <button className="chat-clear-button" onClick={clearChat} title="Clear chat">
                ✕
              </button>
            </div>
          </div>
          
          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="chat-welcome-message">
                <p>Ask me anything about the documentation!</p>
                {selectedText && (
                  <div className="selected-text-preview">
                    <small>Using selected text: "{selectedText.substring(0, 50)}..."</small>
                  </div>
                )}
              </div>
            ) : (
              messages.map((message) => (
                <div 
                  key={message.id} 
                  className={`chat-message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
                >
                  <div className="message-content">
                    {message.sender === 'bot' && message.sources && message.sources.length > 0 && (
                      <div className="message-sources">
                        {message.sources.map((source, idx) => (
                          <div key={idx} className="source-item">
                            <a
                              href="#"
                              onClick={(e) => handleSourceClick(e, source)}
                              className="source-link"
                              title="Click to highlight this passage in the document"
                            >
                              {source.title || 'Source'}
                              {source.url && (
                                <span className="source-anchor">
                                  {source.url.includes('#') ? source.url.split('#')[1] : ''}
                                </span>
                              )}
                            </a>
                          </div>
                        ))}
                      </div>
                    )}
                    {message.subagentResult ? (
                      <div className="subagent-result">
                        <div className="subagent-result-header">
                          {message.text} ({message.subagentName})
                        </div>
                        {message.subagentResult.summary && (
                          <div><strong>Summary:</strong> {message.subagentResult.summary}</div>
                        )}
                        {message.subagentResult.code_snippets && message.subagentResult.code_snippets.length > 0 && (
                          <div>
                            <strong>Code Snippets:</strong>
                            {message.subagentResult.code_snippets.map((snippet, idx) => (
                              <div key={idx} className="subagent-code-block">
                                <div><em>{snippet.title}</em> ({snippet.language}):</div>
                                <pre>{snippet.code}</pre>
                                {snippet.description && <div><small>{snippet.description}</small></div>}
                              </div>
                            ))}
                          </div>
                        )}
                        {message.subagentResult.config_files && message.subagentResult.config_files.length > 0 && (
                          <div>
                            <strong>Config Files:</strong>
                            {message.subagentResult.config_files.map((config, idx) => (
                              <div key={idx} className="subagent-code-block">
                                <div><em>{config.title}</em> ({config.type}):</div>
                                <pre>{config.content}</pre>
                                {config.description && <div><small>{config.description}</small></div>}
                              </div>
                            ))}
                          </div>
                        )}
                        {message.subagentResult.commands && message.subagentResult.commands.length > 0 && (
                          <div>
                            <strong>Commands:</strong>
                            <ul>
                              {message.subagentResult.commands.map((cmd, idx) => (
                                <li key={idx}>{cmd}</li>
                              ))}
                            </ul>
                          </div>
                        )}
                        {message.subagentResult.explanation && (
                          <div><strong>Explanation:</strong> {message.subagentResult.explanation}</div>
                        )}
                      </div>
                    ) : (
                      <div className="message-text">
                        {message.text}
                      </div>
                    )}
                  </div>
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
          
          {error && (
            <div className="chat-error">
              {error}
            </div>
          )}
          
          {rateLimitMessage && (
            <div className="chat-rate-limit">
              {rateLimitMessage}
            </div>
          )}
          
          <form onSubmit={selectedSubagent ? (e) => { e.preventDefault(); callSubagent(selectedSubagent.name, inputValue || subagentQuery); } : handleSubmit} className="chat-input-form">
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder={useSelectedText ? "Ask about selected text..." : selectedSubagent ? `Ask ${selectedSubagent.name}...` : "Type your question..."}
              disabled={isLoading}
              className="chat-input"
            />
            {selectedSubagent && (
              <button
                type="button"
                onClick={() => callSubagent(selectedSubagent.name, inputValue || subagentQuery)}
                disabled={!inputValue.trim() || isLoading}
                className="chat-send-button"
                style={{ backgroundColor: '#ff6b6b' }}
              >
                Ask {selectedSubagent.name}
              </button>
            )}
            <button
              type="submit"
              disabled={(!inputValue.trim() && !selectedSubagent) || isLoading}
              className="chat-send-button"
            >
              Send
            </button>
          </form>
        </div>
      )}
      {/* Source Citation Popover */}
      {popover.visible && popover.source && (
        <div
          className="source-popover"
          style={{
            position: 'absolute',
            left: `${popover.position.x}px`,
            top: `${popover.position.y}px`,
            zIndex: 10000
          }}
          onClick={(e) => e.stopPropagation()}
        >
          <div className="source-popover-title">
            {popover.source.title || 'Source'}
          </div>
          <div className="source-popover-content">
            {popover.source.snippet ? (
              <div>
                <strong>Excerpt:</strong> {popover.source.snippet}
              </div>
            ) : (
              <div>Referenced content in document</div>
            )}
          </div>
          <a
            href={popover.source.url}
            target="_blank"
            rel="noopener noreferrer"
            className="source-popover-link"
            onClick={closePopover}
          >
            Open in chapter
          </a>
        </div>
      )}
    </>
  );
};

export default ChatWidget;
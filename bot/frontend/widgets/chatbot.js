/**
 * RAG Chatbot Widget for Docusaurus
 * Provides a floating chat icon and "Ask about this selection" functionality
 */

class RAGChatbotWidget {
  constructor(config = {}) {
    this.config = {
      apiEndpoint: config.apiEndpoint || 'http://localhost:8000',
      apiKey: config.apiKey || '',
      chatIcon: config.chatIcon || '💬',
      ...config
    };
    
    this.isOpen = false;
    this.sessionId = null;
    this.selectedText = null;
    
    this.initializeWidget();
  }
  
  initializeWidget() {
    // Create the main chat container
    this.createChatInterface();
    
    // Add event listeners for text selection
    document.addEventListener('mouseup', this.handleTextSelection.bind(this));
    
    // Add the widget to the page
    document.body.appendChild(this.chatContainer);
  }
  
  createChatInterface() {
    // Create the main container
    this.chatContainer = document.createElement('div');
    this.chatContainer.id = 'rag-chatbot-container';
    this.chatContainer.innerHTML = `
      <div id="chatbot-icon" class="chatbot-icon">
        ${this.config.chatIcon}
      </div>
      <div id="chatbot-window" class="chatbot-window">
        <div class="chatbot-header">
          <div class="chatbot-title">RAG Chatbot</div>
          <button id="chatbot-close" class="chatbot-close">✕</button>
        </div>
        <div id="chatbot-messages" class="chatbot-messages">
          <div class="message bot-message">
            Hello! I can help answer questions about this documentation. 
            You can ask me anything about the content, or select text and click "Ask about this selection".
          </div>
        </div>
        <div class="chatbot-input-area">
          <input type="text" id="chatbot-input" placeholder="Ask a question..." />
          <button id="chatbot-send" class="chatbot-send">Send</button>
        </div>
        <div id="chatbot-highlight-btn" class="chatbot-highlight-btn hidden">
          Ask about this selection
        </div>
      </div>
    `;
    
    // Add styles
    this.addStyles();
    
    // Add event listeners
    this.addEventListeners();
  }
  
  addStyles() {
    const style = document.createElement('style');
    style.textContent = `
      #rag-chatbot-container {
        position: fixed;
        bottom: 20px;
        right: 20px;
        z-index: 10000;
        font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
      }
      
      .chatbot-icon {
        width: 60px;
        height: 60px;
        border-radius: 50%;
        background: #4f46e5;
        color: white;
        display: flex;
        align-items: center;
        justify-content: center;
        font-size: 24px;
        cursor: pointer;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
        transition: all 0.3s ease;
        position: relative;
      }
      
      .chatbot-icon:hover {
        transform: scale(1.05);
        box-shadow: 0 6px 16px rgba(0, 0, 0, 0.2);
      }
      
      .chatbot-window {
        position: absolute;
        bottom: 70px;
        right: 0;
        width: 380px;
        height: 500px;
        background: white;
        border-radius: 12px;
        box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2);
        display: flex;
        flex-direction: column;
        overflow: hidden;
        transform: translateY(20px);
        opacity: 0;
        transition: all 0.3s ease;
      }
      
      .chatbot-window.open {
        transform: translateY(0);
        opacity: 1;
      }
      
      .chatbot-header {
        background: #4f46e5;
        color: white;
        padding: 15px;
        display: flex;
        justify-content: space-between;
        align-items: center;
      }
      
      .chatbot-title {
        font-weight: 600;
        font-size: 16px;
      }
      
      .chatbot-close {
        background: none;
        border: none;
        color: white;
        font-size: 18px;
        cursor: pointer;
        padding: 0;
        width: 30px;
        height: 30px;
        display: flex;
        align-items: center;
        justify-content: center;
      }
      
      .chatbot-messages {
        flex: 1;
        padding: 15px;
        overflow-y: auto;
        display: flex;
        flex-direction: column;
        gap: 12px;
        background: #f9fafb;
      }
      
      .message {
        max-width: 80%;
        padding: 10px 15px;
        border-radius: 18px;
        font-size: 14px;
        line-height: 1.4;
      }
      
      .user-message {
        align-self: flex-end;
        background: #4f46e5;
        color: white;
        border-bottom-right-radius: 4px;
      }
      
      .bot-message {
        align-self: flex-start;
        background: white;
        color: #374151;
        border: 1px solid #e5e7eb;
        border-bottom-left-radius: 4px;
      }
      
      .sources {
        font-size: 12px;
        margin-top: 8px;
        padding-top: 8px;
        border-top: 1px solid #e5e7eb;
      }
      
      .source-item {
        margin-top: 5px;
        padding: 5px;
        background: #f3f4f6;
        border-radius: 4px;
        font-size: 11px;
      }
      
      .chatbot-input-area {
        padding: 15px;
        background: white;
        border-top: 1px solid #e5e7eb;
        display: flex;
        gap: 8px;
      }
      
      #chatbot-input {
        flex: 1;
        padding: 10px 15px;
        border: 1px solid #d1d5db;
        border-radius: 20px;
        font-size: 14px;
        outline: none;
      }
      
      #chatbot-input:focus {
        border-color: #4f46e5;
      }
      
      .chatbot-send {
        background: #4f46e5;
        color: white;
        border: none;
        border-radius: 20px;
        padding: 0 20px;
        cursor: pointer;
        font-size: 14px;
      }
      
      .chatbot-send:hover {
        background: #4338ca;
      }
      
      .chatbot-highlight-btn {
        position: fixed;
        background: #4f46e5;
        color: white;
        padding: 8px 16px;
        border-radius: 20px;
        font-size: 14px;
        cursor: pointer;
        z-index: 10001;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
        transform: translate(-50%, -20px);
      }
      
      .chatbot-highlight-btn.hidden {
        display: none;
      }
      
      .chatbot-highlight-btn:hover {
        background: #4338ca;
      }
    `;
    
    document.head.appendChild(style);
  }
  
  addEventListeners() {
    const chatIcon = this.chatContainer.querySelector('#chatbot-icon');
    const chatClose = this.chatContainer.querySelector('#chatbot-close');
    const chatInput = this.chatContainer.querySelector('#chatbot-input');
    const chatSend = this.chatContainer.querySelector('#chatbot-send');
    const chatMessages = this.chatContainer.querySelector('#chatbot-messages');
    
    // Toggle chat window
    chatIcon.addEventListener('click', () => {
      this.toggleChatWindow();
    });
    
    // Close chat window
    chatClose.addEventListener('click', () => {
      this.closeChatWindow();
    });
    
    // Send message on button click
    chatSend.addEventListener('click', () => {
      this.sendMessage();
    });
    
    // Send message on Enter key
    chatInput.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') {
        this.sendMessage();
      }
    });
    
    // Add click listener to hide highlight button when clicking elsewhere
    document.addEventListener('click', (e) => {
      if (!e.target.closest('#chatbot-highlight-btn') && 
          !e.target.closest('.chatbot-highlight-btn')) {
        this.hideHighlightButton();
      }
    });
  }
  
  handleTextSelection() {
    const selectedText = window.getSelection().toString().trim();
    
    if (selectedText && selectedText.length > 5) { // At least 5 characters
      this.selectedText = selectedText;
      this.showHighlightButton();
    } else {
      this.hideHighlightButton();
    }
  }
  
  showHighlightButton() {
    const selection = window.getSelection();
    if (selection.rangeCount === 0) return;
    
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();
    
    const highlightBtn = this.chatContainer.querySelector('#chatbot-highlight-btn');
    highlightBtn.style.display = 'block';
    highlightBtn.style.left = rect.left + 'px';
    highlightBtn.style.top = (rect.top - 10) + 'px';
    
    // Add click event to the highlight button
    highlightBtn.onclick = () => {
      this.askAboutSelection();
    };
  }
  
  hideHighlightButton() {
    const highlightBtn = this.chatContainer.querySelector('#chatbot-highlight-btn');
    highlightBtn.style.display = 'none';
    this.selectedText = null;
  }
  
  toggleChatWindow() {
    const chatWindow = this.chatContainer.querySelector('#chatbot-window');
    const isCurrentlyOpen = chatWindow.classList.contains('open');
    
    if (isCurrentlyOpen) {
      this.closeChatWindow();
    } else {
      this.openChatWindow();
    }
  }
  
  openChatWindow() {
    const chatWindow = this.chatContainer.querySelector('#chatbot-window');
    chatWindow.classList.add('open');
    this.isOpen = true;
    
    // Focus the input when opening
    const chatInput = this.chatContainer.querySelector('#chatbot-input');
    setTimeout(() => chatInput.focus(), 300);
  }
  
  closeChatWindow() {
    const chatWindow = this.chatContainer.querySelector('#chatbot-window');
    chatWindow.classList.remove('open');
    this.isOpen = false;
    this.hideHighlightButton();
  }
  
  async sendMessage() {
    const input = this.chatContainer.querySelector('#chatbot-input');
    const message = input.value.trim();
    
    if (!message) return;
    
    // Add user message to UI
    this.addMessageToUI(message, 'user');
    input.value = '';
    
    try {
      // Call the API
      const response = await this.callAPI(message);
      
      // Add bot response to UI
      this.addMessageToUI(response.response, 'bot', response.sources);
      
      // Update session ID if provided
      if (response.session_id) {
        this.sessionId = response.session_id;
      }
    } catch (error) {
      console.error('Error sending message:', error);
      this.addMessageToUI('Sorry, I encountered an error. Please try again.', 'bot');
    }
  }
  
  async askAboutSelection() {
    if (!this.selectedText) return;

    const question = `Based on this text: "${this.selectedText}", ${this.chatContainer.querySelector('#chatbot-input').value || 'explain more about this content.'}`;

    // Add user message to UI
    this.addMessageToUI(`Ask about selection: ${this.selectedText.substring(0, 50)}...`, 'user');

    try {
      // Call the API with highlight_text parameter
      const response = await this.callAPI(
        this.chatContainer.querySelector('#chatbot-input').value || 'Explain this content.',
        this.selectedText,
        window.location.href  // Pass current page URL as document reference
      );

      // Add bot response to UI
      this.addMessageToUI(response.response, 'bot', response.sources);

      // Update session ID if provided
      if (response.session_id) {
        this.sessionId = response.session_id;
      }
    } catch (error) {
      console.error('Error asking about selection:', error);
      this.addMessageToUI('Sorry, I encountered an error. Please try again.', 'bot');
    }

    this.hideHighlightButton();
  }
  
  async callAPI(message, highlightText = null, documentUrl = null) {
    const endpoint = `${this.config.apiEndpoint}/chat`;

    const requestBody = {
      message: message,
      session_id: this.sessionId || null,
    };

    if (highlightText) {
      requestBody.highlight_text = highlightText;
    }

    // Include document URL if available (for storing highlights in DB)
    if (documentUrl) {
      requestBody.document_url = documentUrl;
    }

    const response = await fetch(endpoint, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-API-Key': this.config.apiKey
      },
      body: JSON.stringify(requestBody)
    });

    if (!response.ok) {
      throw new Error(`API request failed: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  }
  
  addMessageToUI(message, sender, sources = null) {
    const messagesContainer = this.chatContainer.querySelector('#chatbot-messages');
    
    const messageElement = document.createElement('div');
    messageElement.classList.add('message', `${sender}-message`);
    messageElement.textContent = message;
    
    // Add sources if provided
    if (sources && sources.length > 0) {
      const sourcesElement = document.createElement('div');
      sourcesElement.classList.add('sources');
      sourcesElement.innerHTML = '<strong>Sources:</strong>';
      
      sources.forEach((source, index) => {
        const sourceItem = document.createElement('div');
        sourceItem.classList.add('source-item');
        // Truncate source text for display
        const sourceText = source.text.length > 100 ? 
          source.text.substring(0, 100) + '...' : source.text;
        sourceItem.innerHTML = `<em>Excerpt:</em> ${sourceText}`;
        sourcesElement.appendChild(sourceItem);
      });
      
      messageElement.appendChild(sourcesElement);
    }
    
    messagesContainer.appendChild(messageElement);
    
    // Scroll to bottom
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }
}

// Initialize the widget when the document is ready
document.addEventListener('DOMContentLoaded', () => {
  // Configuration can be customized here
  const config = {
    apiEndpoint: window.RAG_CHATBOT_CONFIG?.apiEndpoint || 'http://localhost:8000',
    apiKey: window.RAG_CHATBOT_CONFIG?.apiKey || ''
  };
  
  new RAGChatbotWidget(config);
});
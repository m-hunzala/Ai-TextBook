/**
 * Book RAG Chatbot Frontend
 * Handles chat interface and selected text queries
 */

class ChatApp {
    constructor() {
        // Configuration - can be set via environment or passed from HTML
        this.API_BASE_URL = this.getApiBaseUrl();
        
        // DOM elements
        this.chatHistory = document.getElementById('chatHistory');
        this.userInput = document.getElementById('userInput');
        this.sendButton = document.getElementById('sendButton');
        this.askSelectedButton = document.getElementById('askSelectedButton');
        
        // State
        this.isLoading = false;
        
        // Initialize the app
        this.init();
    }
    
    /**
     * Get API base URL from environment or fallback
     */
    getApiBaseUrl() {
        // Try to get from environment variable or data attribute
        return window.BOOK_RAG_API_URL || 
               document.documentElement.getAttribute('data-api-url') || 
               '/api/v1';
    }
    
    /**
     * Initialize event listeners and app state
     */
    init() {
        // Event listeners
        this.sendButton.addEventListener('click', () => this.sendMessage());
        this.askSelectedButton.addEventListener('click', () => this.askAboutSelectedText());
        
        // Keyboard event for sending message with Enter key
        this.userInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                this.sendMessage();
            }
        });
        
        // Selection change listener for floating button
        document.addEventListener('selectionchange', () => {
            this.handleSelectionChange();
        });
        
        // Initial check for any existing selection
        this.handleSelectionChange();
    }
    
    /**
     * Handle text selection changes to show/hide the floating button
     */
    handleSelectionChange() {
        const selectedText = window.getSelection().toString().trim();
        
        if (selectedText) {
            this.askSelectedButton.classList.remove('hidden');
        } else {
            this.askSelectedButton.classList.add('hidden');
        }
    }
    
    /**
     * Add a message to the chat history
     */
    addMessageToHistory(text, isUser = false, isFromSelectedText = false) {
        const messageDiv = document.createElement('div');
        messageDiv.className = isUser ? 'message user-message' : 'message bot-message';
        
        const strong = document.createElement('strong');
        strong.textContent = isUser ? 'You:' : 'Bot:';
        
        const textSpan = document.createElement('span');
        textSpan.textContent = text;
        
        messageDiv.appendChild(strong);
        messageDiv.appendChild(document.createElement('br'));
        messageDiv.appendChild(textSpan);
        
        // Add badge for selected text responses
        if (isFromSelectedText) {
            const badge = document.createElement('div');
            badge.className = 'selected-text-badge';
            badge.textContent = 'Selected-text only';
            messageDiv.appendChild(badge);
        }
        
        this.chatHistory.appendChild(messageDiv);
        
        // Scroll to bottom
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
    }
    
    /**
     * Show loading indicator
     */
    showLoading() {
        const loadingDiv = document.createElement('div');
        loadingDiv.className = 'message bot-message';
        loadingDiv.id = 'loadingMessage';
        loadingDiv.innerHTML = '<strong>Bot:</strong> <div class="loading"></div> Thinking...';
        this.chatHistory.appendChild(loadingDiv);
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
    }
    
    /**
     * Hide loading indicator
     */
    hideLoading() {
        const loadingMessage = document.getElementById('loadingMessage');
        if (loadingMessage) {
            loadingMessage.remove();
        }
    }
    
    /**
     * Show error message
     */
    showError(error) {
        const errorDiv = document.createElement('div');
        errorDiv.className = 'error';
        errorDiv.textContent = `Error: ${error}`;
        this.chatHistory.appendChild(errorDiv);
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
    }
    
    /**
     * Send a regular message to the chat
     */
    async sendMessage() {
        const query = this.userInput.value.trim();
        if (!query || this.isLoading) return;
        
        // Add user message to history
        this.addMessageToHistory(query, true);
        this.userInput.value = '';
        
        // Show loading indicator
        this.showLoading();
        this.isLoading = true;
        
        try {
            const response = await fetch(`${this.API_BASE_URL}/query`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    book_id: 'default_book', // In a real app, this would be set appropriately
                    question: query,
                    top_k: 5
                })
            });
            
            if (!response.ok) {
                throw new Error(`Server error: ${response.status} - ${response.statusText}`);
            }
            
            const data = await response.json();
            
            // Hide loading indicator
            this.hideLoading();
            
            // Add bot response to history
            this.addMessageToHistory(data.answer);
        } catch (error) {
            // Hide loading indicator
            this.hideLoading();
            this.showError(error.message);
        } finally {
            this.isLoading = false;
        }
    }
    
    /**
     * Ask about selected text
     */
    async askAboutSelectedText() {
        const selectedText = window.getSelection().toString().trim();
        if (!selectedText) {
            this.showError('No text selected');
            return;
        }
        
        const question = this.userInput.value.trim();
        if (!question) {
            this.showError('Please enter a question');
            return;
        }
        
        // Add user message to history
        this.addMessageToHistory(`Question: ${question}\n(about selected text)`, true);
        
        // Show loading indicator
        this.showLoading();
        this.isLoading = true;
        
        try {
            const response = await fetch(`${this.API_BASE_URL}/query_selected`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    selected_text: selectedText,
                    question: question
                })
            });
            
            if (!response.ok) {
                throw new Error(`Server error: ${response.status} - ${response.statusText}`);
            }
            
            const data = await response.json();
            
            // Hide loading indicator
            this.hideLoading();
            
            // Add bot response to history with selected text indicator
            this.addMessageToHistory(data.answer, false, true);
        } catch (error) {
            // Hide loading indicator
            this.hideLoading();
            this.showError(error.message);
        } finally {
            this.isLoading = false;
        }
    }
}

// Initialize the app when the DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    new ChatApp();
});

// Example fetch code for API calls (can be used as reference)
/*
// Example for regular query
async function exampleQuery() {
    const response = await fetch(`${API_BASE_URL}/query`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            book_id: 'some_book_id',
            question: 'What is this book about?',
            top_k: 5
        })
    });
    
    const data = await response.json();
    console.log(data);
}

// Example for selected text query
async function exampleQuerySelected() {
    const response = await fetch(`${API_BASE_URL}/query_selected`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            selected_text: 'The selected text content...',
            question: 'What does this mean?'
        })
    });
    
    const data = await response.json();
    console.log(data);
}
*/
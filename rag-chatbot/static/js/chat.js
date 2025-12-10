class ChatApp {
    constructor() {
        this.chatHistory = document.getElementById('chatHistory');
        this.userInput = document.getElementById('userInput');
        this.sendButton = document.getElementById('sendButton');
        this.askSelectedButton = document.getElementById('askSelectedButton');
        this.bookTitle = document.getElementById('bookTitle');
        this.bookAuthor = document.getElementById('bookAuthor');
        this.bookContent = document.getElementById('bookContent');
        this.ingestButton = document.getElementById('ingestButton');
        
        this.initializeEventListeners();
    }
    
    initializeEventListeners() {
        this.sendButton.addEventListener('click', () => this.sendMessage());
        this.askSelectedButton.addEventListener('click', () => this.askAboutSelectedText());
        this.ingestButton.addEventListener('click', () => this.ingestBook());
        
        // Allow sending message with Enter key (but not Shift+Enter for new line)
        this.userInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                this.sendMessage();
            }
        });
    }
    
    addMessageToHistory(text, isUser = false) {
        const messageDiv = document.createElement('div');
        messageDiv.className = isUser ? 'message user-message' : 'message bot-message';
        
        const strong = document.createElement('strong');
        strong.textContent = isUser ? 'You:' : 'Bot:';
        
        const textSpan = document.createElement('span');
        textSpan.textContent = text;
        
        messageDiv.appendChild(strong);
        messageDiv.appendChild(document.createElement('br'));
        messageDiv.appendChild(textSpan);
        
        this.chatHistory.appendChild(messageDiv);
        
        // Scroll to bottom
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
    }
    
    showLoading() {
        const loadingDiv = document.createElement('div');
        loadingDiv.className = 'message bot-message';
        loadingDiv.id = 'loadingMessage';
        loadingDiv.innerHTML = '<strong>Bot:</strong> <div class="loading"></div> Thinking...';
        this.chatHistory.appendChild(loadingDiv);
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
    }
    
    hideLoading() {
        const loadingMessage = document.getElementById('loadingMessage');
        if (loadingMessage) {
            loadingMessage.remove();
        }
    }
    
    showError(error) {
        const errorDiv = document.createElement('div');
        errorDiv.className = 'error';
        errorDiv.textContent = `Error: ${error}`;
        this.chatHistory.appendChild(errorDiv);
        this.chatHistory.scrollTop = this.chatHistory.scrollHeight;
    }
    
    async sendMessage() {
        const query = this.userInput.value.trim();
        if (!query) return;
        
        // Add user message to history
        this.addMessageToHistory(query, true);
        this.userInput.value = '';
        
        // Show loading indicator
        this.showLoading();
        
        try {
            const response = await fetch('/api/v1/query', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    query: query,
                    top_k: 5
                })
            });
            
            if (!response.ok) {
                throw new Error(`Server error: ${response.status}`);
            }
            
            const data = await response.json();
            
            // Remove loading indicator
            this.hideLoading();
            
            // Add bot response to history
            this.addMessageToHistory(data.response);
        } catch (error) {
            // Remove loading indicator
            this.hideLoading();
            this.showError(error.message);
        }
    }
    
    async askAboutSelectedText() {
        const selectedText = window.getSelection().toString().trim();
        if (!selectedText) {
            this.showError('Please select text on the page first');
            return;
        }
        
        const query = this.userInput.value.trim();
        if (!query) {
            this.showError('Please enter a question');
            return;
        }
        
        // Add user message to history
        this.addMessageToHistory(`Query: ${query}\nAbout selected text: ${selectedText.substring(0, 100)}...`, true);
        this.userInput.value = '';
        
        // Show loading indicator
        this.showLoading();
        
        try {
            const response = await fetch('/api/v1/query_selected', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    query: query,
                    selected_text: selectedText
                })
            });
            
            if (!response.ok) {
                throw new Error(`Server error: ${response.status}`);
            }
            
            const data = await response.json();
            
            // Remove loading indicator
            this.hideLoading();
            
            // Add bot response to history
            this.addMessageToHistory(data.response);
        } catch (error) {
            // Remove loading indicator
            this.hideLoading();
            this.showError(error.message);
        }
    }
    
    async ingestBook() {
        const title = this.bookTitle.value.trim();
        const author = this.bookAuthor.value.trim();
        const content = this.bookContent.value.trim();
        
        if (!title || !author || !content) {
            this.showError('Please fill in all fields');
            return;
        }
        
        // Show loading indicator
        this.showLoading();
        
        try {
            const response = await fetch('/api/v1/ingest', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    title: title,
                    author: author,
                    text: content
                })
            });
            
            if (!response.ok) {
                throw new Error(`Server error: ${response.status}`);
            }
            
            const data = await response.json();
            
            // Remove loading indicator
            this.hideLoading();
            
            // Add success message to history
            this.addMessageToHistory(`Book "${title}" ingested successfully! Processed ${data.chunks_processed} chunks.`);
            
            // Clear the form
            this.bookTitle.value = '';
            this.bookAuthor.value = '';
            this.bookContent.value = '';
        } catch (error) {
            // Remove loading indicator
            this.hideLoading();
            this.showError(error.message);
        }
    }
}

// Initialize the app when the page loads
document.addEventListener('DOMContentLoaded', () => {
    new ChatApp();
});
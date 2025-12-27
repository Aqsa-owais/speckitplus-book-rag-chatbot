// RAG Chatbot Interface JavaScript
document.addEventListener('DOMContentLoaded', function() {
    const queryInput = document.getElementById('query-input');
    const sendButton = document.getElementById('send-button');
    const chatHistory = document.getElementById('chat-history');
    const statusText = document.getElementById('status-text');

    // Add event listeners
    sendButton.addEventListener('click', sendMessage);
    queryInput.addEventListener('keypress', function(e) {
        if (e.key === 'Enter') {
            sendMessage();
        }
    });

    function sendMessage() {
        const query = queryInput.value.trim();
        if (!query) {
            return;
        }

        // Add user message to chat
        addMessageToChat(query, 'user');

        // Clear input and disable button while processing
        queryInput.value = '';
        sendButton.disabled = true;
        updateStatus('Processing...');

        // Call the backend API
        fetch('http://localhost:8000/chat', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                query: query,
                top_k: 5
            })
        })
        .then(response => {
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
        })
        .then(data => {
            // Add bot response to chat
            addMessageToChat(data.formatted_response || data.response, 'bot');
            updateStatus('Response received');
        })
        .catch(error => {
            console.error('Error:', error);
            addMessageToChat('Error: ' + error.message, 'bot');
            updateStatus('Error occurred');
        })
        .finally(() => {
            sendButton.disabled = false;
        });
    }

    function addMessageToChat(message, sender) {
        const messageDiv = document.createElement('div');
        messageDiv.classList.add('message');
        messageDiv.classList.add(sender + '-message');

        // Format the message - replace newlines with <br> for display
        const formattedMessage = message.replace(/\n/g, '<br>');
        messageDiv.innerHTML = `<strong>${sender === 'user' ? 'You:' : 'Assistant:'}</strong> ${formattedMessage}`;

        chatHistory.appendChild(messageDiv);

        // Scroll to bottom
        chatHistory.scrollTop = chatHistory.scrollHeight;
    }

    function updateStatus(text) {
        statusText.textContent = text;
        setTimeout(() => {
            if (statusText.textContent === text) {
                statusText.textContent = 'Ready';
            }
        }, 3000);
    }

    // Initial status
    updateStatus('Ready to chat');
});
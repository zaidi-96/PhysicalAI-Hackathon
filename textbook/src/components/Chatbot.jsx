import React, { useState } from 'react';
import './Chatbot.css';

const Chatbot = () => {
  const [messages, setMessages] = useState([
    { text: "Hello! I'm your Physical AI assistant. Ask me anything about ROS 2, Gazebo, or robotics!", sender: 'bot' }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');

  // Get selected text from page
  React.useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection.toString().trim()) {
        setSelectedText(selection.toString().trim());
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    // Add user message
    const userMessage = { text: input, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          question: input,
          context: selectedText || null
        })
      });

      const data = await response.json();
      
      // Add bot response
      const botMessage = { text: data.answer, sender: 'bot' };
      setMessages(prev => [...prev, botMessage]);
      
      // Clear selected text after use
      if (selectedText) {
        setSelectedText('');
      }
      
    } catch (error) {
      console.error('Error:', error);
      const errorMessage = { text: "Sorry, I'm having trouble connecting. Please try again.", sender: 'bot' };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const clearChat = () => {
    setMessages([
      { text: "Hello! I'm your Physical AI assistant. Ask me anything about ROS 2, Gazebo, or robotics!", sender: 'bot' }
    ]);
  };

  return (
    <div className="chatbot-container">
      {/* Header */}
      <div className="chatbot-header">
        <h3>🤖 Physical AI Assistant</h3>
        <button onClick={clearChat} className="clear-btn">Clear</button>
      </div>

      {/* Selected Text Indicator */}
      {selectedText && (
        <div className="selected-text-indicator">
          <span>📑 Asking about selected text</span>
          <button onClick={() => setSelectedText('')} className="close-btn">×</button>
        </div>
      )}

      {/* Messages */}
      <div className="messages-container">
        {messages.map((msg, index) => (
          <div key={index} className={`message ${msg.sender}`}>
            <div className="message-content">
              {msg.text}
            </div>
          </div>
        ))}
        
        {loading && (
          <div className="message bot">
            <div className="typing-indicator">
              <span>●</span>
              <span>●</span>
              <span>●</span>
            </div>
          </div>
        )}
      </div>

      {/* Input */}
      <div className="input-container">
        <textarea
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder="Ask about ROS 2, Gazebo, or robotics..."
          onKeyPress={(e) => e.key === 'Enter' && !e.shiftKey && sendMessage()}
          rows="2"
          disabled={loading}
        />
        <button 
          onClick={sendMessage} 
          disabled={!input.trim() || loading}
          className="send-btn"
        >
          {loading ? 'Sending...' : 'Send'}
        </button>
      </div>

      {/* Hints */}
      <div className="hints">
        <small>Try: "How to install ROS 2?" or "What is Gazebo?"</small>
      </div>
    </div>
  );
};

export default Chatbot;
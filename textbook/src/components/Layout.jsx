import React, { useState } from 'react';
import Chatbot from './Chatbot';

const Layout = ({ children }) => {
  const [showChat, setShowChat] = useState(false);

  return (
    <div>
      {/* Navbar */}
      <nav style={{
        background: 'linear-gradient(to right, #006600, #01411C)',
        padding: '15px',
        color: 'white',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center'
      }}>
        <h2>🤖 Physical AI Textbook</h2>
        <button 
          onClick={() => setShowChat(!showChat)}
          style={{
            background: 'white',
            color: '#006600',
            border: 'none',
            padding: '8px 15px',
            borderRadius: '5px',
            cursor: 'pointer'
          }}
        >
          {showChat ? '✕ Close Chat' : '💬 Open AI Assistant'}
        </button>
      </nav>

      {/* Main Content */}
      <div style={{ display: 'flex', minHeight: 'calc(100vh - 70px)' }}>
        {/* Sidebar Area */}
        <div style={{ width: '250px', borderRight: '1px solid #ddd' }}>
          {/* Docusaurus sidebar automatically appears here */}
        </div>

        {/* Content Area */}
        <div style={{ flex: 1, padding: '20px', position: 'relative' }}>
          {children}
          
          {/* Chatbot Sidebar */}
          {showChat && (
            <div style={{
              position: 'fixed',
              right: '20px',
              bottom: '20px',
              width: '400px',
              height: '600px',
              zIndex: 1000,
              boxShadow: '0 0 20px rgba(0,0,0,0.3)'
            }}>
              <Chatbot />
            </div>
          )}
        </div>
      </div>

      {/* Footer */}
      <footer style={{
        background: '#f5f5f5',
        padding: '20px',
        textAlign: 'center',
        borderTop: '1px solid #ddd',
        marginTop: '20px'
      }}>
        <p>© 2024 Physical AI Textbook - Made for Pakistani Students</p>
      </footer>
    </div>
  );
};

export default Layout;
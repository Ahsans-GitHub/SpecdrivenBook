import React from 'react';
import Chatbot from './Chatbot/ChatbotClient';
import ChatbotClient from './Chatbot/ChatbotClient';

/**
 * Test component to verify the Chatbot component is properly integrated
 */
const TestChatbot: React.FC = () => {
  return (
    <div style={{ padding: '20px', maxWidth: '800px', margin: '0 auto' }}>
      <h1>Physical AI Textbook Chatbot</h1>
      <p>Testing the integrated chatbot component:</p>
      <div style={{ border: '1px solid #ccc', borderRadius: '8px', height: '500px' }}>
        <ChatbotClient/>
      </div>
    </div>
  );
};

export default TestChatbot;
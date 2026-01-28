import React, { JSX } from 'react';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot/ChatbotClient';
import ChatbotClient from '../components/Chatbot/ChatbotClient';

export default function TestChatbotPage(): JSX.Element {
  return (
    <Layout
      title="Test Chatbot"
      description="Test page for the Physical AI textbook RAG Chatbot">
      <main>
        <div style={{ padding: '2rem' }}>
          <div className="container">
            <h1>Physical AI Textbook RAG Chatbot Test</h1>
            <p>This page tests the integration between the frontend and backend systems.</p>

            <div style={{
              border: '1px solid #ddd',
              borderRadius: '8px',
              padding: '1rem',
              marginTop: '1rem',
              height: '600px',
              display: 'flex',
              flexDirection: 'column'
            }}>
              <ChatbotClient/>
            </div>

            <div style={{ marginTop: '1rem', paddingTop: '1rem', borderTop: '1px solid #eee' }}>
              <h3>Integration Information:</h3>
              <ul>
                <li>Backend API: <code>http://localhost:8000</code></li>
                <li>Frontend: Docusaurus with React components</li>
                <li>Communication: REST API with JSON payloads</li>
                <li>Features: Selected text context, multi-turn conversations, adaptive UI</li>
              </ul>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
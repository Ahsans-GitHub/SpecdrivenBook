import React, { useState, useEffect, JSX } from 'react';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot/ChatbotClient';
import ChatbotClient from '../components/Chatbot/ChatbotClient';

export default function TestIntegrationPage(): JSX.Element {
  const [backendStatus, setBackendStatus] = useState<string | null>(null);
  const [connectionTested, setConnectionTested] = useState<boolean>(false);

  // Test backend connection on page load
  useEffect(() => {
    const testBackend = async () => {
      try {
        const response = await fetch('http://localhost:8000/health');
        const data = await response.json();
        setBackendStatus(data.status);
      } catch (error) {
        console.error('Error testing backend connection:', error);
        setBackendStatus('unavailable');
      }
      setConnectionTested(true);
    };

    testBackend();
  }, []);

  return (
    <Layout
      title="Integration Test"
      description="Test page to verify backend-frontend integration">
      <main>
        <div style={{ padding: '2rem' }}>
          <div className="container">
            <h1>Physical AI Textbook RAG Chatbot Integration Test</h1>

            <div style={{ marginBottom: '2rem', padding: '1rem', backgroundColor: '#f5f5f5', borderRadius: '4px' }}>
              <h2>System Status</h2>
              <div style={{ display: 'flex', gap: '2rem' }}>
                <div>
                  <strong>Backend:</strong>
                  <span style={{
                    color: backendStatus === 'healthy' ? 'green' : 'red',
                    marginLeft: '0.5rem'
                  }}>
                    {connectionTested ? backendStatus : 'Testing...'}
                  </span>
                </div>
                <div>
                  <strong>Frontend:</strong>
                  <span style={{ color: 'green', marginLeft: '0.5rem' }}>Running</span>
                </div>
                <div>
                  <strong>Integration:</strong>
                  <span style={{
                    color: backendStatus === 'healthy' ? 'green' : 'orange',
                    marginLeft: '0.5rem'
                  }}>
                    {backendStatus === 'healthy' ? 'Ready' : 'Backend Unavailable'}
                  </span>
                </div>
              </div>
            </div>

            <div style={{
              border: '1px solid #ddd',
              borderRadius: '8px',
              padding: '1rem',
              marginTop: '1rem',
              height: '600px',
              display: 'flex',
              flexDirection: 'column'
            }}>
              <h2>Chat Interface</h2>
              <p>Try interacting with the chatbot below to test the integration:</p>

              <div style={{ flex: 1, marginTop: '1rem' }}>
                <ChatbotClient/>
              </div>
            </div>

            <div style={{ marginTop: '2rem', paddingTop: '1rem', borderTop: '1px solid #eee' }}>
              <h3>Integration Features:</h3>
              <ul>
                <li>✅ Backend-frontend communication via REST API</li>
                <li>✅ Selected text context capture and processing</li>
                <li>✅ Multi-turn conversation support</li>
                <li>✅ Adaptive UI elements with ChatKit SDK</li>
                <li>✅ Source citations with clickable links</li>
                <li>✅ Real-time feedback and typing indicators</li>
                <li>✅ Error handling and retry logic</li>
              </ul>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
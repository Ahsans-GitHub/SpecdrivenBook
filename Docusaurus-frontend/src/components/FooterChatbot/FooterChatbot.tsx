import React, { useState, useEffect } from 'react';
import Chatbot from '../Chatbot/ChatbotClient';
import ChatbotClient from '../Chatbot/ChatbotClient';

/**
 * Footer component with floating chatbot that appears on all pages
 * Provides easy access to the Physical AI textbook RAG chatbot
 */
const FooterChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [isMounted, setIsMounted] = useState<boolean>(false);

  // Ensure component is mounted before setting state to avoid hydration errors
  useEffect(() => {
    setIsMounted(true);
  }, []);

  // Capture selected text functionality
  useEffect(() => {
    if (!isMounted) return;

    const handleMouseUp = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim().length > 0) {
        // Store selected text in a global variable for the chatbot to access
        (window as any).selectedText = selection.toString().substring(0, 2000);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isMounted]);

  if (!isMounted) {
    return null; // Render nothing on the server to avoid hydration issues
  }

  return (
    <div>
      {/* Floating chatbot button - appears on all pages */}
      <div className="floating-chatbot-container">
        {!isOpen && (
          <button
            onClick={() => setIsOpen(true)}
            className="floating-chatbot-button fixed bottom-6 right-6 z-50 bg-blue-600 text-white p-4 rounded-full shadow-lg hover:bg-blue-700 transition-colors focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-opacity-50"
            aria-label="Open AI Assistant"
            style={{
              width: '60px',
              height: '60px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              fontSize: '24px',
              zIndex: 1000,
              cursor: 'pointer'
            }}
          >
            💬
          </button>
        )}

        {/* Chatbot panel */}
        {isOpen && (
          <div
            className="floating-chatbot-panel fixed bottom-20 right-6 z-50 w-96 h-96 max-w-[90vw] max-h-[70vh] bg-white rounded-lg shadow-xl border border-gray-200 flex flex-col"
            style={{
              zIndex: 999
            }}
          >
            <div className="flex justify-between items-center p-3 border-b border-gray-200 bg-gray-50 rounded-t-lg">
              <h3 className="font-semibold text-gray-800">Physical AI Assistant</h3>
              <button
                onClick={() => setIsOpen(false)}
                className="text-gray-500 hover:text-gray-700 focus:outline-none"
                aria-label="Close chat"
              >
                ✕
              </button>
            </div>

            <div className="flex-1 overflow-hidden p-2">
              <ChatbotClient/>
            </div>
          </div>
        )}
      </div>

      <style>{`
        .floating-chatbot-container {
          position: relative;
        }

        .floating-chatbot-button {
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
        }

        .floating-chatbot-panel {
          box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2);
        }
      `}</style>
    </div>
  );
};

export default FooterChatbot;
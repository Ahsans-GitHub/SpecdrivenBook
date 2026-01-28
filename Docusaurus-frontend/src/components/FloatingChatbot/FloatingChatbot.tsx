import React, { useState, useEffect } from 'react';
// import Chatbot from '../Chatbot/Chatbot';
import ChatbotClient from '../Chatbot/ChatbotClient';
import BrowserOnly from '@docusaurus/BrowserOnly';
import ErrorBoundary from '../ErrorBoundary';

/**
 * Floating chatbot component that appears on all pages
 * Provides easy access to the Physical AI textbook RAG chatbot
 */
const FloatingChatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [isMinimized, setIsMinimized] = useState<boolean>(false);
  const [sizeMode, setSizeMode] = useState<'normal' | 'bigger'>('normal'); // Track current size mode
  const [isMounted, setIsMounted] = useState<boolean>(false);

  // Ensure component is mounted before setting state to avoid hydration errors
  useEffect(() => {
    setIsMounted(true);
  }, []);

  // Handler for auto-expanding after response
  const handleAutoExpand = () => {
    setSizeMode('bigger'); // Expand to bigger size after response (like browsers do)
  };

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
    <div style={{ position: 'fixed', bottom: '24px', right: '24px', zIndex: 9999 }}>
      {/* Floating chatbot button - appears when closed */}
      {!isOpen && (
        <button
          onClick={() => {
            setIsOpen(true);
            setSizeMode('normal'); // Reset to normal size when opening
          }}
          className="flex items-center justify-center w-14 h-14 bg-blue-600 text-white rounded-full shadow-lg hover:bg-blue-700 transition-all duration-300 ease-in-out focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-opacity-50"
          aria-label="Open AI Assistant"
          style={{
            cursor: 'pointer',
            boxShadow: '0 4px 20px rgba(0, 0, 0, 0.2)',
            border: 'none',
            outline: 'none',
            transform: 'scale(1)',
          }}
          onMouseEnter={(e) => e.currentTarget.style.transform = 'scale(1.1)'}
          onMouseLeave={(e) => e.currentTarget.style.transform = 'scale(1)'}
        >
          <img
            src="/img/humanoid.png"
            alt="AI Assistant"
            style={{
              width: '32px',
              height: '32px',
              objectFit: 'contain',
              filter: 'invert(1)' // Make it white to contrast with blue background
            }}
          />
        </button>
      )}

      {/* Minimized chatbot button - appears when minimized */}
      {isOpen && isMinimized && (
        <button
          onClick={() => {
            setIsMinimized(false);
            setSizeMode('normal'); // Ensure it returns to normal size when restored from minimized
          }}
          className="flex items-center justify-center w-14 h-14 bg-blue-600 text-white rounded-full shadow-lg hover:bg-blue-700 transition-all duration-300 ease-in-out focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-opacity-50"
          aria-label="Restore AI Assistant"
          style={{
            cursor: 'pointer',
            boxShadow: '0 4px 20px rgba(0, 0, 0, 0.2)',
            border: 'none',
            outline: 'none',
            transform: 'scale(1)',
            position: 'absolute',
            bottom: '24px',
            right: '24px',
          }}
          onMouseEnter={(e) => e.currentTarget.style.transform = 'scale(1.1)'}
          onMouseLeave={(e) => e.currentTarget.style.transform = 'scale(1)'}
        >
          <img
            src="/img/humanoid.png"
            alt="AI Assistant"
            style={{
              width: '32px',
              height: '32px',
              objectFit: 'contain',
              filter: 'invert(1)' // Make it white to contrast with blue background
            }}
          />
        </button>
      )}

      {/* Chatbot panel - standard size with resize and minimize controls */}
      {isOpen && (
        <div
          className="bg-white rounded-lg shadow-xl border border-gray-200 flex flex-col overflow-auto"
          style={{
            width: sizeMode === 'bigger' ? '700px' : '1400px',
            height: sizeMode === 'bigger' ? '800px' : '200px',
            position: 'fixed',
            bottom: sizeMode === 'bigger' ? '24px' : '100px',
            right: sizeMode === 'bigger' ? '24px' : '50%',
            left: sizeMode === 'bigger' ? 'auto' : '50%',
            transform: sizeMode === 'bigger' ? 'none' : 'translateX(-50%)',
            display: isMinimized ? 'none' : 'flex', // Hide instead of not rendering
            flexDirection: 'column',
            minHeight: 0,
            maxHeight: 'calc(100vh - 48px)', // Ensure it fits within viewport with margin
            // overflow: 'auto' // Allow scrolling if needed
          }}
        >
          <div
            className="flex justify-between bg-gradient-to-r from-blue-500 to-blue-600 rounded-t-lg"
            style={{
              height: '40px', // Set to PNG height
              cursor: 'default', // Changed from move to default
              userSelect: 'none',
              paddingLeft: '12px',
              paddingRight: '12px',
              flexShrink: 0, // Prevent header from shrinking
              boxSizing: 'border-box'
            }}
          >
            <div className="flex justify-between" style={{ alignItems: 'center', display: 'flex' }}>
              <div className='flex justify-start'>
              <img
                src="/img/humanoid.png"
                alt="AI Assistant"
                style={{
                  width: '24px',
                  height: '24px',
                  objectFit: 'contain'
                }}
              />
            </div>
            <div className='flex justify-end items-end' style={{ alignItems: 'center', display: 'flex' }}>
              {/* Minimize button - Implements browser-style minimize functionality */}
              <button
                onClick={(e) => {
                  e.stopPropagation();
                  // Minimize the chat interface to the icon
                  setIsMinimized(true);
                }}
                className="text-white hover:text-gray-200 focus:outline-none flex items-center justify-center"
                aria-label="Minimize chat"
                style={{
                  background: 'rgba(255,255,255,0.2)',
                  border: '1px solid rgba(255,255,255,0.4)',
                  borderRadius: '4px',
                  width: '24px',
                  height: '24px',
                  fontSize: '12px',
                  fontWeight: 'bold',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  minWidth: '24px',
                  lineHeight: '1'
                }}
              >
                −
              </button>
              {/* Restore Down button - Toggles between normal and bigger size */}
              <button
                onClick={(e) => {
                  e.stopPropagation();
                  if (sizeMode === 'normal') {
                    // If currently normal, expand to bigger size (like after response)
                    setSizeMode('bigger');
                  } else {
                    // If currently bigger or fullscreen, return to normal size
                    setSizeMode('normal');
                  }
                }}
                className="text-white hover:text-gray-200 focus:outline-none flex items-center justify-center"
                aria-label={sizeMode === 'normal' ? "Expand to bigger size" : "Return to normal size"}
                style={{
                  background: 'rgba(255,255,255,0.2)',
                  border: '1px solid rgba(255,255,255,0.4)',
                  borderRadius: '4px',
                  width: '24px',
                  height: '24px',
                  fontSize: '12px',
                  fontWeight: 'bold',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  minWidth: '24px',
                  lineHeight: '1'
                }}
              >
                {sizeMode === 'normal' ? '□' : '❐'}
              </button>
              {/* Close button (X) */}
              <button
                onClick={(e) => {
                  e.stopPropagation();
                  setIsOpen(false);
                }}
                className="text-white hover:text-gray-200 focus:outline-none flex items-center justify-center"
                aria-label="Close chat"
                style={{
                  background: 'rgba(255,255,255,0.2)',
                  border: '1px solid rgba(255,255,255,0.4)',
                  borderRadius: '4px',
                  width: '24px',
                  height: '24px',
                  fontSize: '12px',
                  fontWeight: 'bold',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  minWidth: '24px',
                  lineHeight: '1'
                }}
              >
                ✕
              </button>
              </div>

            </div>
          </div>

          <div className={`${isMinimized ? 'hidden' : 'flex-1'} overflow-auto p-0`} style={{ display: isMinimized ? 'none' : 'flex', minHeight: 0, flex: '1 1 auto', height: '100%' }}>
            <ErrorBoundary>
              <ChatbotClient onAutoExpand={handleAutoExpand} />
            </ErrorBoundary>
          </div>
        </div>
      )}
    </div>
  );
};

export default FloatingChatbot;
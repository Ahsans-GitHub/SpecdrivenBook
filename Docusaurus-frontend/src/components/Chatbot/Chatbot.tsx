/**
 * Enhanced Chatbot Component for Physical AI Textbook
 * This component provides a chat interface that connects to the RAG backend
 * with support for selected text context, multi-turn conversations, and adaptive UI.
 */

import React, { useState, useRef, useEffect, useCallback } from 'react';
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message,
  MessageInput,
  TypingIndicator
} from '@chatscope/chat-ui-kit-react';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
import apiService from './apiService';
import historyService from './historyService';

// Define TypeScript interfaces
interface RetrievedChunk {
  id: string;
  title: string;
  url: string;
  content: string;
  section: string;
  tags: string[];
  score: number;
  similarity: number;
}

interface ChatMetadata {
  adaptive_prompt_hint?: string;
  visualization_suggestion?: string[];
  confidence_score?: number;
  retrieval_success?: boolean;
  adaptive_prompts?: string[];
  ui_enhancement_metadata?: {
    has_visualization_opportunities?: boolean;
    suggest_follow_up_questions?: boolean;
    suggest_related_topics?: boolean;
    suggest_content_format?: string;
  };
}

interface ChatMessage {
  id: number;
  message: string;
  sender: 'user' | 'assistant' | 'system';
  timestamp: Date;
  selectedText?: string | null;
  sources?: RetrievedChunk[];
  metadata?: ChatMetadata;
}

interface ApiResponse {
  success: boolean;
  data?: {
    response: string;
    sources: RetrievedChunk[];
    session_id?: string;
    metadata: ChatMetadata;
    retrieval_time: number;
    generation_time: number;
  };
  error?: string;
  status?: number;
}

/**
 * Chatbot component for the Physical AI textbook
 * Integrates with the backend RAG system to provide contextual responses
 */
const Chatbot: React.FC<{ title?: string; onAutoExpand?: () => void }> = ({ title = "Physical AI Assistant", onAutoExpand }) => {
  const [messages, setMessages] = useState<ChatMessage[]>([
    {
      id: 1,
      message: "Hello! I'm your Physical AI and Robotics assistant. How can I help you today?",
      sender: 'assistant',
      timestamp: new Date(),
      sources: []
    }
  ]);
  const [isTyping, setIsTyping] = useState<boolean>(false);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [connectionStatus, setConnectionStatus] = useState<'checking' | 'connected' | 'disconnected'>('checking');
  const [suggestedPrompts, setSuggestedPrompts] = useState<string[]>([]);
  const selectedTextRef = useRef<string>('');
  const [showSuggestedPrompts, setShowSuggestedPrompts] = useState<boolean>(true);

  // Check backend connection on component mount
  useEffect(() => {
    const checkConnection = async () => {
      try {
        // Add a small delay to ensure the API service is properly initialized
        await new Promise(resolve => setTimeout(resolve, 100));

        const isConnected = await apiService.testConnection();
        setConnectionStatus(isConnected ? 'connected' : 'disconnected');
      } catch (error) {
        console.error('Connection check failed:', error);
        setConnectionStatus('disconnected');
      }
    };

    // Wrap in a try-catch to prevent the entire component from crashing
    checkConnection().catch(error => {
      console.error('Unexpected error during connection check:', error);
      setConnectionStatus('disconnected');
    });
  }, []);

  // Function to capture selected text
  const captureSelectedText = () => {
    const selection = window.getSelection();
    if (selection && selection.toString().trim().length > 0) {
      // Limit to 2000 characters to prevent oversized payloads
      selectedTextRef.current = selection.toString().substring(0, 2000);
      console.log('Captured selected text:', selectedTextRef.current);
    }
  };

  // Handle sending a message
  const handleSend = async (message: string) => {
    const trimmedMessage = message.trim();

    if (!trimmedMessage) return;

    // Add user message to UI immediately for responsive feedback
    const userMessage: ChatMessage = {
      id: Date.now(),
      message: trimmedMessage,
      sender: 'user',
      timestamp: new Date(),
      selectedText: selectedTextRef.current || null
    };

    setMessages(prev => [...prev, userMessage]);
    setIsTyping(true);

    try {
      console.log('Sending message to backend:', trimmedMessage);

      // Send to backend with retry logic (3 attempts, exponential backoff)
      const response: ApiResponse = await apiService.sendMessage({
        query: trimmedMessage,
        selectedText: selectedTextRef.current || '',
        sessionId: sessionId || null
      }, 3, 1000); // 3 retries with 1000ms base delay

      console.log('Response received from backend:', response);

      if (response.success && response.data) {
        // Update session ID if this is the first message in the session
        if (!sessionId && response.data.session_id) {
          setSessionId(response.data.session_id);
        }

        // Add bot response to UI with all associated metadata
        const botMessage: ChatMessage = {
          id: Date.now() + 1,
          message: response.data.response,
          sender: 'assistant',
          timestamp: new Date(),
          sources: response.data.sources || [], // Include retrieved sources
          metadata: response.data.metadata || {} // Include response metadata
        };

        setMessages(prev => [...prev, botMessage]);

        // Save to both local history and Neon database
        historyService.saveToHistory(trimmedMessage, response.data.response, response.data.sources);

        // Also save to Neon database
        historyService.saveToNeonDatabase(trimmedMessage, response.data.response, response.data.sources, sessionId || undefined);

        // Extract and display adaptive prompts from response metadata
        if (response.data.metadata?.adaptive_prompts) {
          setSuggestedPrompts(response.data.metadata.adaptive_prompts);
          setShowSuggestedPrompts(true);
        } else {
          setSuggestedPrompts([]);
        }

        // Auto-expand to full screen after response like browsers do
        if (onAutoExpand) {
          onAutoExpand(); // Call parent component to handle expansion
        }

        // Clear selected text after successfully sending the message
        selectedTextRef.current = '';
      } else {
        console.error('Backend returned unsuccessful response:', response);
        throw new Error(response.error || 'Failed to get response');
      }
    } catch (error) {
      console.error('Error sending message:', error);

      // Check if error has more specific information
      let errorMessageText = 'Sorry, I encountered an error. Please try again.';
      if (error instanceof Error) {
        errorMessageText = `Error: ${error.message}`;
      } else if (typeof error === 'string') {
        errorMessageText = error;
      }

      // Display user-friendly error message in the chat
      const errorMessage: ChatMessage = {
        id: Date.now() + 1,
        message: errorMessageText,
        sender: 'system',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
      setSuggestedPrompts([]);
    } finally {
      // Always hide the typing indicator when the request completes
      setIsTyping(false);
    }
  };

  // Handle clicking on a suggested prompt
  const handleSuggestedPromptClick = (prompt: string) => {
    // Add the suggested prompt as a user message
    const userMessage: ChatMessage = {
      id: Date.now(),
      message: prompt,
      sender: 'user',
      timestamp: new Date(),
      selectedText: null
    };

    setMessages(prev => [...prev, userMessage]);

    // Simulate sending the message
    handleSend(prompt);

    // Hide suggested prompts after selection
    setShowSuggestedPrompts(false);
  };

  // Add event listener for text selection
  useEffect(() => {
    const handleMouseUp = () => {
      captureSelectedText();
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  return (
    <div
      className="flex flex-col h-full border border-gray-300 rounded-lg overflow-hidden shadow-md bg-white"
      style={{
        height: '100%',
        maxHeight: 'calc(100% - 40px)',
        minHeight: '200px',
        flex: '1 1 auto',
        display: 'flex'
      }} // Full height minus header height with minimum height and proper flex properties
      data-testid="chat-container"
    >

      <div className="flex-1 overflow-auto" style={{ minHeight: 0 }}>
        <MainContainer style={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
          <ChatContainer style={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
            <MessageList
              scrollBehavior="smooth"
              style={{ overflowY: 'auto', height: '100%', flex: 1 }}
              typingIndicator={isTyping ? <TypingIndicator content="AI is thinking..." /> : null}
            >
              {messages.map((msg) => (
                <Message
                  key={msg.id}
                  model={{
                    message: msg.message,
                    sender: msg.sender,
                    direction: msg.sender === 'user' ? 'outgoing' : 'incoming',
                    position: 'normal'
                  }}
                >
                  {msg.sources && msg.sources.length > 0 && (
                    <Message.Footer>
                      <div className="mt-2 pt-2 border-t border-gray-200 text-xs">
                        <strong>Sources:</strong>
                        <ul className="ml-4 mt-1">
                          {msg.sources.slice(0, 3).map((source, index) => (
                            <li key={index} className="mb-1">
                              <a
                                href={source.url}
                                target="_blank"
                                rel="noopener noreferrer"
                                className="text-blue-600 hover:underline"
                              >
                                {source.title || source.url}
                              </a>
                            </li>
                          ))}
                        </ul>

                        {/* Visualization of metadata */}
                        {msg.metadata && (
                          <div className="mt-2 pt-2 border-t border-gray-100">
                            {msg.metadata.confidence_score !== undefined && (
                              <div className="mb-1">
                                <span className="text-gray-600">Confidence:</span>
                                <div className="w-full bg-gray-200 rounded-full h-1.5 mt-1">
                                  <div
                                    className="bg-blue-600 h-1.5 rounded-full"
                                    style={{ width: `${(msg.metadata.confidence_score || 0) * 100}%` }}
                                  ></div>
                                </div>
                                <span className="text-xs text-gray-500">{(msg.metadata.confidence_score || 0).toFixed(2)}</span>
                              </div>
                            )}

                            {msg.metadata.ui_enhancement_metadata?.suggest_content_format && (
                              <div className="text-xs text-gray-600">
                                Format: {msg.metadata.ui_enhancement_metadata.suggest_content_format}
                              </div>
                            )}

                            {msg.metadata.visualization_suggestion && msg.metadata.visualization_suggestion.length > 0 && (
                              <div className="text-xs text-gray-600 mt-1">
                                Visualizations: {msg.metadata.visualization_suggestion.join(', ')}
                              </div>
                            )}
                          </div>
                        )}
                      </div>
                    </Message.Footer>
                  )}
                </Message>
              ))}
            </MessageList>

            {/* Adaptive prompts section */}
            {suggestedPrompts.length > 0 && showSuggestedPrompts && (
              <div className="p-3 bg-blue-50 border-t border-gray-200">
                <p className="text-xs text-gray-600 mb-2">Suggested prompts:</p>
                <div className="flex flex-wrap gap-2">
                  {suggestedPrompts.map((prompt, index) => (
                    <button
                      key={index}
                      onClick={() => handleSuggestedPromptClick(prompt)}
                      className="text-xs bg-white border border-blue-200 rounded-full px-3 py-1 hover:bg-blue-100 transition-colors"
                    >
                      {prompt.length > 40 ? `${prompt.substring(0, 40)}...` : prompt}
                    </button>
                  ))}
                </div>
              </div>
            )}

            <MessageInput
              placeholder="Type your message here..."
              onSend={handleSend}
              attachButton={false}
            />
          </ChatContainer>
        </MainContainer>
      </div>

      {selectedTextRef.current && (
        <div className="p-2 bg-gray-100 text-xs text-gray-600 border-t border-gray-200">
          Selected text context: "{selectedTextRef.current.substring(0, 100)}{selectedTextRef.current.length > 100 ? '...' : ''}"
        </div>
      )}
    </div>
  );
};

export default Chatbot;
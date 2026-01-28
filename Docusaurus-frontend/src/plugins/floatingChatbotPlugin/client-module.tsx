import React, { useState, useEffect } from 'react';
import ErrorBoundary from '../../components/ErrorBoundary';

// FloatingChatbot component that initializes safely
function SafeFloatingChatbot() {
  const [mounted, setMounted] = useState(false);

  // Only initialize the chatbot on the client side after mount
  useEffect(() => {
    setMounted(true);
  }, []);

  // Render nothing on the server or before mounting to avoid blocking
  if (!mounted) {
    return null;
  }

  // Dynamically import and render the floating chatbot with error boundary
  const FloatingChatbot = require('../../components/FloatingChatbot/FloatingChatbot').default;
  return (
    <ErrorBoundary>
      <FloatingChatbot />
    </ErrorBoundary>
  );
}

export default SafeFloatingChatbot;
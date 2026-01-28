import React from 'react';
import FloatingChatbot from '../components/FloatingChatbot/FloatingChatbot';

// Root component that wraps the entire application
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <div style={{ position: 'relative' }}>
      {children}
      <FloatingChatbot />
    </div>
  );
}
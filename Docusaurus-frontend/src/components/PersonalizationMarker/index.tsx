import React from 'react';

interface PersonalizationMarkerProps {
  userBackground: 'beginner' | 'basics' | 'normal' | 'pro' | 'advanced' | 'researcher' | string;
  children: React.ReactNode;
}

const PersonalizationMarker: React.FC<PersonalizationMarkerProps> = ({ userBackground, children }) => {
  // In a real application, this component would conditionally render or adapt content
  // based on the userBackground prop, possibly fetching specific content variants.
  // For now, it simply wraps content and could display a small label.

  const renderLabel = () => {
    if (userBackground && userBackground !== 'default') {
      return (
        <span
          style={{
            fontSize: '0.8em',
            backgroundColor: 'var(--ifm-color-primary-lighter)',
            color: 'var(--ifm-color-primary-darkest)',
            padding: '2px 6px',
            borderRadius: '4px',
            marginLeft: '8px',
            verticalAlign: 'middle',
            whiteSpace: 'nowrap',
          }}
        >
          For {userBackground}s
        </span>
      );
    }
    return null;
  };

  return (
    <div style={{
        borderLeft: '4px solid var(--ifm-color-primary)',
        paddingLeft: '10px',
        marginBottom: '1rem',
        marginTop: '1rem',
        backgroundColor: 'var(--ifm-background-color)', // Adjust as needed
        borderRadius: '5px'
      }}>
      <p>
        **Personalized Content** {renderLabel()}
      </p>
      {children}
    </div>
  );
};

export default PersonalizationMarker;

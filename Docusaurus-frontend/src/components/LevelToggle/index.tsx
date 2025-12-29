import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './styles.module.css';

// Type definitions for skill levels
export type SkillLevel = 'beginner' | 'normal' | 'pro' | 'advanced' | 'research';

const LEVELS: SkillLevel[] = ['beginner', 'normal', 'pro', 'advanced', 'research'];

export const LevelToggle: React.FC = () => {
  return (
    <BrowserOnly fallback={<div className={styles.container}>Loading...</div>}>
      {() => {
        const [level, setLevel] = useState<SkillLevel>('normal');

        useEffect(() => {
          const stored = localStorage.getItem('user_skill_level') as SkillLevel;
          if (stored && LEVELS.includes(stored)) {
            setLevel(stored);
          }
        }, []);

        const handleLevelChange = (newLevel: SkillLevel) => {
          setLevel(newLevel);
          localStorage.setItem('user_skill_level', newLevel);
          // Dispatch event for other components to listen to
          window.dispatchEvent(new Event('levelChange'));
        };

        return (
          <div className={styles.container}>
            <span className={styles.label}>Skill Level:</span>
            <div className={styles.toggleGroup}>
              {LEVELS.map((l) => (
                <button
                  key={l}
                  className={`${styles.button} ${level === l ? styles.active : ''}`}
                  onClick={() => handleLevelChange(l)}
                >
                  {l.charAt(0).toUpperCase() + l.slice(1)}
                </button>
              ))}
            </div>
          </div>
        );
      }}
    </BrowserOnly>
  );
};

export default LevelToggle;

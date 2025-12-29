import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './styles.module.css';
import { getQuizData, QuizData, Question } from './data';

// Define the lesson paths for each chapter to fetch context
const CHAPTER_LESSONS: Record<string, string[]> = {
  chapter2: [
    '/docs/chapter2/module1/lesson1',
    '/docs/chapter2/module1/lesson2',
    '/docs/chapter2/module1/lesson3',
    '/docs/chapter2/module1/lesson4'
  ],
  chapter3: [
    '/docs/chapter3/module2/lesson1',
    '/docs/chapter3/module2/lesson2',
    '/docs/chapter3/module2/lesson3',
    '/docs/chapter3/module2/lesson4'
  ],
  chapter4: [
    '/docs/chapter4/module3/lesson1',
    '/docs/chapter4/module3/lesson2',
    '/docs/chapter4/module3/lesson3',
    '/docs/chapter4/module3/lesson4'
  ],
  chapter5: [
    '/docs/chapter5/module4/lesson1',
    '/docs/chapter5/module4/lesson2',
    '/docs/chapter5/module4/lesson3',
    '/docs/chapter5/module4/lesson4'
  ],
  chapter7: [
    '/docs/chapter7/assessment1',
    '/docs/chapter7/assessment2',
    '/docs/chapter7/assessment3',
    '/docs/chapter7/assessment4'
  ]
};

interface QuizProps {
  chapterId: string;
}

// SECURITY WARNING: In a production environment, never expose API keys in frontend code.
// This is for demonstration/hackathon purposes only. A proxy backend is recommended.
const API_KEYS = [
  process.env.OPENAI_API_KEY_1,
  process.env.OPENAI_API_KEY_2,
  process.env.OPENAI_API_KEY_3
];

export const Quiz: React.FC<QuizProps> = ({ chapterId }) => {
  return (
    <BrowserOnly fallback={<div>Loading Quiz...</div>}>
      {() => {
        const [quizData, setQuizData] = useState<QuizData | null>(null);
        const [currentQuestionIndex, setCurrentQuestionIndex] = useState(0);
        const [score, setScore] = useState(0);
        const [showFeedback, setShowFeedback] = useState(false);
        const [isCorrect, setIsCorrect] = useState(false);
        const [isFinished, setIsFinished] = useState(false);
        const [selectedOption, setSelectedOption] = useState<number | null>(null);
        const [loadingStatus, setLoadingStatus] = useState<string>("Initializing...");

        useEffect(() => {
          const loadDynamicQuiz = async () => {
            let context = "";
            
            // 1. Try to fetch lesson content for context
            try {
              setLoadingStatus("Gathering chapter context...");
              const lessonPaths = CHAPTER_LESSONS[chapterId] || [];
              const contents = await Promise.all(
                lessonPaths.map(async (path) => {
                  try {
                    // Fetch the raw markdown
                    const res = await fetch(`${path}.md`);
                    if (res.ok) return await res.text();
                    return "";
                  } catch { return ""; }
                })
              );
              context = contents.join("\n\n").slice(0, 6000); // Limit context size
            } catch (e) {
              console.warn("Could not fetch raw lesson files, using chapterId for generation.");
            }

            // 2. Try OpenAI Fallback Chain
            for (let i = 0; i < API_KEYS.length; i++) {
              const key = API_KEYS[i];
              if (!key) continue;

              try {
                setLoadingStatus(`Generating fresh questions (Attempt ${i + 1}/3)...`);
                const response = await fetch('https://api.openai.com/v1/chat/completions', {
                  method: 'POST',
                  headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${key}`
                  },
                  body: JSON.stringify({
                    model: 'gpt-4o-mini',
                    messages: [
                      {
                        role: 'system',
                        content: 'You are a robotics expert. Generate a JSON quiz based on the provided text. Return EXACTLY this format: { "questions": [ { "id": "string", "text": "string", "options": ["string", "string", "string", "string"], "correctIndex": number, "feedback": { "correct": "string", "incorrect": "string" } } ] }'
                      },
                      {
                        role: 'user',
                        content: `Generate 15 dynamic multiple-choice questions for ${chapterId}. Context: ${context || "Focus on " + chapterId + " fundamentals in humanoid robotics and physical AI."}`
                      }
                    ],
                    response_format: { type: 'json_object' }
                  })
                });

                if (response.ok) {
                  const result = await response.json();
                  if (result.questions && result.questions.length > 0) {
                    setQuizData({ chapterId, questions: result.questions });
                    return;
                  }
                }
                
                if (response.status === 429) {
                  console.warn(`Key ${i+1} rate limited, switching...`);
                  continue;
                }
              } catch (e) {
                console.error(`Error with Key ${i+1}:`, e);
              }
            }

            // 3. Final Fallback: Static Data
            setLoadingStatus("Using offline backup questions...");
            const staticData = getQuizData(chapterId);
            setQuizData(staticData);
          };

          loadDynamicQuiz();
        }, [chapterId]);

        const handleOptionSelect = (index: number) => {
          if (showFeedback || !quizData) return; 
          
          setSelectedOption(index);
          const currentQuestion = quizData.questions[currentQuestionIndex];
          const correct = index === currentQuestion.correctIndex;
          
          setIsCorrect(correct);
          if (correct) setScore(s => s + 1);
          setShowFeedback(true);
        };

        const handleNext = () => {
          if (!quizData) return; 
          
          if (currentQuestionIndex < quizData.questions.length - 1) {
            setCurrentQuestionIndex(i => i + 1);
            setShowFeedback(false);
            setSelectedOption(null);
          } else {
            setIsFinished(true);
            const scores = JSON.parse(localStorage.getItem('quiz_scores') || '{}');
            scores[chapterId] = { 
              score: score,
              total: quizData.questions.length,
              date: new Date().toISOString() 
            };
            localStorage.setItem('quiz_scores', JSON.stringify(scores));
          }
        };

        if (!quizData) return (
          <div className={styles.loadingContainer}>
            <div className={styles.spinner}></div>
            <div className={styles.loadingText}>{loadingStatus}</div>
          </div>
        );

        if (quizData.questions.length === 0) return <div className={styles.error}>Quiz unavailable — review chapter manually</div>;

        if (isFinished) {
          const percentage = Math.round((score / quizData.questions.length) * 100);
          let message = "";
          if (percentage >= 85) message = "Strong work! You're ready for the next module.";
          else if (percentage >= 60) message = "Good effort. Review the core concepts to solidify your knowledge.";
          else message = "Review recommended. Re-read the chapter to master these concepts.";

          return (
            <div className={styles.container}>
              <h3>Quiz Complete!</h3>
              <div className={styles.score}>
                Score: {score} / {quizData.questions.length} ({percentage}%)
              </div>
              <p className={styles.message}>{message}</p>
              <button 
                className={styles.button}
                onClick={() => window.location.reload()}
              >
                Retry with Fresh Questions
              </button>
            </div>
          );
        }

        const question = quizData.questions[currentQuestionIndex];

        return (
          <div className={styles.container}>
            <div className={styles.header}>
              <span>Question {currentQuestionIndex + 1} of {quizData.questions.length}</span>
              <span>Score: {score}</span>
            </div>
            
            <div className={styles.question}>{question.text}</div>
            
            <div className={styles.options}>
              {question.options.map((option, idx) => (
                <button
                  key={idx}
                  className={`
                    ${styles.option} 
                    ${selectedOption === idx ? styles.selected : ''}
                    ${showFeedback && idx === question.correctIndex ? styles.correct : ''}
                    ${showFeedback && selectedOption === idx && idx !== question.correctIndex ? styles.incorrect : ''}
                  `}
                  onClick={() => handleOptionSelect(idx)}
                  disabled={showFeedback}
                >
                  {option}
                </button>
              ))}
            </div>

            {showFeedback && (
              <div className={`${styles.feedback} ${isCorrect ? styles.feedbackCorrect : styles.feedbackIncorrect}`}>
                <strong>{isCorrect ? "Correct!" : "Incorrect"}</strong>
                <p>{isCorrect ? question.feedback.correct : question.feedback.incorrect}</p>
                <button className={styles.button} onClick={handleNext}>
                  {currentQuestionIndex < quizData.questions.length - 1 ? "Next Question" : "Finish Quiz"}
                </button>
              </div>
            )}
          </div>
        );
      }}
    </BrowserOnly>
  );
};

export default Quiz;
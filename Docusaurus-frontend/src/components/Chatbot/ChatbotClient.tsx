import BrowserOnly from '@docusaurus/BrowserOnly';

export default function ChatbotClient({ onAutoExpand }: { onAutoExpand?: () => void }) {
  return (
    <BrowserOnly>
      {() => {
        const Chatbot = require('../Chatbot/Chatbot').default;
        return <Chatbot onAutoExpand={onAutoExpand} />;
      }}
    </BrowserOnly>
  );
}

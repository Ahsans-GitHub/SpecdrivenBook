/**
 * History Service for Chatbot Component
 * Handles storing and retrieving chat history using localStorage and potentially a backend
 */

interface ChatHistoryItem {
  id: string;
  query: string;
  response: string;
  timestamp: Date;
  sources?: any[];
}

class HistoryService {
  private readonly STORAGE_KEY = 'chatbot_history';
  private readonly MAX_HISTORY_ITEMS = 50; // Maximum number of history items to store

  /**
   * Save a chat interaction to history
   * @param query - The user's query
   * @param response - The AI's response
   * @param sources - Optional sources used in the response
   */
  saveToHistory(query: string, response: string, sources?: any[]): void {
    try {
      const history = this.getHistory();

      const newItem: ChatHistoryItem = {
        id: Date.now().toString(), // Simple ID generation
        query,
        response,
        timestamp: new Date(),
        sources
      };

      // Add new item to the beginning of the array
      history.unshift(newItem);

      // Keep only the most recent items
      if (history.length > this.MAX_HISTORY_ITEMS) {
        history.splice(this.MAX_HISTORY_ITEMS);
      }

      // Save back to localStorage
      localStorage.setItem(this.STORAGE_KEY, JSON.stringify(history));
    } catch (error) {
      console.error('Failed to save chat history:', error);
    }
  }

  /**
   * Retrieve chat history
   * @returns Array of chat history items, most recent first
   */
  getHistory(): ChatHistoryItem[] {
    try {
      const stored = localStorage.getItem(this.STORAGE_KEY);
      if (!stored) {
        return [];
      }

      const parsed = JSON.parse(stored);

      // Ensure all items have the correct structure and convert timestamp strings back to Dates
      return parsed.map((item: any) => ({
        ...item,
        timestamp: new Date(item.timestamp)
      }));
    } catch (error) {
      console.error('Failed to retrieve chat history:', error);
      return [];
    }
  }

  /**
   * Clear all chat history
   */
  clearHistory(): void {
    try {
      localStorage.removeItem(this.STORAGE_KEY);
    } catch (error) {
      console.error('Failed to clear chat history:', error);
    }
  }

  /**
   * Get recent queries for autocomplete/suggestions
   * @param limit - Maximum number of queries to return
   * @returns Array of recent queries
   */
  getRecentQueries(limit: number = 5): string[] {
    const history = this.getHistory();
    return history
      .slice(0, limit)
      .map(item => item.query);
  }

  /**
   * Async method to sync history with Neon database backend
   */
  async syncWithBackend(userId?: string): Promise<void> {
    try {
      const history = this.getHistory();

      // In a real implementation, we would send this to the backend
      // which would then store it in the Neon database
      // The API key would be securely handled on the backend
      const response = await fetch('https://ahsan350-rag.hf.space/history', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          userId: userId || 'anonymous',
          history
        })
      });

      if (!response.ok) {
        throw new Error(`Sync failed with status: ${response.status}`);
      }

      console.log('History synced successfully with backend');
    } catch (error) {
      console.error('History sync failed:', error);
      // Fallback to localStorage if backend sync fails
    }
  }

  /**
   * Method to save chat interaction to Neon database via backend
   */
  async saveToNeonDatabase(query: string, response: string, sources?: any[], userId?: string): Promise<void> {
    try {
      const chatEntry = {
        id: Date.now().toString(),
        query,
        response,
        timestamp: new Date().toISOString(),
        sources: sources || [],
        userId: userId || 'anonymous'
      };

      const fetchResponse = await fetch('https://ahsan350-rag.hf.space/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(chatEntry)
      });

      if (!fetchResponse.ok) {
        throw new Error(`Save to Neon DB failed with status: ${fetchResponse.status}`);
      }

      console.log('Chat entry saved to Neon database successfully');
    } catch (error) {
      console.error('Failed to save to Neon database:', error);
      // Fallback to saving in localStorage
      this.saveToHistory(query, response, sources);
    }
  }

  /**
   * Method to retrieve chat history from Neon database
   */
  async getFromNeonDatabase(userId?: string): Promise<any[]> {
    try {
      const response = await fetch('https://ahsan350-rag.hf.space/chat', {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        }
      });

      if (!response.ok) {
        throw new Error(`Retrieve from Neon DB failed with status: ${response.status}`);
      }

      const data = await response.json();
      console.log('History retrieved from Neon database successfully');
      return data.history || [];
    } catch (error) {
      console.error('Failed to retrieve from Neon database:', error);
      // Fallback to localStorage
      return this.getHistory();
    }
  }
}

// Export a singleton instance
const historyService = new HistoryService();
export default historyService;

// Export the class for potential instantiation elsewhere if needed
export { HistoryService, type ChatHistoryItem };
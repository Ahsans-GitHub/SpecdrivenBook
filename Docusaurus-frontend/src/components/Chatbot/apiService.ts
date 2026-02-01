/**
 * API Service for Chatbot Component
 * Handles communication between the frontend chatbot and the backend RAG system
 */
// import axios, { AxiosInstance, AxiosResponse } from 'axios';

import axios, { AxiosInstance, AxiosResponse} from 'axios';

// Define TypeScript interfaces
interface SendMessageParams {
  query: string;
  selectedText?: string;
  sessionId?: string | null;
  topK?: number;
  minSimilarity?: number;
  temperature?: number;
}

interface ChatRequest {
  query: string;
  selected_text: string;
  session_id: string | null;
  top_k: number;
  min_similarity: number;
  temperature: number;
}

interface ChatResponse {
  response: string;
  sources: any[];
  session_id?: string;
  metadata: any;
  retrieval_time: number;
  generation_time: number;
}

interface HealthResponse {
  status: string;
  timestamp: string;
  services: Record<string, string>;
}

interface StatusResponse {
  status: string;
  vector_db_status: any;
  agent_status: any;
  uptime: number;
}

interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  status?: number;
}

/**
 * API service for communicating with the RAG backend
 */
class ApiService {
  private baseURL: string;
  private api: AxiosInstance;

  constructor() {
    try {
      // Use the backend URL from environment or default based on environment
      if (typeof window !== 'undefined') {
        // Browser environment
        this.baseURL = process.env.REACT_APP_BACKEND_URL ||
                      (window.location.hostname === 'localhost' ? 'https://ahsan350-rag.hf.space'
                       : window.location.origin); // For production, use the same origin
      } else {
        // Server environment (SSR)
        this.baseURL = process.env.REACT_APP_BACKEND_URL || 'https://ahsan350-rag.hf.space';
      }

      // Create axios instance with default configuration
      this.api = axios.create({
        baseURL: this.baseURL,
        timeout: 60000, // 60 second timeout to accommodate longer processing times
        headers: {
          'Content-Type': 'application/json',
        },
      });

      // Add request interceptor to log requests (for debugging)
      this.api.interceptors.request.use(
        (config) => {
          console.log('API Request:', config.method?.toUpperCase(), config.baseURL + config.url);
          return config;
        },
        (error) => {
          console.error('API Request Error:', error);
          return Promise.reject(error);
        }
      );

      // Add response interceptor to log responses (for debugging)
      this.api.interceptors.response.use(
        (response) => {
          console.log('API Response:', response.status, response.config.url);
          return response;
        },
        (error) => {
          console.error('API Response Error:', error.response || error.message);
          return Promise.reject(error);
        }
      );
    } catch (error) {
      console.error('Failed to initialize API service:', error);
      // Fallback to a default configuration if initialization fails
      this.baseURL = 'https://ahsan350-rag.hf.space';
      this.api = axios.create({
        baseURL: this.baseURL,
        timeout: 60000, // Consistent 60 second timeout
        headers: {
          'Content-Type': 'application/json',
        },
      });
    }
  }

  /**
   * Send a chat message to the backend with retry logic
   * @param params - Parameters for the chat request
   * @param retries - Number of retry attempts (default: 2)
   * @param delay - Delay between retries in ms (default: 2000)
   * @returns Promise containing the response from the backend
   */
  async sendMessage(params: SendMessageParams, retries = 2, delay = 2000): Promise<ApiResponse<ChatResponse>> {
    const {
      query,
      selectedText = '',
      sessionId = null,
      topK = 3,
      minSimilarity = 0.4,
      temperature = 0.7
    } = params;
    let lastError: any = null;

    for (let attempt = 0; attempt <= retries; attempt++) {
      try {
        const requestData: ChatRequest = {
          query,
          selected_text: selectedText,
          session_id: sessionId,
          top_k: topK,
          min_similarity: minSimilarity,
          temperature: temperature,
        };

        // Temporarily increase timeout for this specific request if needed
        const response: AxiosResponse<ChatResponse> = await this.api.post('/chat', requestData, {
          timeout: 90000, // 90 seconds for chat requests specifically
        });

        return {
          success: true,
          data: response.data,
        };
      } catch (error: any) {
        lastError = error;
        console.error(`Error sending message (attempt ${attempt + 1}/${retries + 1}):`, error);

        // Check if it's a timeout error specifically
        if (error.code === 'ECONNABORTED' || error.message.includes('timeout')) {
          console.warn(`Request timed out on attempt ${attempt + 1}, retrying...`);
        }

        // Don't wait after the last attempt
        if (attempt < retries) {
          // Exponential backoff: wait longer after each failed attempt
          const waitTime = delay * Math.pow(2, attempt);
          await this.delay(waitTime);
        }
      }
    }

    // All retries exhausted, return the last error
    return {
      success: false,
      error: lastError?.response?.data?.detail || lastError?.message || 'Unknown error occurred',
      status: lastError?.response?.status || null,
    };
  }

  /**
   * Check the health of the backend service
   * @returns Promise containing the health status response
   */
  async checkHealth(): Promise<ApiResponse<HealthResponse>> {
    try {
      const response: AxiosResponse<HealthResponse> = await this.api.get('/health');
// response.data is typed as HealthResponse
// response.status, response.headers, etc. are also available
      return {
        success: true,
        data: response.data,
      };
    } catch (error: any) {
      console.error('Error checking health:', error);

      return {
        success: false,
        error: error.response?.data?.detail || error.message || 'Health check failed',
        status: error.response?.status || null,
      };
    }
  }

  /**
   * Get detailed status information about the RAG system
   * @returns Promise containing the status information
   */
  async getStatus(): Promise<ApiResponse<StatusResponse>> {
    try {
      const response: AxiosResponse<StatusResponse> = await this.api.get('/status');

      return {
        success: true,
        data: response.data,
      };
    } catch (error: any) {
      console.error('Error getting status:', error);

      return {
        success: false,
        error: error.response?.data?.detail || error.message || 'Status check failed',
        status: error.response?.status || null,
      };
    }
  }

  /**
   * Test the connection to the backend
   * @returns Promise indicating whether the connection is successful
   */
  async testConnection(): Promise<boolean> {
    try {
      // Use a shorter timeout for health checks
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 10000); // 10 second timeout for health check

      try {
        const response = await this.checkHealth();
        clearTimeout(timeoutId);
        return response.success && response.data?.status !== 'unhealthy';
      } catch (error) {
        clearTimeout(timeoutId);
        // Re-throw the error to be caught by the caller
        throw error;
      }
    } catch (error) {
      console.error('Connection test failed:', error);
      return false;
    }
  }

  /**
   * Helper function to create a delay
   * @param ms - Milliseconds to wait
   * @returns Promise that resolves after the specified time
   */
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

// Export a singleton instance
const apiService = new ApiService();
export default apiService;

// Export the class for potential instantiation elsewhere if needed
export { ApiService };

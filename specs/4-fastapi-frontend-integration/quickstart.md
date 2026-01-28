# Quickstart Guide: FastAPI Frontend Integration

## Overview
This guide will help you quickly set up and run the integrated frontend-backend system for the Physical AI textbook RAG Chatbot.

## Prerequisites
- Node.js 18+ with npm or yarn
- Python 3.11+ with uv package manager
- Access to Qdrant vector database
- API keys for Cohere and your preferred LLM provider (OpenAI, DeepSeek, or OpenRouter)
- Both backend and frontend repositories set up

## Backend Setup (RAG-backend)

### 1. Navigate to the Backend Directory
```bash
cd RAG-backend
```

### 2. Install Backend Dependencies with uv
```bash
# Make sure uv is installed
pip install uv

# Install project dependencies
uv pip install -e .
```

### 3. Set Up Environment Variables
Create a `.env` file in the RAG-backend directory with your API keys:
```env
# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=physical_ai_textbook

# API Keys
COHERE_API_KEY=your_cohere_api_key
OPENAI_API_KEY=your_openai_api_key  # Or DEEPSEEK_API_KEY, OPENROUTER_API_KEY

# Model Configuration (optional)
MODEL_NAME=gpt-3.5-turbo  # Or deepseek/deepseek-r1-distill-llama-70b via OpenRouter
EMBEDDING_MODEL=embed-multilingual-v3.0

# Neon Postgres Configuration (optional, for conversation history)
NEON_POSTGRES_URL=your_neon_postgres_connection_string

# Application Configuration
CHUNK_SIZE=1000
OVERLAP_SIZE=200
DEBUG=true
```

### 4. Run the Backend Server
```bash
# Development mode
uvicorn api:app --reload --host 0.0.0.0 --port 8000

# Or using the project's main module
python -m uvicorn api:app --reload --port 8000
```

The backend should now be running on `http://localhost:8000`.

## Frontend Setup (Docusaurus-frontend)

### 1. Navigate to the Frontend Directory
```bash
cd Docusaurus-frontend
```

### 2. Install Frontend Dependencies
```bash
npm install
```

### 3. Configure Environment Variables
Create a `.env` file in the Docusaurus-frontend directory:
```env
# Backend API configuration
REACT_APP_BACKEND_URL=http://localhost:8000

# Optional: Analytics and other configurations
# REACT_APP_GA_ID=your_google_analytics_id
```

### 4. Run the Frontend Development Server
```bash
npm run dev
# or
npm start
```

The frontend will be available at `http://localhost:3000`.

## Integration Testing

### 1. Verify Backend Connection
- Visit `http://localhost:8000/health` to confirm the backend is running
- You should see a JSON response with service status information

### 2. Test API Endpoints
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is embodied AI in robotics?",
    "top_k": 3,
    "min_similarity": 0.4
  }'
```

### 3. Test Frontend Integration
- Visit the Docusaurus site at `http://localhost:3000`
- Look for the embedded chatbot component
- Test the chat functionality to ensure it connects to the backend

## Using the Chatbot Component

### 1. Embedding in Docusaurus Pages
To embed the chatbot in a Docusaurus page, use the component:
```jsx
import Chatbot from '@site/src/components/Chatbot/Chatbot';

<Chatbot title="Physical AI Assistant" />
```

### 2. Text Selection Feature
- Select text in the book content
- The chatbot will automatically detect the selection
- When you ask a question, the selected text context will be included

### 3. Multi-turn Conversations
- The chatbot maintains conversation history
- Ask follow-up questions that reference previous exchanges
- Session context is preserved across messages

## Key Features

### 1. Backend-Frontend Connection
- Local connection established between frontend and backend
- API communication with proper error handling
- Real-time response with source citations

### 2. Selected Text Context
- JavaScript event listeners capture selected text
- Context automatically passed to backend with queries
- Responses incorporate selected text context

### 3. Multi-turn Conversations
- Session management with UUID-based tracking
- Conversation history maintained across exchanges
- Context preservation for natural interaction

### 4. Futuristic UI Elements
- Adaptive prompts based on conversation context
- Confidence score visualization
- Real-time feedback indicators
- Source citations with clickable links

## Running Both Servers Simultaneously

### Option 1: Separate Terminals
Open two terminals:
- Terminal 1: Run the backend server (`uvicorn api:app --reload --host 0.0.0.0 --port 8000`)
- Terminal 2: Run the frontend server (`npm start`)

### Option 2: Using Concurrently (if configured)
```bash
# From the root directory, if a combined script exists
npm run dev:both
```

## Troubleshooting

### Common Issues
1. **CORS Errors**: Verify backend allows requests from `http://localhost:3000`
2. **Connection Issues**: Ensure both servers are running on correct ports
3. **Environment Variables**: Double-check all required environment variables are set
4. **Dependency Issues**: Clear caches and reinstall if needed

### Backend Troubleshooting
- Check backend logs for errors
- Verify all API keys are correct
- Confirm Qdrant connection

### Frontend Troubleshooting
- Check browser console for JavaScript errors
- Verify API endpoint configuration
- Test network connectivity to backend

## Production Deployment

### Backend
- Use a production WSGI server like Gunicorn instead of Uvicorn's development server
- Configure proper security headers and CORS policies
- Set up a reverse proxy (nginx) in production

### Frontend
- Build the frontend: `npm run build`
- Serve the build directory with a web server
- Configure proper proxy rules for API requests if needed

## Next Steps

1. Customize the chatbot component styling to match your site
2. Add more sophisticated error handling and loading states
3. Implement user session management if needed
4. Add analytics and monitoring
5. Test with real content and user workflows
# Quickstart Guide: Frontend-Backend Integration

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

### 2. Set Up Environment Variables
Create a `.env` file in the RAG-backend directory with your API keys:

```env
# Qdrant Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=physical_ai

# API Keys
COHERE_API_KEY=your_cohere_api_key_here
OPENAI_API_KEY=your_openai_api_key_here  # Optional, for fallback
DEEPSEEK_API_KEY=your_deepseek_api_key_here  # Or use OPENROUTER_API_KEY
OPENROUTER_API_KEY=your_openrouter_api_key_here  # Alternative to DeepSeek

# Model Configuration (optional)
MODEL_NAME=deepseek/deepseek-r1-distill-llama-70b
EMBEDDING_MODEL=embed-multilingual-v3.0

# Application Configuration
CHUNK_SIZE=1000
OVERLAP_SIZE=200
DEBUG=true
```

### 3. Install Backend Dependencies with uv
```bash
# Install uv if not already installed
pip install uv

# Install project dependencies
uv pip install -e .
```

### 4. Run the Backend Server
```bash
# Using uvicorn directly
uvicorn api:app --reload --host 0.0.0.0 --port 8000

# Or using the project's start script if available
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
- Backend API documentation: `http://localhost:8000/docs`
- Test the chat endpoint with curl:
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

## Key Features Available

### 1. Chatbot Integration
- Embedded chatbot component with real-time responses
- Support for selected text context
- Multi-turn conversation capabilities
- Source citations and metadata display

### 2. Adaptive UI Elements
- Dynamic confidence score visualization
- Suggested prompts based on conversation context
- Loading indicators and error handling
- Responsive design for different screen sizes

### 3. Error Handling
- Network failure retry logic
- Graceful degradation when backend is unavailable
- User-friendly error messages
- Connection status indicators

## Troubleshooting

### Common Issues
1. **CORS Errors**: Ensure backend allows requests from `http://localhost:3000`
2. **Connection Issues**: Verify both servers are running on correct ports
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
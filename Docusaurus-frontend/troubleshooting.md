# Troubleshooting Guide: Frontend-Backend Integration

## Common Issues and Solutions

### 1. Connection Issues

#### Problem: Frontend cannot connect to backend
**Symptoms:**
- Network errors in browser console
- "Failed to fetch" messages
- Connection timeouts

**Solutions:**
1. Verify backend is running on the correct port (usually 8000)
   ```bash
   # Check if backend is listening
   netstat -an | grep 8000
   # Or on newer systems
   ss -tuln | grep 8000
   ```

2. Check CORS configuration in backend
   - Ensure the backend allows requests from your frontend origin (e.g., `http://localhost:3000`)
   - Look for CORS middleware in `api.py`

3. Verify environment variable configuration
   - Check `REACT_APP_BACKEND_URL` in frontend `.env` file
   - Ensure it matches the actual backend URL

#### Problem: Backend returns 404 for API endpoints
**Symptoms:**
- HTTP 404 errors when calling `/chat`, `/health`, etc.
- API documentation shows endpoints are missing

**Solutions:**
1. Check if FastAPI app is properly configured
2. Verify the backend server is running the correct application
3. Confirm the correct API routes are registered

### 2. Authentication and Authorization

#### Problem: Unauthorized access errors
**Symptoms:**
- HTTP 401 or 403 errors
- API keys rejected
- Permission denied messages

**Solutions:**
1. Verify all required API keys are set in the backend `.env` file
2. Check that API keys are valid and have the required permissions
3. Confirm API key formats match what the providers expect

### 3. Performance Issues

#### Problem: Slow response times (>10 seconds)
**Symptoms:**
- Long delays in chat responses
- Timeout errors
- Poor user experience

**Solutions:**
1. Check Qdrant database performance
   - Ensure sufficient indexing
   - Verify database is not overloaded
   - Consider optimizing query parameters (top_k, min_similarity)

2. Verify LLM provider connectivity
   - Check if OpenAI/other providers are responding quickly
   - Confirm API quotas have not been exceeded

3. Optimize frontend rendering
   - Minimize unnecessary re-renders
   - Implement proper loading states

### 4. Data and Content Issues

#### Problem: No results returned from queries
**Symptoms:**
- Empty responses from /chat endpoint
- "No relevant content found" messages
- Low confidence scores

**Solutions:**
1. Verify content has been properly ingested into Qdrant
   - Check if documents exist in the collection
   - Confirm embeddings were generated correctly

2. Adjust query parameters
   - Lower `min_similarity` threshold
   - Increase `top_k` to retrieve more results
   - Modify query phrasing

3. Check embedding model consistency
   - Ensure the same model was used for ingestion and querying

### 5. Frontend-Specific Issues

#### Problem: Chatbot component not rendering
**Symptoms:**
- Blank space where chatbot should appear
- JavaScript errors in console
- Component fails to load

**Solutions:**
1. Verify all required dependencies are installed
   ```bash
   # Reinstall if needed
   npm install
   ```

2. Check for TypeScript compilation errors
3. Verify component imports are correct

#### Problem: Selected text context not working
**Symptoms:**
- Selected text not captured
- Context not passed to backend
- Selection functionality broken

**Solutions:**
1. Check if selection event listeners are properly attached
2. Verify text extraction logic handles different content types
3. Confirm selected text is properly truncated and sanitized

### 6. Build and Deployment Issues

#### Problem: Frontend build fails
**Symptoms:**
- `npm run build` fails with errors
- Production build contains errors
- Missing assets in build

**Solutions:**
1. Check for missing environment variables during build
2. Verify all dependencies are properly declared
3. Clear npm cache if needed:
   ```bash
   npm cache clean --force
   rm -rf node_modules package-lock.json
   npm install
   ```

#### Problem: Backend deployment issues
**Symptoms:**
- Server fails to start in production
- Dependency conflicts
- Port binding errors

**Solutions:**
1. Verify production dependencies match development
2. Check for environment-specific configurations
3. Confirm correct Python version is used

## Debugging Steps

### 1. Frontend Debugging
1. Open browser developer tools
2. Check the Console tab for JavaScript errors
3. Check the Network tab for API call results
4. Verify request/response payloads

### 2. Backend Debugging
1. Check backend logs for error messages
2. Verify environment variables are loaded
3. Test API endpoints directly with curl or Postman
4. Monitor system resources (CPU, memory, disk)

### 3. Integration Debugging
1. Test backend API endpoints independently
2. Verify frontend is calling correct endpoints
3. Check data format consistency between frontend and backend
4. Monitor network traffic for anomalies

## Useful Commands

### Frontend Diagnostics
```bash
# Check environment variables
echo $REACT_APP_BACKEND_URL

# Clear browser cache and reload
# (Use Ctrl+Shift+R or Cmd+Shift+R)

# Check for dependency conflicts
npm ls
```

### Backend Diagnostics
```bash
# Check if backend is running
curl http://localhost:8000/health

# Test chat endpoint directly
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "test"}'

# Check environment variables
python -c "import os; print(os.environ.get('QDRANT_URL', 'Not set'))"
```

## Contact and Support

### When to Escalate
Contact your development team if you encounter:
- Persistent infrastructure issues
- API provider outages
- Data corruption problems
- Security concerns

### Logs to Collect
When reporting issues, provide:
- Frontend browser console errors
- Backend server logs
- Network request/response details
- Environment configuration
- Steps to reproduce the issue
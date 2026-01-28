# Data Model: FastAPI Frontend Integration

## 1. Frontend Message Entity

### Fields
- `message_id`: string (UUID) - Unique identifier for the message
- `session_id`: string (UUID) - Session identifier for the conversation
- `sender`: enum ('user', 'assistant', 'system') - Who sent the message
- `content`: string (max 2000 chars) - Content of the message
- `timestamp`: datetime - When the message was sent
- `sources`: array of SourceReference (optional) - Sources cited in the message
- `metadata`: object (optional) - Additional metadata for UI enhancements
- `selected_text_context`: string (optional) - Selected text context if applicable
- `message_type`: enum ('query', 'response', 'citation', 'system') - Type of message

### Validation Rules
- `message_id`: Required, UUID format
- `session_id`: Required if part of conversation, UUID format
- `sender`: Required, one of enum values
- `content`: Required, non-empty, max 2000 chars
- `timestamp`: Required, ISO 8601 format
- `message_type`: Required, one of enum values
- `sources`: Array of SourceReference objects if provided
- `selected_text_context`: Max 2000 chars if provided

### Relationships
- Belongs to one ConversationSession (many-to-one)

## 2. SourceReference Entity

### Fields
- `source_id`: string - Unique identifier for the source
- `title`: string - Title of the source document
- `url`: string - URL of the source
- `content_preview`: string - Preview of the content (first 200 chars)
- `section`: string - Section of the textbook where source appears
- `similarity_score`: float - Similarity score to the query (0.0-1.0)
- `relevance_score`: float - Relevance score (0.0-1.0)
- `tags`: array of strings - Associated tags for categorization

### Validation Rules
- `source_id`: Required, unique identifier
- `title`: Required, non-empty
- `url`: Required, valid URL format
- `content_preview`: Required, max 500 chars
- `similarity_score`: Required, float between 0.0 and 1.0
- `relevance_score`: Required, float between 0.0 and 1.0
- `tags`: Array of strings, max 10 tags, each max 50 chars

### Relationships
- Embedded in FrontendMessage (no separate table needed)

## 3. ConversationSession Entity

### Fields
- `session_id`: string (UUID) - Unique session identifier
- `user_id`: string (optional) - User identifier if available
- `created_at`: datetime - When the session was created
- `last_interaction`: datetime - When the last interaction occurred
- `active`: boolean - Whether the session is currently active
- `metadata`: object - Additional session metadata
- `message_count`: integer - Count of messages in the session

### Validation Rules
- `session_id`: Required, UUID format
- `created_at`: Required, ISO 8601 format
- `last_interaction`: Required, ISO 8601 format, >= created_at
- `active`: Required, boolean
- `message_count`: Required, non-negative integer
- `user_id`: UUID format if provided

### Relationships
- Contains many FrontendMessages (one-to-many)

## 4. APIRequest Entity

### Fields
- `request_id`: string (UUID) - Unique identifier for the API request
- `endpoint`: string - API endpoint being called (e.g., "/chat", "/health")
- `method`: enum ('GET', 'POST', 'PUT', 'DELETE') - HTTP method
- `request_body`: object - Request payload
- `headers`: object - Request headers
- `timestamp`: datetime - When the request was made
- `response_time_ms`: float - Time taken for response in milliseconds
- `status_code`: integer - HTTP status code of response
- `session_id`: string (UUID, optional) - Associated session ID

### Validation Rules
- `request_id`: Required, UUID format
- `endpoint`: Required, non-empty
- `method`: Required, one of enum values
- `timestamp`: Required, ISO 8601 format
- `response_time_ms`: Required, positive number
- `status_code`: Required, integer between 100-599
- `session_id`: UUID format if provided

### Relationships
- May belong to one ConversationSession (many-to-one, optional)

## 5. APIResponse Entity

### Fields
- `response_id`: string (UUID) - Unique identifier for the response
- `request_id`: string (UUID) - Reference to the associated request
- `status_code`: integer - HTTP status code
- `response_body`: object - Response payload
- `headers`: object - Response headers
- `timestamp`: datetime - When the response was received
- `response_size_bytes`: integer - Size of response in bytes
- `error_message`: string (optional) - Error message if request failed

### Validation Rules
- `response_id`: Required, UUID format
- `request_id`: Required, UUID format, references existing APIRequest
- `status_code`: Required, integer between 100-599
- `timestamp`: Required, ISO 8601 format
- `response_size_bytes`: Required, non-negative integer
- `error_message`: Max 1000 chars if provided

### Relationships
- Belongs to one APIRequest (many-to-one)

## 6. UserSelectionContext Entity

### Fields
- `selection_id`: string (UUID) - Unique identifier for the selection
- `session_id`: string (UUID) - Associated conversation session
- `selected_text`: string (max 2000 chars) - The selected text
- `source_element`: string - Element where text was selected (URL or selector)
- `selection_bounds`: object - Start/end positions of the selection
  - `start_offset`: integer - Start position in text
  - `end_offset`: integer - End position in text
  - `start_container`: string - Starting container element
  - `end_container`: string - Ending container element
- `timestamp`: datetime - When the selection was captured
- `query_associated`: string (optional) - Associated query that used this context

### Validation Rules
- `selection_id`: Required, UUID format
- `session_id`: Required, UUID format
- `selected_text`: Required, non-empty, max 2000 chars
- `source_element`: Required, non-empty
- `timestamp`: Required, ISO 8601 format
- `start_offset`: Required, non-negative integer
- `end_offset`: Required, non-negative integer, >= start_offset

### Relationships
- Belongs to one ConversationSession (many-to-one)
- May be associated with one or more ChatRequests (many-to-many via association)

## 7. ChatMetadata Entity

### Fields
- `metadata_id`: string (UUID) - Unique identifier for the metadata
- `session_id`: string (UUID) - Associated conversation session
- `request_id`: string (UUID) - Associated request ID
- `confidence_score`: float (0.0-1.0) - Confidence in the response
- `adaptive_prompt_hint`: string (optional) - Hint for adaptive prompts
- `retrieval_success`: boolean - Whether retrieval was successful
- `adaptive_prompts`: array of strings - Suggested follow-up prompts
- `visualization_suggestions`: array of strings - Suggested visualizations
- `timestamp`: datetime - When metadata was generated
- `ui_enhancement_metadata`: object - Metadata for UI enhancements
  - `has_visualization_opportunities`: boolean
  - `suggest_follow_up_questions`: boolean
  - `suggest_related_topics`: array of strings
  - `suggest_content_format`: string

### Validation Rules
- `metadata_id`: Required, UUID format
- `session_id`: Required, UUID format
- `request_id`: Required, UUID format
- `confidence_score`: Required, float between 0.0 and 1.0
- `timestamp`: Required, ISO 8601 format
- `adaptive_prompts`: Array of strings, max 5 prompts, each max 100 chars
- `visualization_suggestions`: Array of strings, max 10 suggestions
- `suggest_related_topics`: Array of strings, max 5 topics

### Relationships
- Belongs to one ConversationSession (many-to-one)
- Associated with one APIRequest (many-to-one)

## 8. FrontendComponentState Entity

### Fields
- `state_id`: string (UUID) - Unique identifier for the component state
- `component_name`: string - Name of the component (e.g., "ChatWidget", "MessageInput")
- `session_id`: string (UUID) - Associated conversation session
- `state_data`: object - Current state of the component
- `last_updated`: datetime - When state was last updated
- `user_action`: string - Last user action that triggered state change
- `error_state`: boolean - Whether component is in an error state

### Validation Rules
- `state_id`: Required, UUID format
- `component_name`: Required, non-empty
- `session_id`: Required, UUID format
- `last_updated`: Required, ISO 8601 format
- `user_action`: Required, non-empty
- `error_state`: Required, boolean

### Relationships
- Belongs to one ConversationSession (many-to-one)

## API Contract Models

### Frontend ChatRequest Model
```typescript
interface FrontendChatRequest {
  query: string;
  selected_text?: string;
  session_id?: string;
  top_k?: number;
  min_similarity?: number;
  temperature?: number;
}

interface Validation {
  query: {
    required: true;
    minLength: 1;
    maxLength: 2000;
  };
  selected_text: {
    maxLength: 2000;
  };
  session_id: {
    format: "uuid";
  };
  top_k: {
    min: 1;
    max: 20;
  };
  min_similarity: {
    min: 0.0;
    max: 1.0;
  };
  temperature: {
    min: 0.0;
    max: 2.0;
  };
}
```

### Frontend ChatResponse Model
```typescript
interface FrontendChatResponse {
  response: string;
  sources: SourceReference[];
  session_id: string;
  metadata: ChatMetadata;
  retrieval_time: number;
  generation_time: number;
  timestamp: string;
}

interface Validation {
  response: {
    required: true;
    minLength: 1;
  };
  sources: {
    type: "array";
    maxItems: 10;
  };
  session_id: {
    required: true;
    format: "uuid";
  };
  retrieval_time: {
    required: true;
    minimum: 0;
  };
  generation_time: {
    required: true;
    minimum: 0;
  };
  timestamp: {
    required: true;
    format: "iso-date-time";
  };
}
```

## 9. State Transitions

### Component States
- **INITIAL**: Component just loaded, no interaction yet
- **LOADING**: Component is processing a request
- **ACTIVE**: Component is ready for interaction
- **ERROR**: Component encountered an error
- **DISCONNECTED**: Component lost connection to backend

### Transition Rules
- INITIAL → LOADING: User submits a query
- LOADING → ACTIVE: Query successfully processed
- LOADING → ERROR: Query processing failed
- ACTIVE → LOADING: User submits another query
- ACTIVE → DISCONNECTED: Network connection lost
- ERROR → LOADING: User retries after error
- DISCONNECTED → LOADING: User attempts to reconnect

## 10. Indexing Strategy

### Database Indexes
- ConversationSession: Index on `session_id`, `user_id`, `created_at`, `active`
- FrontendMessage: Index on `session_id`, `timestamp`, `sender`
- APIRequest: Index on `session_id`, `timestamp`, `endpoint`
- UserSelectionContext: Index on `session_id`, `timestamp`
- ChatMetadata: Index on `session_id`, `request_id`, `timestamp`
- SourceReference: Index on `source_id`, embedded in messages

### Performance Considerations
- Primary key indexes on all ID fields
- Composite indexes for common query patterns (session_id + timestamp)
- Time-based partitioning for historical data
- Efficient indexing for frequent lookup patterns
- Optimized for read-heavy workloads (responses viewed more than created)
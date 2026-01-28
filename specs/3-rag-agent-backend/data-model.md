# Data Model: RAG Agent Backend

## 1. ChatRequest Entity

### Fields
- `query`: string (max 2000 chars) - Main query from user
- `selected_text`: string (max 2000 chars, optional) - Selected text from book content for context
- `session_id`: string (UUID, optional) - For multi-turn conversations
- `top_k`: integer (1-20, default: 3) - Number of results to retrieve
- `min_similarity`: float (0.0-1.0, default: 0.4) - Minimum similarity threshold
- `temperature`: float (0.0-2.0, default: 0.7) - Temperature for generation
- `timestamp`: datetime - When query was submitted

### Validation Rules
- `query`: Required, min 1 char, max 2000 chars
- `selected_text`: Optional, max 2000 chars if provided
- `session_id`: UUID format if provided
- `top_k`: Integer between 1 and 20
- `min_similarity`: Float between 0.0 and 1.0
- `temperature`: Float between 0.0 and 2.0
- `timestamp`: ISO 8601 format

### Relationships
- Belongs to one ConversationSession (many-to-one, if session_id provided)

## 2. ChatResponse Entity

### Fields
- `response`: string - Generated response from agent
- `sources`: array of RetrievedChunk - Citations and references
- `session_id`: string (UUID) - Associated conversation session
- `metadata`: object - Additional metadata for UI enhancements
- `retrieval_time`: float - Time taken for retrieval phase in seconds
- `generation_time`: float - Time taken for generation phase in seconds
- `timestamp`: datetime - When response was generated

### Validation Rules
- `response`: Required, non-empty
- `sources`: Array of RetrievedChunk objects
- `session_id`: Required, UUID format
- `retrieval_time`: Required, positive number
- `generation_time`: Required, positive number
- `timestamp`: Required, ISO 8601 format

### Relationships
- Belongs to one ConversationSession (many-to-one)

## 3. RetrievedChunk Entity

### Fields
- `id`: string - Unique identifier for the chunk
- `title`: string - Title of the source document
- `url`: string - URL of the source document
- `content`: string - Content of the retrieved chunk
- `section`: string - Section of the textbook
- `tags`: array of strings - Associated tags
- `score`: float - Relevance score from retrieval
- `similarity`: float - Similarity score to the query
- `context_window`: string (optional) - Broader context around the chunk

### Validation Rules
- `id`: Required, unique identifier
- `content`: Required, non-empty
- `score`: Required, positive number
- `similarity`: Required, float between 0.0 and 1.0
- `tags`: Array of strings, each with max 50 chars

### Relationships
- Embedded in ChatResponse (no separate table needed)

## 4. ConversationSession Entity

### Fields
- `session_id`: string (UUID) - Unique session identifier
- `user_id`: string (optional) - User identifier if available
- `created_at`: datetime - Session creation time
- `last_interaction`: datetime - Last activity in session
- `active`: boolean - Whether session is currently active
- `conversation_history`: array of ConversationTurn (optional) - Full conversation history
- `metadata`: object - Additional session metadata

### Validation Rules
- `session_id`: Required, UUID format
- `created_at`: Required, ISO 8601 format
- `last_interaction`: Required, ISO 8601 format, >= created_at
- `active`: Required, boolean
- `conversation_history`: Array of ConversationTurn objects, max 50 turns

### Relationships
- Contains many ChatRequests/ChatResponses (one-to-many via session_id)

## 5. ConversationTurn Entity

### Fields
- `turn_id`: string (UUID) - Unique identifier for the turn
- `session_id`: string (UUID) - Reference to conversation session
- `user_message`: string - User's input message
- `agent_response`: string - Agent's response
- `sources_used`: array of RetrievedChunk - Sources used in this turn
- `timestamp`: datetime - When the turn occurred
- `metadata`: object - Additional turn-specific metadata

### Validation Rules
- `turn_id`: Required, UUID format
- `session_id`: Required, UUID format, references existing session
- `user_message`: Required, non-empty
- `agent_response`: Required, non-empty
- `timestamp`: Required, ISO 8601 format

### Relationships
- Belongs to one ConversationSession (many-to-one)

## 6. AgentState Entity

### Fields
- `query`: string - Original user query
- `processed_query`: string - Query potentially modified by selected text
- `retrieved_chunks`: array of RetrievedChunk - Chunks retrieved from vector store
- `context`: string - Formatted context for the LLM
- `response`: string - Final response from the agent
- `retrieval_time`: float - Time taken for retrieval
- `generation_time`: float - Time taken for generation
- `session_id`: string (UUID) - Associated session
- `metadata`: object - Additional processing metadata

### Validation Rules
- `query`: Required, non-empty
- `processed_query`: Required, non-empty
- `retrieval_time`: Required, positive number
- `generation_time`: Required, positive number
- `session_id`: Required, UUID format

### Relationships
- Belongs to one ConversationSession (many-to-one)

## 7. AgentMetadata Entity

### Fields
- `adaptive_prompt_hint`: string (optional) - Hint for adaptive prompts
- `confidence_score`: float (0.0-1.0) - Confidence in the response
- `retrieval_success`: boolean - Whether retrieval was successful
- `adaptive_prompts`: array of strings - Suggested follow-up prompts
- `ui_enhancement_metadata`: object - Metadata for UI enhancements
  - `has_visualization_opportunities`: boolean
  - `suggest_follow_up_questions`: boolean
  - `suggest_related_topics`: array of strings
  - `suggest_content_format`: string

### Validation Rules
- `confidence_score`: Float between 0.0 and 1.0 if provided
- `adaptive_prompts`: Array of strings, max 5 prompts
- `suggest_related_topics`: Array of strings, max 5 topics
- `suggest_content_format`: One of predefined formats

### Relationships
- Embedded in ChatResponse (no separate table needed)

## 8. ChatHistory Entity (for Neon Postgres if available)

### Fields
- `history_id`: string (UUID) - Unique identifier for the history record
- `session_id`: string (UUID) - Reference to conversation session
- `user_message`: string - User's message
- `assistant_response`: string - Agent's response
- `timestamp`: datetime - When the exchange happened
- `sources_cited`: array of strings - URLs of sources cited
- `interaction_type`: enum (query, followup, clarification) - Type of interaction

### Validation Rules
- `history_id`: Required, UUID format
- `session_id`: Required, UUID format, references existing session
- `user_message`: Required, non-empty
- `assistant_response`: Required, non-empty
- `timestamp`: Required, ISO 8601 format
- `interaction_type`: Required, one of enum values

### Relationships
- Belongs to one ConversationSession (many-to-one)

## API Request/Response Models

### ChatRequest Model
```python
class ChatRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=2000)
    selected_text: str = Field(default="", max_length=2000)
    session_id: Optional[str] = Field(default=None)
    top_k: int = Field(default=3, ge=1, le=20)
    min_similarity: float = Field(default=0.4, ge=0.0, le=1.0)
    temperature: float = Field(default=0.7, ge=0.0, le=2.0)

    @validator('query')
    def validate_query(cls, v):
        if len(v.strip()) == 0:
            raise ValueError('Query cannot be empty or whitespace only')
        return v.strip()

    @validator('session_id')
    def validate_session_id(cls, v):
        if v is not None:
            try:
                uuid.UUID(v)
            except ValueError:
                raise ValueError('session_id must be a valid UUID')
        return v
```

### ChatResponse Model
```python
class ChatResponse(BaseModel):
    response: str
    sources: List[RetrievedChunk]
    session_id: str
    metadata: Dict[str, Any]
    retrieval_time: float
    generation_time: float
    timestamp: datetime = Field(default_factory=datetime.utcnow)

    @validator('session_id')
    def validate_session_id(cls, v):
        try:
            uuid.UUID(v)
            return v
        except ValueError:
            raise ValueError('session_id must be a valid UUID')

    @validator('retrieval_time', 'generation_time')
    def validate_positive_time(cls, v):
        if v < 0:
            raise ValueError('Time values must be non-negative')
        return v
```

### RetrievedChunk Model
```python
class RetrievedChunk(BaseModel):
    id: str
    title: str
    url: str
    content: str
    section: str
    tags: List[str] = Field(default=[])
    score: float
    similarity: float

    @validator('similarity', 'score')
    def validate_score_range(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('Score and similarity must be between 0.0 and 1.0')
        return v

    @validator('tags')
    def validate_tags(cls, v):
        if len(v) > 10:
            raise ValueError('Maximum 10 tags allowed')
        for tag in v:
            if len(tag) > 50:
                raise ValueError('Tags must be less than 50 characters')
        return v
```

## 9. State Transitions

### Conversation Session States
- **CREATED**: Session initialized, no messages exchanged
- **ACTIVE**: At least one message exchanged, session ongoing
- **INACTIVE**: No activity for extended period (30 mins)
- **ENDED**: Explicitly ended by user or system

### Transition Rules
- CREATED → ACTIVE: First message sent/received
- ACTIVE → INACTIVE: No activity for 30 minutes
- INACTIVE → ACTIVE: New message received
- ACTIVE → ENDED: User explicitly ends session or timeout after 2 hours
- INACTIVE → ENDED: Timeout after extended inactivity (2 hours)

## 10. Indexing Strategy

### Database Indexes
- ConversationSession: Index on `session_id`, `user_id`, `created_at`, `active`
- ConversationTurn: Index on `session_id`, `timestamp`
- ChatHistory: Index on `session_id`, `timestamp`, `interaction_type`
- RetrievedChunk: Primary key indexes on `id`

### Performance Considerations
- Primary key indexes on all ID fields
- Composite indexes for common query patterns (session_id + timestamp)
- Time-based partitioning for historical conversation data
- Full-text search indexes for content search capabilities
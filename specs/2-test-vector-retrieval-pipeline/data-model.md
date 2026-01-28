# Data Model: Test Vector Retrieval Pipeline

## 1. TestQuery Entity

### Fields
- `query_id`: string (UUID) - Unique identifier for the test query
- `query_text`: string (max 2000 chars) - The test query text
- `expected_results`: array of strings - Expected relevant content IDs for validation
- `query_type`: enum (factual, conceptual, procedural, comparative) - Type of query
- `semantic_variants`: array of strings - Semantically equivalent query variations for testing
- `difficulty_level`: enum (basic, intermediate, advanced) - Difficulty level of the query
- `created_at`: datetime - When the test query was created

### Validation Rules
- `query_id`: Required, UUID format
- `query_text`: Required, min 1 char, max 2000 chars
- `query_type`: Required, must be one of the enum values
- `difficulty_level`: Required, must be one of the enum values
- `created_at`: Required, ISO 8601 format

### Relationships
- Associated with many TestResults (one-to-many)

## 2. TestResult Entity

### Fields
- `result_id`: string (UUID) - Unique identifier for the test result
- `query_id`: string (UUID) - Reference to the test query
- `retrieved_chunks`: array of RetrievedChunk - Actual results from system during test
- `precision_score`: float - Precision@K metric for the test run
- `recall_score`: float - Recall@K metric for the test run
- `confidence_correlation`: float - Correlation of confidence to relevance
- `response_time_ms`: float - Time taken for retrieval during test
- `test_timestamp`: datetime - When test was executed
- `test_environment`: string - Environment where test was run

### Validation Rules
- `result_id`: Required, UUID format
- `query_id`: Required, UUID format, references existing TestQuery
- `precision_score`: Required, float between 0.0 and 1.0
- `recall_score`: Required, float between 0.0 and 1.0
- `response_time_ms`: Required, positive number
- `test_timestamp`: Required, ISO 8601 format

### Relationships
- Belongs to one TestQuery (many-to-one)
- Contains many RetrievedChunks (embedded)

## 3. RetrievedChunk Entity (for testing)

### Fields
- `id`: string - Unique identifier for the chunk
- `content`: string - Content of the retrieved chunk
- `section`: string - Section of the textbook where content appears
- `similarity_score`: float - Similarity score from retrieval system
- `is_relevant`: boolean - Whether chunk is actually relevant to query (for validation)
- `source_url`: string - URL of the source document
- `source_title`: string - Title of the source document
- `tags`: array of strings - Associated tags for categorization

### Validation Rules
- `id`: Required, unique identifier
- `content`: Required, non-empty
- `similarity_score`: Required, float between 0.0 and 1.0
- `is_relevant`: Required, boolean value
- `tags`: Array of strings, each with max 50 chars

### Relationships
- Embedded in TestResult (no separate table needed for testing)

## 4. TestMetrics Entity

### Fields
- `metrics_id`: string (UUID) - Unique identifier for the metrics record
- `test_set_name`: string - Name of the test set (e.g., "physics_fundamentals", "robotics_algorithms")
- `total_queries`: integer - Number of queries in the test set
- `avg_precision`: float - Average precision across all queries
- `avg_recall`: float - Average recall across all queries
- `avg_response_time`: float - Average response time in milliseconds
- `success_rate`: float - Percentage of successful queries
- `test_run_timestamp`: datetime - When the test run was executed
- `environment_details`: object - Details about the test environment

### Validation Rules
- `metrics_id`: Required, UUID format
- `test_set_name`: Required, non-empty
- `total_queries`: Required, positive integer
- `avg_precision`: Required, float between 0.0 and 1.0
- `avg_recall`: Required, float between 0.0 and 1.0
- `avg_response_time`: Required, positive number
- `success_rate`: Required, float between 0.0 and 1.0
- `test_run_timestamp`: Required, ISO 8601 format

### Relationships
- Contains many TestResults (embedded or separate depending on implementation)

## 5. TestSet Entity

### Fields
- `set_id`: string (UUID) - Unique identifier for the test set
- `name`: string - Name of the test set
- `description`: string - Description of the test set purpose
- `query_count`: integer - Number of queries in the set
- `category`: string - Category of tests (accuracy, performance, semantic, etc.)
- `created_at`: datetime - When the test set was created
- `updated_at`: datetime - When the test set was last updated
- `queries`: array of TestQuery references - The queries included in this set

### Validation Rules
- `set_id`: Required, UUID format
- `name`: Required, unique within category
- `query_count`: Required, non-negative integer
- `category`: Required, one of predefined categories
- `created_at`: Required, ISO 8601 format, <= updated_at
- `updated_at`: Required, ISO 8601 format

### Relationships
- Contains many TestQueries (one-to-many)

## 6. ConfidenceValidation Entity

### Fields
- `validation_id`: string (UUID) - Unique identifier for the validation
- `test_result_id`: string (UUID) - Reference to the test result being validated
- `manual_rating`: float - Manual relevance rating (0.0-1.0 scale)
- `confidence_score`: float - System's confidence score
- `correlation_coefficient`: float - Correlation between confidence and relevance
- `validator_id`: string - Identifier of the person who performed manual validation
- `validation_timestamp`: datetime - When validation was performed
- `notes`: string - Additional notes about the validation

### Validation Rules
- `validation_id`: Required, UUID format
- `test_result_id`: Required, UUID format, references existing TestResult
- `manual_rating`: Required, float between 0.0 and 1.0
- `confidence_score`: Required, float between 0.0 and 1.0
- `correlation_coefficient`: Required, float between -1.0 and 1.0
- `validation_timestamp`: Required, ISO 8601 format

### Relationships
- Links to one TestResult (many-to-one)

## API Request/Response Models

### TestAccuracyRequest Model
```python
class TestAccuracyRequest(BaseModel):
    test_set_name: str
    top_k: int = Field(default=5, ge=1, le=20)
    min_similarity: float = Field(default=0.4, ge=0.0, le=1.0)

    @validator('top_k')
    def validate_top_k(cls, v):
        if v < 1 or v > 20:
            raise ValueError('top_k must be between 1 and 20')
        return v

    @validator('min_similarity')
    def validate_min_similarity(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('min_similarity must be between 0.0 and 1.0')
        return v
```

### TestAccuracyResponse Model
```python
class TestAccuracyResponse(BaseModel):
    test_set: str
    total_queries: int
    precision_at_k: float
    recall_at_k: float
    mean_reciprocal_rank: float
    results: List[Dict[str, Any]]
    execution_time_ms: float

    @validator('precision_at_k', 'recall_at_k', 'mean_reciprocal_rank')
    def validate_scores(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('Scores must be between 0.0 and 1.0')
        return v
```

### TestPerformanceRequest Model
```python
class TestPerformanceRequest(BaseModel):
    concurrent_users: int = Field(default=10, ge=1, le=100)
    test_duration: int = Field(default=60, ge=10, le=600)  # in seconds
    query_set: str = "standard"

    @validator('concurrent_users')
    def validate_concurrent_users(cls, v):
        if v < 1 or v > 100:
            raise ValueError('concurrent_users must be between 1 and 100')
        return v

    @validator('test_duration')
    def validate_test_duration(cls, v):
        if v < 10 or v > 600:
            raise ValueError('test_duration must be between 10 and 600 seconds')
        return v
```

## 7. State Transitions

### TestRun States
- **PENDING**: Test run scheduled but not started
- **RUNNING**: Test currently executing
- **COMPLETED**: Test run finished successfully
- **FAILED**: Test run failed due to errors
- **CANCELLED**: Test run cancelled by user

### Transition Rules
- PENDING → RUNNING: Test execution starts
- RUNNING → COMPLETED: All tests in run complete successfully
- RUNNING → FAILED: One or more tests fail critically
- PENDING/RUNNING → CANCELLED: User cancels test run

## 8. Indexing Strategy

### Database Indexes
- TestQuery: Index on `query_id`, `query_type`, `difficulty_level`
- TestResult: Index on `result_id`, `query_id`, `test_timestamp`
- TestMetrics: Index on `test_set_name`, `test_run_timestamp`
- TestSet: Index on `set_id`, `name`, `category`
- ConfidenceValidation: Index on `validation_id`, `test_result_id`, `validation_timestamp`

### Performance Considerations
- Primary key indexes on all ID fields
- Composite indexes for common query patterns (test_set + timestamp)
- Time-based partitioning for historical test results
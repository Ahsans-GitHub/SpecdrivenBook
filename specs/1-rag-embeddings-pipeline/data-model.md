# Data Model: RAG Embeddings Pipeline

## Entities

### ContentChunk
**Description**: Represents a processed chunk of content from the source URLs
- **id**: string (unique identifier for the chunk)
- **url**: string (source URL of the content)
- **text**: string (the actual text content of the chunk)
- **title**: string (title of the document/chapter/lesson)
- **section**: string (section identifier like "chapter", "module", "lesson")
- **level**: string (level tag like "beginner", "intermediate", "expert")
- **tags**: array of strings (additional metadata tags)
- **embedding**: array of floats (vector representation of the text content)
- **created_at**: timestamp (when the chunk was created)
- **processed_at**: timestamp (when the chunk was processed)

### EmbeddingRecord
**Description**: Represents a record stored in the vector database
- **id**: string (unique identifier matching the ContentChunk)
- **vector**: array of floats (embedding vector for similarity search)
- **payload**: object (metadata including url, text, title, section, level, tags)
- **collection**: string (name of the Qdrant collection)

### ProcessingResult
**Description**: Represents the result of processing a URL
- **url**: string (the source URL that was processed)
- **success**: boolean (whether the processing was successful)
- **chunks_processed**: integer (number of content chunks created)
- **error_message**: string (error message if processing failed)
- **processing_time**: float (time taken to process in seconds)

## Relationships

- Each **ContentChunk** generates one **EmbeddingRecord** (1:1)
- Each **ProcessingResult** may contain multiple **ContentChunk** records (1:many)

## Validation Rules

### ContentChunk Validation
- **url**: Must be a valid URL format
- **text**: Must not be empty, maximum 2000 characters (per Defensive Programming principle)
- **title**: Must not be empty
- **section**: Must be one of predefined values (e.g., "chapter", "module", "lesson", "part")
- **level**: Must be one of predefined values ("beginner", "intermediate", "expert")
- **embedding**: Must be a valid vector (array of floats)

### EmbeddingRecord Validation
- **id**: Must be unique within the collection
- **vector**: Must have consistent dimensions (per embedding model requirements)
- **payload**: Must contain required metadata fields

### ProcessingResult Validation
- **url**: Must be a valid URL format
- **chunks_processed**: Must be non-negative integer
- **processing_time**: Must be positive float

## State Transitions

### Content Processing Flow
1. **URL Fetched** → Raw HTML content retrieved from source URL
2. **Content Parsed** → HTML parsed into text content with metadata
3. **Content Chunked** → Text content split into semantic chunks
4. **Embedding Generated** → Embeddings created for each chunk
5. **Stored in Vector DB** → Embedding records stored in Qdrant with metadata
6. **Processing Complete** → Result recorded with success/failure status

## Constraints

- Each content chunk must be less than 2000 characters to prevent buffer overflows (per Defensive Programming principle)
- Embedding vectors must have consistent dimensions based on the Cohere model used
- URLs must be validated before processing to prevent injection attacks
- All timestamps must be in ISO 8601 format
- Error messages must be sanitized to prevent information disclosure
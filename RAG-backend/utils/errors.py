"""Error handling utilities for the RAG pipeline."""

class RAGPipelineError(Exception):
    """Base exception for RAG pipeline errors."""
    pass

class ContentFetchError(RAGPipelineError):
    """Raised when content fetching fails."""
    pass

class EmbeddingGenerationError(RAGPipelineError):
    """Raised when embedding generation fails."""
    pass

class VectorStorageError(RAGPipelineError):
    """Raised when vector storage operations fail."""
    pass

class ValidationError(RAGPipelineError):
    """Raised when validation fails."""
    pass

def handle_error(error: Exception, context: str = ""):
    """
    Generic error handler that logs and potentially re-raises errors.

    Args:
        error: The exception that occurred
        context: Additional context about where the error occurred

    Returns:
        str: A formatted error message
    """
    error_msg = f"Error in {context}: {str(error)} (Type: {type(error).__name__})"

    # Log the error (in a real implementation, this would go to a proper logging system)
    print(f"[ERROR] {error_msg}")

    return error_msg

def safe_execute(func, *args, context: str = "", **kwargs):
    """
    Safely execute a function and handle any exceptions.

    Args:
        func: The function to execute
        *args: Positional arguments to pass to the function
        context: Context description for error reporting
        **kwargs: Keyword arguments to pass to the function

    Returns:
        Tuple of (result, error_message) where result is None if error occurred
    """
    try:
        result = func(*args, **kwargs)
        return result, None
    except Exception as e:
        error_msg = handle_error(e, context)
        return None, error_msg
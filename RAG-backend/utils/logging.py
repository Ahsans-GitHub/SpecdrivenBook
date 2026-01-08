"""Logging configuration for the RAG pipeline."""

import logging
import sys
from datetime import datetime
from typing import Optional

class RAGLogger:
    """Custom logger class for the RAG pipeline with structured logging."""

    def __init__(self, name: str = "RAGPipeline", level: int = logging.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        # Avoid adding multiple handlers if logger already has handlers
        if not self.logger.handlers:
            # Create console handler
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(level)

            # Create formatter
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
            console_handler.setFormatter(formatter)

            # Add handler to logger
            self.logger.addHandler(console_handler)

    def info(self, message: str, extra: Optional[dict] = None):
        """Log an info message."""
        self.logger.info(message, extra=extra)

    def warning(self, message: str, extra: Optional[dict] = None):
        """Log a warning message."""
        self.logger.warning(message, extra=extra)

    def error(self, message: str, extra: Optional[dict] = None):
        """Log an error message."""
        self.logger.error(message, extra=extra)

    def debug(self, message: str, extra: Optional[dict] = None):
        """Log a debug message."""
        self.logger.debug(message, extra=extra)

    def log_pipeline_event(self, event_type: str, details: dict):
        """Log a structured pipeline event."""
        message = f"Pipeline Event: {event_type}"
        self.logger.info(message, extra=details)

    def log_processing_result(self, url: str, success: bool, chunks_count: int = 0, processing_time: float = 0.0):
        """Log the result of processing a URL."""
        event_details = {
            'event_type': 'processing_result',
            'url': url,
            'success': success,
            'chunks_count': chunks_count,
            'processing_time': processing_time,
            'timestamp': datetime.utcnow().isoformat()
        }
        self.logger.info(f"Processed URL: {url}, Success: {success}", extra=event_details)

    def log_embedding_result(self, chunk_id: str, success: bool, vector_dimension: int = 0):
        """Log the result of embedding generation."""
        event_details = {
            'event_type': 'embedding_result',
            'chunk_id': chunk_id,
            'success': success,
            'vector_dimension': vector_dimension,
            'timestamp': datetime.utcnow().isoformat()
        }
        self.logger.info(f"Embedding for chunk {chunk_id}, Success: {success}", extra=event_details)

    def log_storage_result(self, chunk_id: str, success: bool, collection: str = ""):
        """Log the result of storing embeddings."""
        event_details = {
            'event_type': 'storage_result',
            'chunk_id': chunk_id,
            'success': success,
            'collection': collection,
            'timestamp': datetime.utcnow().isoformat()
        }
        self.logger.info(f"Storage for chunk {chunk_id}, Success: {success}", extra=event_details)

# Global logger instance
rag_logger = RAGLogger()

def get_logger(name: str = "RAGPipeline") -> RAGLogger:
    """Get a logger instance."""
    return RAGLogger(name)

def log_function_call(func_name: str, params: dict = None):
    """Decorator to log function calls."""
    def decorator(func):
        def wrapper(*args, **kwargs):
            if params:
                rag_logger.debug(f"Calling {func_name} with params: {params}")
            else:
                rag_logger.debug(f"Calling {func_name}")

            result = func(*args, **kwargs)
            rag_logger.debug(f"Completed {func_name}")
            return result
        return wrapper
    return decorator
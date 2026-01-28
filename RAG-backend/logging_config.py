"""
Logging Configuration for RAG Agent Backend
This module sets up centralized logging for the entire RAG agent system.
"""

import logging
import logging.handlers
import os
import sys
from datetime import datetime
from pathlib import Path


def setup_logging(log_level: str = "INFO", log_dir: str = "logs"):
    """
    Set up centralized logging for the RAG agent system.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_dir: Directory to store log files
    """
    # Create logs directory if it doesn't exist
    Path(log_dir).mkdir(exist_ok=True)

    # Convert string log level to logging constant
    numeric_level = getattr(logging, log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f"Invalid log level: {log_level}")

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(funcName)s - %(message)s'
    )

    # Set up root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(numeric_level)

    # Clear any existing handlers
    root_logger.handlers.clear()

    # Console handler for development
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    console_handler.setLevel(numeric_level)
    root_logger.addHandler(console_handler)

    # File handler for general application logs
    app_log_file = os.path.join(log_dir, "rag_agent.log")
    file_handler = logging.handlers.RotatingFileHandler(
        app_log_file,
        maxBytes=10 * 1024 * 1024,  # 10MB
        backupCount=5
    )
    file_handler.setFormatter(formatter)
    file_handler.setLevel(numeric_level)
    root_logger.addHandler(file_handler)

    # Specialized handler for security-related logs
    security_log_file = os.path.join(log_dir, "security.log")
    security_handler = logging.handlers.RotatingFileHandler(
        security_log_file,
        maxBytes=10 * 1024 * 1024,  # 10MB
        backupCount=5
    )
    security_formatter = logging.Formatter(
        '%(asctime)s - SECURITY - %(levelname)s - %(message)s'
    )
    security_handler.setFormatter(security_formatter)
    security_handler.setLevel(logging.WARNING)  # Only log warnings and errors for security
    root_logger.addHandler(security_handler)

    # Specialized handler for performance metrics
    perf_log_file = os.path.join(log_dir, "performance.log")
    perf_handler = logging.handlers.RotatingFileHandler(
        perf_log_file,
        maxBytes=10 * 1024 * 1024,  # 10MB
        backupCount=5
    )
    perf_formatter = logging.Formatter(
        '%(asctime)s - PERF - %(message)s'
    )
    perf_handler.setFormatter(perf_formatter)
    perf_handler.setLevel(logging.INFO)
    root_logger.addHandler(perf_handler)

    logging.info("Logging configuration completed")


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name.

    Args:
        name: Name of the logger

    Returns:
        Configured logger instance
    """
    return logging.getLogger(name)


def log_request_details(logger: logging.Logger, request_id: str, details: dict):
    """
    Log detailed information about a request.

    Args:
        logger: Logger instance to use
        request_id: Unique request identifier
        details: Dictionary with request details
    """
    logger.info(f"REQUEST_DETAILS id={request_id} details={details}")


def log_response_details(logger: logging.Logger, request_id: str, details: dict):
    """
    Log detailed information about a response.

    Args:
        logger: Logger instance to use
        request_id: Unique request identifier
        details: Dictionary with response details
    """
    logger.info(f"RESPONSE_DETAILS id={request_id} details={details}")


def log_error_occurrence(logger: logging.Logger, error_type: str, error_details: dict):
    """
    Log an error occurrence with structured details.

    Args:
        logger: Logger instance to use
        error_type: Type of error that occurred
        error_details: Dictionary with error details
    """
    logger.error(f"ERROR_OCCURRED type={error_type} details={error_details}")


def log_performance_metric(logger: logging.Logger, operation: str, duration: float, details: dict = None):
    """
    Log a performance metric.

    Args:
        logger: Logger instance to use (should be the performance logger)
        operation: Name of the operation being measured
        duration: Duration of the operation in seconds
        details: Additional details about the operation
    """
    perf_details = f"operation={operation} duration={duration:.3f}s"
    if details:
        perf_details += f" extra={details}"

    logger.info(perf_details)


def log_security_event(logger: logging.Logger, event_type: str, details: dict):
    """
    Log a security-related event.

    Args:
        logger: Logger instance to use (should be the security logger)
        event_type: Type of security event
        details: Dictionary with event details
    """
    logger.warning(f"SECURITY_EVENT type={event_type} details={details}")


# Create specialized loggers
app_logger = get_logger("rag_agent")
security_logger = get_logger("rag_agent.security")
perf_logger = get_logger("rag_agent.performance")


class RAGLogger:
    """
    Specialized logger class for the RAG agent system.
    Provides structured logging for different aspects of the system.
    """

    def __init__(self, name: str = "rag_agent"):
        """
        Initialize the RAG logger.

        Args:
            name: Name for the logger
        """
        self.logger = get_logger(name)
        self.security_logger = get_logger(f"{name}.security")
        self.perf_logger = get_logger(f"{name}.performance")
        self.request_id = None

    def set_request_id(self, request_id: str):
        """
        Set the request ID for correlation across logs.

        Args:
            request_id: Unique request identifier
        """
        self.request_id = request_id

    def log_request_start(self, method: str, path: str, client_ip: str = None):
        """
        Log the start of a request.

        Args:
            method: HTTP method
            path: Request path
            client_ip: Client IP address
        """
        msg = f"REQUEST_START method={method} path={path}"
        if self.request_id:
            msg = f"REQ_ID={self.request_id} {msg}"
        if client_ip:
            msg += f" client_ip={client_ip}"

        self.logger.info(msg)

    def log_request_end(self, status_code: int, processing_time: float):
        """
        Log the end of a request.

        Args:
            status_code: HTTP status code
            processing_time: Time taken to process the request
        """
        msg = f"REQUEST_END status_code={status_code} processing_time={processing_time:.3f}s"
        if self.request_id:
            msg = f"REQ_ID={self.request_id} {msg}"

        self.logger.info(msg)

    def log_retrieval_start(self, query: str, top_k: int, min_similarity: float):
        """
        Log the start of a retrieval operation.

        Args:
            query: Query being processed
            top_k: Number of results to retrieve
            min_similarity: Minimum similarity threshold
        """
        msg = f"RETRIEVAL_START query_len={len(query)} top_k={top_k} min_similarity={min_similarity}"
        if self.request_id:
            msg = f"REQ_ID={self.request_id} {msg}"

        self.logger.info(msg)

    def log_retrieval_result(self, num_results: int, retrieval_time: float):
        """
        Log the result of a retrieval operation.

        Args:
            num_results: Number of results retrieved
            retrieval_time: Time taken for retrieval
        """
        msg = f"RETRIEVAL_RESULT count={num_results} time={retrieval_time:.3f}s"
        if self.request_id:
            msg = f"REQ_ID={self.request_id} {msg}"

        self.logger.info(msg)

    def log_generation_start(self, model: str, temperature: float):
        """
        Log the start of a generation operation.

        Args:
            model: Model being used for generation
            temperature: Temperature setting for generation
        """
        msg = f"GENERATION_START model={model} temperature={temperature}"
        if self.request_id:
            msg = f"REQ_ID={self.request_id} {msg}"

        self.logger.info(msg)

    def log_generation_result(self, generation_time: float, response_length: int):
        """
        Log the result of a generation operation.

        Args:
            generation_time: Time taken for generation
            response_length: Length of the generated response
        """
        msg = f"GENERATION_RESULT time={generation_time:.3f}s response_length={response_length}"
        if self.request_id:
            msg = f"REQ_ID={self.request_id} {msg}"

        self.logger.info(msg)

    def log_error(self, error_type: str, message: str, details: dict = None):
        """
        Log an error.

        Args:
            error_type: Type of error
            message: Error message
            details: Additional error details
        """
        msg = f"ERROR type={error_type} message={message}"
        if details:
            msg += f" details={details}"
        if self.request_id:
            msg = f"REQ_ID={self.request_id} {msg}"

        self.logger.error(msg)

    def log_security_warning(self, warning_type: str, message: str, details: dict = None):
        """
        Log a security warning.

        Args:
            warning_type: Type of security warning
            message: Warning message
            details: Additional details
        """
        msg = f"SECURITY_WARNING type={warning_type} message={message}"
        if details:
            msg += f" details={details}"
        if self.request_id:
            msg = f"REQ_ID={self.request_id} {msg}"

        self.security_logger.warning(msg)

    def log_performance(self, operation: str, duration: float, details: dict = None):
        """
        Log a performance metric.

        Args:
            operation: Operation being measured
            duration: Duration of the operation
            details: Additional details
        """
        msg = f"PERFORMANCE operation={operation} duration={duration:.3f}s"
        if details:
            msg += f" details={details}"
        if self.request_id:
            msg = f"REQ_ID={self.request_id} {msg}"

        self.perf_logger.info(msg)

    def log_embedding_result(self, chunk_id: str, success: bool, vector_dimension: int = None):
        """
        Log the result of an embedding operation.

        Args:
            chunk_id: ID of the chunk being embedded
            success: Whether the operation was successful
            vector_dimension: Dimension of the resulting vector
        """
        msg = f"EMBEDDING_RESULT chunk_id={chunk_id} success={success}"
        if vector_dimension:
            msg += f" dimension={vector_dimension}"

        self.logger.info(msg)

    def log_storage_result(self, chunk_id: str, success: bool, collection: str = None):
        """
        Log the result of a storage operation.

        Args:
            chunk_id: ID of the chunk being stored
            success: Whether the operation was successful
            collection: Collection where the chunk was stored
        """
        msg = f"STORAGE_RESULT chunk_id={chunk_id} success={success}"
        if collection:
            msg += f" collection={collection}"

        self.logger.info(msg)


# Global logger instance
rag_logger = RAGLogger()


# Set up default logging
setup_logging()
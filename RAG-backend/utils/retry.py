"""Retry mechanism with exponential backoff for the RAG pipeline."""

import time
import random
from typing import Callable, Any, Type, Tuple
from functools import wraps

def retry_with_backoff(
    max_retries: int = 3,
    base_delay: float = 1.0,
    max_delay: float = 60.0,
    backoff_factor: float = 2.0,
    jitter: bool = True,
    allowed_exceptions: Tuple[Type[Exception], ...] = (Exception,)
):
    """
    Decorator to retry a function with exponential backoff.

    Args:
        max_retries: Maximum number of retry attempts
        base_delay: Initial delay between retries in seconds
        max_delay: Maximum delay between retries in seconds
        backoff_factor: Factor by which delay increases after each retry
        jitter: Whether to add random jitter to delay to avoid thundering herd
        allowed_exceptions: Tuple of exceptions that trigger a retry
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return func(*args, **kwargs)
                except allowed_exceptions as e:
                    last_exception = e

                    if attempt == max_retries:
                        # Last attempt, re-raise the exception
                        raise last_exception

                    # Calculate delay with exponential backoff
                    delay = min(base_delay * (backoff_factor ** attempt), max_delay)

                    # Add jitter if requested
                    if jitter:
                        delay = delay * (0.5 + random.random() * 0.5)

                    print(f"[WARNING] {func.__name__} failed on attempt {attempt + 1}, "
                          f"retrying in {delay:.2f} seconds... Error: {str(e)}")

                    time.sleep(delay)

            # This line should never be reached, but included for type checking
            raise last_exception

        return wrapper
    return decorator

class RetryHandler:
    """Class-based retry handler for more complex retry scenarios."""

    def __init__(
        self,
        max_retries: int = 3,
        base_delay: float = 1.0,
        max_delay: float = 60.0,
        backoff_factor: float = 2.0,
        jitter: bool = True,
        allowed_exceptions: Tuple[Type[Exception], ...] = (Exception,)
    ):
        self.max_retries = max_retries
        self.base_delay = base_delay
        self.max_delay = max_delay
        self.backoff_factor = backoff_factor
        self.jitter = jitter
        self.allowed_exceptions = allowed_exceptions

    def execute_with_retry(self, func: Callable, *args, **kwargs) -> Any:
        """
        Execute a function with retry logic.

        Args:
            func: The function to execute
            *args: Positional arguments for the function
            **kwargs: Keyword arguments for the function

        Returns:
            The result of the function call

        Raises:
            The last exception if all retries are exhausted
        """
        last_exception = None

        for attempt in range(self.max_retries + 1):
            try:
                return func(*args, **kwargs)
            except self.allowed_exceptions as e:
                last_exception = e

                if attempt == self.max_retries:
                    # Last attempt, re-raise the exception
                    raise last_exception

                # Calculate delay with exponential backoff
                delay = min(self.base_delay * (self.backoff_factor ** attempt), self.max_delay)

                # Add jitter if requested
                if self.jitter:
                    delay = delay * (0.5 + random.random() * 0.5)

                print(f"[WARNING] Function execution failed on attempt {attempt + 1}, "
                      f"retrying in {delay:.2f} seconds... Error: {str(e)}")

                time.sleep(delay)

        # This line should never be reached, but included for type checking
        raise last_exception

def rate_limit_delay(delay: float = 1.0):
    """
    Decorator to add a fixed delay between function calls to respect rate limits.

    Args:
        delay: Delay in seconds between function calls
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            result = func(*args, **kwargs)
            time.sleep(delay)
            return result
        return wrapper
    return decorator
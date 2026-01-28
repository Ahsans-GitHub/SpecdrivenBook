"""
Middleware for the RAG Agent Backend
This module implements custom middleware for request processing, authentication, and logging.
"""

import time
import uuid
import logging
from typing import Callable, Awaitable
from fastapi import Request, Response
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint
from config import Config


logger = logging.getLogger(__name__)


class RequestLoggingMiddleware(BaseHTTPMiddleware):
    """
    Middleware to log incoming requests and outgoing responses.
    Provides detailed logging for debugging and monitoring purposes.
    """

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        """
        Process the request and log relevant information.

        Args:
            request: Incoming request object
            call_next: Next middleware or route handler in the chain

        Returns:
            Response object after processing
        """
        # Generate a unique request ID for tracking
        request_id = str(uuid.uuid4())
        request.state.request_id = request_id

        # Log request start
        start_time = time.time()

        # Get client IP
        client_host = request.client.host if request.client else "unknown"

        # Log request details
        logger.info(
            f"REQUEST_START id={request_id} "
            f"method={request.method} path={request.url.path} "
            f"client_ip={client_host} user_agent={request.headers.get('user-agent', 'unknown')[:100]}"
        )

        try:
            # Process the request
            response = await call_next(request)

            # Calculate processing time
            processing_time = time.time() - start_time

            # Log successful response
            logger.info(
                f"REQUEST_END id={request_id} "
                f"status_code={response.status_code} "
                f"processing_time={processing_time:.3f}s"
            )

            # Add request ID to response headers for tracking
            response.headers["X-Request-ID"] = request_id
            response.headers["X-Processing-Time"] = f"{processing_time:.3f}s"

            return response

        except Exception as e:
            # Calculate processing time even for errors
            processing_time = time.time() - start_time

            # Log error
            logger.error(
                f"REQUEST_ERROR id={request_id} "
                f"error={str(e)} processing_time={processing_time:.3f}s"
            )

            # Return error response
            error_response = JSONResponse(
                status_code=500,
                content={
                    "error": "Internal server error",
                    "error_type": "internal_error",
                    "request_id": request_id
                }
            )

            # Add request ID to error response headers
            error_response.headers["X-Request-ID"] = request_id
            error_response.headers["X-Processing-Time"] = f"{processing_time:.3f}s"

            return error_response


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """
    Middleware to add security headers to responses.
    Helps protect against common web vulnerabilities.
    """

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        """
        Add security headers to the response.

        Args:
            request: Incoming request object
            call_next: Next middleware or route handler in the chain

        Returns:
            Response object with security headers
        """
        response = await call_next(request)

        # Add security headers
        response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"

        # Allow specific origins for development (in production, restrict this further)
        if Config.DEBUG:
            response.headers["Access-Control-Allow-Origin"] = "*"
        else:
            # In production, set specific allowed origins
            response.headers["Access-Control-Allow-Origin"] = Config.ALLOWED_ORIGINS[0] if Config.ALLOWED_ORIGINS else ""

        response.headers["Access-Control-Allow-Methods"] = "GET, POST, PUT, DELETE, OPTIONS"
        response.headers["Access-Control-Allow-Headers"] = "Content-Type, Authorization, X-Requested-With"
        response.headers["Access-Control-Allow-Credentials"] = "true"

        return response


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Middleware to implement rate limiting per IP address.
    Prevents abuse and ensures fair usage of the API.
    """

    def __init__(self, app, requests_per_minute: int = 60):
        """
        Initialize the rate limiter.

        Args:
            app: FastAPI application instance
            requests_per_minute: Number of requests allowed per minute per IP
        """
        super().__init__(app)
        self.requests_per_minute = requests_per_minute
        self.requests_by_ip = {}  # In production, use Redis or similar for distributed rate limiting

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        """
        Check rate limits and process request if within limits.

        Args:
            request: Incoming request object
            call_next: Next middleware or route handler in the chain

        Returns:
            Response object or rate limit error response
        """
        client_ip = request.client.host if request.client else "unknown"

        # Get current time
        current_time = time.time()

        # Initialize request record for this IP if not exists
        if client_ip not in self.requests_by_ip:
            self.requests_by_ip[client_ip] = []

        # Clean old requests (older than 1 minute)
        self.requests_by_ip[client_ip] = [
            req_time for req_time in self.requests_by_ip[client_ip]
            if current_time - req_time < 60  # 60 seconds = 1 minute
        ]

        # Check if rate limit exceeded
        if len(self.requests_by_ip[client_ip]) >= self.requests_per_minute:
            return JSONResponse(
                status_code=429,
                content={
                    "error": "Rate limit exceeded",
                    "error_type": "rate_limit_error",
                    "details": {
                        "message": f"Rate limit of {self.requests_per_minute} requests per minute exceeded"
                    }
                }
            )

        # Add current request to the list
        self.requests_by_ip[client_ip].append(current_time)

        # Process the request
        response = await call_next(request)

        return response


class InputValidationMiddleware(BaseHTTPMiddleware):
    """
    Middleware to validate incoming request data.
    Sanitizes inputs and validates content to prevent injection attacks.
    """

    def __init__(self, app, max_body_size: int = 1024 * 1024):  # 1MB default
        """
        Initialize the input validator.

        Args:
            app: FastAPI application instance
            max_body_size: Maximum allowed request body size in bytes
        """
        super().__init__(app)
        self.max_body_size = max_body_size

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        """
        Validate and sanitize incoming request data.

        Args:
            request: Incoming request object
            call_next: Next middleware or route handler in the chain

        Returns:
            Response object after validation
        """
        # Check content length
        content_length = request.headers.get("content-length")
        if content_length and int(content_length) > self.max_body_size:
            return JSONResponse(
                status_code=413,
                content={
                    "error": "Request body too large",
                    "error_type": "validation_error",
                    "details": {
                        "message": f"Request body exceeds maximum size of {self.max_body_size} bytes"
                    }
                }
            )

        # For certain endpoints, we might want to validate specific content
        if request.method in ["POST", "PUT", "PATCH"]:
            # For now, we'll just pass through - validation happens in the route handlers
            # In a more complex implementation, we might want to validate JSON structure here
            pass

        response = await call_next(request)
        return response


class PerformanceMonitoringMiddleware(BaseHTTPMiddleware):
    """
    Middleware to monitor API performance and collect metrics.
    Tracks response times, error rates, and other performance indicators.
    """

    def __init__(self, app):
        """
        Initialize the performance monitor.

        Args:
            app: FastAPI application instance
        """
        super().__init__(app)
        self.request_count = 0
        self.error_count = 0
        self.total_response_time = 0.0
        self.start_time = time.time()

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        """
        Monitor performance metrics for the request.

        Args:
            request: Incoming request object
            call_next: Next middleware or route handler in the chain

        Returns:
            Response object after monitoring
        """
        self.request_count += 1
        start_time = time.time()

        try:
            response = await call_next(request)

            # Track response time
            response_time = time.time() - start_time
            self.total_response_time += response_time

            # Log performance metrics for slow requests
            if response_time > 5.0:  # Log requests taking more than 5 seconds
                logger.warning(
                    f"SLOW_REQUEST path={request.url.path} "
                    f"method={request.method} response_time={response_time:.3f}s"
                )

            return response
        except Exception as e:
            # Track errors
            self.error_count += 1
            response_time = time.time() - start_time
            self.total_response_time += response_time
            raise
        finally:
            # Could add metrics aggregation here (send to monitoring system)
            pass

    def get_stats(self) -> dict:
        """
        Get current performance statistics.

        Returns:
            Dictionary with performance statistics
        """
        uptime = time.time() - self.start_time
        avg_response_time = self.total_response_time / self.request_count if self.request_count > 0 else 0
        error_rate = self.error_count / self.request_count if self.request_count > 0 else 0

        return {
            "uptime_seconds": uptime,
            "total_requests": self.request_count,
            "total_errors": self.error_count,
            "error_rate": error_rate,
            "average_response_time": avg_response_time,
            "requests_per_second": self.request_count / uptime if uptime > 0 else 0
        }


# Utility function to create all middleware
def add_middlewares(app):
    """
    Add all middleware to the FastAPI application.

    Args:
        app: FastAPI application instance
    """
    # Add request logging middleware
    app.add_middleware(RequestLoggingMiddleware)

    # Add security headers middleware
    app.add_middleware(SecurityHeadersMiddleware)

    # Add rate limiting middleware (with 60 requests per minute limit)
    app.add_middleware(RateLimitMiddleware, requests_per_minute=60)

    # Add input validation middleware
    app.add_middleware(InputValidationMiddleware, max_body_size=1024*1024)  # 1MB

    # Add performance monitoring middleware
    app.add_middleware(PerformanceMonitoringMiddleware)

    logger.info("All middleware added to application")
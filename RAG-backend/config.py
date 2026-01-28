import os
from dotenv import load_dotenv
import re

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration class to handle environment variables and settings.

    NOTE: Do not raise on import. Call `Config.validate_config()` at runtime
    when you need to enforce required variables. This prevents import-time
    crashes when running tools that only need a subset of configuration.
    """

    # Cohere API configuration
    COHERE_API_KEY = os.getenv('COHERE_API_KEY')

    # Qdrant configuration
    QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
    QDRANT_URL = os.getenv('QDRANT_URL')

    # Vercel deployment URL (optional but if present, should be valid)
    DEPLOY_VERCEL_URL = os.getenv('DEPLOY_VERCEL_URL')

    # Application settings
    CHUNK_SIZE = int(os.getenv('CHUNK_SIZE', '1000'))
    OVERLAP_SIZE = int(os.getenv('OVERLAP_SIZE', '200'))
    EMBEDDING_MODEL = os.getenv('EMBEDDING_MODEL', 'embed-multilingual-v3.0')
    QDRANT_COLLECTION = os.getenv('QDRANT_COLLECTION', 'physical_ai_content')

    # Rate limiting
    COHERE_RPM_LIMIT = int(os.getenv('COHERE_RPM_LIMIT', '10'))  # Requests per minute
    RATE_LIMIT_DELAY = float(os.getenv('RATE_LIMIT_DELAY', '6.0'))  # Delay in seconds

    @classmethod
    def validate_config(cls):
        """Validate that all required configuration values are present and valid."""
        errors = []

        if not cls.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is missing")

        if not cls.QDRANT_API_KEY:
            errors.append("QDRANT_API_KEY is missing")

        if not cls.QDRANT_URL:
            errors.append("QDRANT_URL is missing")
        else:
            # Validate QDRANT_URL format
            if not cls.is_valid_url(cls.QDRANT_URL):
                errors.append("QDRANT_URL is not a valid URL format")

        if cls.DEPLOY_VERCEL_URL:
            # Validate DEPLOY_VERCEL_URL format if provided
            if not cls.is_valid_url(cls.DEPLOY_VERCEL_URL):
                errors.append("DEPLOY_VERCEL_URL is not a valid URL format")

        if cls.CHUNK_SIZE <= 0:
            errors.append("CHUNK_SIZE must be positive")
        elif cls.CHUNK_SIZE > 2000:  # As per data model constraint
            errors.append("CHUNK_SIZE must be 2000 or less to prevent buffer overflows")

        if cls.OVERLAP_SIZE < 0:
            errors.append("OVERLAP_SIZE must be non-negative")

        if cls.OVERLAP_SIZE >= cls.CHUNK_SIZE:
            errors.append("OVERLAP_SIZE must be less than CHUNK_SIZE")

        if cls.COHERE_RPM_LIMIT <= 0:
            errors.append("COHERE_RPM_LIMIT must be positive")

        if cls.RATE_LIMIT_DELAY <= 0:
            errors.append("RATE_LIMIT_DELAY must be positive")

        if errors:
            raise ValueError(f"Configuration validation failed: {'; '.join(errors)}")

    @staticmethod
    def is_valid_url(url: str) -> bool:
        """
        Validate if a string is a valid URL format.

        Args:
            url: The URL string to validate

        Returns:
            bool: True if the URL is valid, False otherwise
        """
        if not url:
            return False

        # Basic URL regex pattern
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)

        return url_pattern.match(url) is not None

    @classmethod
    def get_config_summary(cls) -> dict:
        """
        Get a summary of the current configuration for logging/debugging purposes.

        Returns:
            dict: A dictionary with configuration summary
        """
        return {
            'COHERE_API_KEY': '***MASKED***' if cls.COHERE_API_KEY else 'NOT_SET',
            'QDRANT_URL': cls.QDRANT_URL if cls.QDRANT_URL else 'NOT_SET',
            'QDRANT_API_KEY': '***MASKED***' if cls.QDRANT_API_KEY else 'NOT_SET',
            'DEPLOY_VERCEL_URL': cls.DEPLOY_VERCEL_URL if cls.DEPLOY_VERCEL_URL else 'NOT_SET',
            'CHUNK_SIZE': cls.CHUNK_SIZE,
            'OVERLAP_SIZE': cls.OVERLAP_SIZE,
            'EMBEDDING_MODEL': cls.EMBEDDING_MODEL,
            'QDRANT_COLLECTION': cls.QDRANT_COLLECTION,
            'COHERE_RPM_LIMIT': cls.COHERE_RPM_LIMIT,
            'RATE_LIMIT_DELAY': cls.RATE_LIMIT_DELAY
        }
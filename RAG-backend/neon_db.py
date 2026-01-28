"""
Neon Database Integration for Chat History
Handles storing and retrieving chat history using Neon PostgreSQL database
"""

import os
import asyncpg
import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

class NeonHistoryManager:
    def __init__(self):
        # Parse the NEON_API_KEY to extract connection parameters
        # Assuming NEON_API_KEY is a connection string like: postgresql://username:password@host:port/database
        neon_connection_string = os.getenv("NEON_API_KEY", "")

        if neon_connection_string.startswith("postgresql://"):
            # Parse the connection string
            import re
            # Match postgresql://user:pass@host:port/database
            match = re.match(r"postgresql://([^:]+):([^@]+)@([^:]+):(\d+)/(.+)", neon_connection_string)
            if match:
                user, password, host, port, database = match.groups()
                self.connection_params = {
                    "host": host,
                    "port": int(port),
                    "database": database,
                    "user": user,
                    "password": password,
                    "ssl": "require"
                }
            else:
                # If parsing fails, set up empty params to fall back to file-based storage
                self.connection_params = {
                    "host": "",
                    "database": "",
                    "user": "",
                    "password": "",
                    "ssl": "require"
                }
        else:
            # If NEON_API_KEY is not a connection string, fall back to individual environment variables
            self.connection_params = {
                "host": os.getenv("NEON_DB_HOST", ""),
                "database": os.getenv("NEON_DB_NAME", ""),
                "user": os.getenv("NEON_DB_USER", ""),
                "password": os.getenv("NEON_API_KEY", ""),  # Using NEON_API_KEY as password
                "ssl": "require"
            }
        self.pool = None

    async def initialize(self):
        """Initialize the connection pool"""
        # Check if we have valid connection parameters
        if not self.connection_params.get("host") or not self.connection_params.get("database") or not self.connection_params.get("user") or not self.connection_params.get("password"):
            logger.warning("Neon database connection parameters not properly configured, initializing with fallback storage")
            self.pool = None
            return

        try:
            self.pool = await asyncpg.create_pool(**self.connection_params)

            # Create the history table if it doesn't exist
            async with self.pool.acquire() as conn:
                await conn.execute("""
                    CREATE TABLE IF NOT EXISTS chat_history (
                        id SERIAL PRIMARY KEY,
                        user_id VARCHAR(255) NOT NULL DEFAULT 'anonymous',
                        query TEXT NOT NULL,
                        response TEXT NOT NULL,
                        sources JSONB,
                        timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                        session_id VARCHAR(255)
                    );
                """)

            logger.info("Neon database connection initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize Neon database: {str(e)}")
            # Fall back to in-memory storage or file-based storage
            self.pool = None

    async def save_chat_interaction(self, query: str, response: str, user_id: str = "anonymous",
                                   sources: Optional[List[Dict[str, Any]]] = None, session_id: Optional[str] = None):
        """Save a chat interaction to the Neon database"""
        try:
            if not self.pool:
                logger.warning("Neon pool not initialized, skipping save")
                return

            async with self.pool.acquire() as conn:
                await conn.execute(
                    """
                    INSERT INTO chat_history (user_id, query, response, sources, session_id)
                    VALUES ($1, $2, $3, $4, $5)
                    """,
                    user_id, query, response, sources, session_id
                )

            logger.info(f"Saved chat interaction for user {user_id}")
        except Exception as e:
            logger.error(f"Failed to save chat interaction: {str(e)}")

    async def get_user_history(self, user_id: str, limit: int = 50) -> List[Dict[str, Any]]:
        """Retrieve chat history for a specific user"""
        try:
            if not self.pool:
                logger.warning("Neon pool not initialized, returning empty history")
                return []

            async with self.pool.acquire() as conn:
                rows = await conn.fetch(
                    """
                    SELECT query, response, sources, timestamp
                    FROM chat_history
                    WHERE user_id = $1
                    ORDER BY timestamp DESC
                    LIMIT $2
                    """,
                    user_id, limit
                )

            history = []
            for row in rows:
                history.append({
                    "query": row["query"],
                    "response": row["response"],
                    "sources": row["sources"],
                    "timestamp": row["timestamp"]
                })

            logger.info(f"Retrieved {len(history)} chat interactions for user {user_id}")
            return history
        except Exception as e:
            logger.error(f"Failed to retrieve user history: {str(e)}")
            return []

# Global instance
neon_manager = NeonHistoryManager()
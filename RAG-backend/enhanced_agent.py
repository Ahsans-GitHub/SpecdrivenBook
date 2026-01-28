"""
Enhanced RAG Agent Implementation
This module implements an enhanced version of the RAG agent with improved
capabilities for selected text context, multi-turn conversations, and metadata handling.
"""

import asyncio
import time
import logging
import os
from typing import Optional, List, Dict, Any, Tuple
from dataclasses import dataclass, field
from datetime import datetime
from pydantic import BaseModel, Field

from openai import OpenAI, AsyncOpenAI
from config import Config
from retrieval import RAGRetrievalSystem
from models import RetrievedChunk, ChatRequest, ChatResponse
from logging_config import rag_logger


logger = logging.getLogger(__name__)


class EnhancedRAGAgent:
    """
    Enhanced RAG Agent with improved capabilities for context handling and conversation management.
    """

    def __init__(self):
        """Initialize the enhanced RAG agent with required components."""
        # Initialize configuration
        self.config = Config

        # Initialize the retrieval system
        self.retriever = RAGRetrievalSystem()

        # Initialize the LLM client
        api_key = os.getenv("OPENAI_API_KEY") or os.getenv("DEEPSEEK_API_KEY") or os.getenv("OPENROUTER_API_KEY")
        if not api_key:
            raise ValueError("No API key found. Please set OPENAI_API_KEY, DEEPSEEK_API_KEY, or OPENROUTER_API_KEY environment variable.")

        # Use OpenRouter API with specified model
        self.client = AsyncOpenAI(
            api_key=api_key,
            base_url="https://openrouter.ai/api/v1"
        )

        # Model to use
        self.model_name = "deepseek/deepseek-r1-distill-llama-70b"

        # Conversation history storage (in-memory for now, can be extended to use Neon Postgres)
        self.conversation_history = {}

        logger.info(f"Enhanced RAG Agent initialized with model: {self.model_name}")

    async def process_query(
        self,
        query: str,
        session_id: Optional[str] = None,
        selected_text: str = "",
        top_k: int = 3,
        min_similarity: float = 0.4,
        temperature: float = 0.7
    ) -> Dict[str, Any]:
        """
        Process a user query through the enhanced RAG pipeline.

        Args:
            query: User's query
            session_id: Optional session identifier for conversation history
            selected_text: Optional text selected by user to bias retrieval
            top_k: Number of top results to retrieve
            min_similarity: Minimum similarity threshold for retrieval
            temperature: Temperature for generation

        Returns:
            Dictionary containing response, sources, session info, and metadata
        """
        start_time = time.time()
        logger.info(f"Processing query: '{query[:50]}...', session_id={session_id}, selected_text_len={len(selected_text)}")

        try:
            # Sanitize inputs
            sanitized_query = self._sanitize_input(query)
            sanitized_selected_text = self._sanitize_input(selected_text)

            # Validate input lengths
            if len(sanitized_query) > 2000:
                logger.warning(f"Query too long: {len(sanitized_query)} characters")
                raise ValueError("Query too long. Maximum 2000 characters allowed.")

            if len(sanitized_selected_text) > 2000:
                logger.warning(f"Selected text too long: {len(sanitized_selected_text)} characters")
                raise ValueError("Selected text too long. Maximum 2000 characters allowed.")

            # Validate parameter ranges
            if top_k < 1 or top_k > 20:
                logger.warning(f"Invalid top_k value: {top_k}")
                raise ValueError("top_k must be between 1 and 20")

            if min_similarity < 0.0 or min_similarity > 1.0:
                logger.warning(f"Invalid min_similarity value: {min_similarity}")
                raise ValueError("min_similarity must be between 0.0 and 1.0")

            if temperature < 0.0 or temperature > 2.0:
                logger.warning(f"Invalid temperature value: {temperature}")
                raise ValueError("temperature must be between 0.0 and 2.0")

            # Get conversation history if session_id is provided
            conversation_context = ""
            if session_id:
                conversation_context = self._get_conversation_context(session_id)

            # Create enhanced search query with conversation context and selected text
            enhanced_query = self._enhance_query(sanitized_query, conversation_context, sanitized_selected_text)

            # Step 1: Retrieve relevant content with biasing
            retrieval_start = time.time()

            # Perform retrieval with biasing if selected text is provided
            retrieved_results = await self.retriever.retrieve_with_biasing(
                query=enhanced_query,
                selected_text=sanitized_selected_text,
                top_k=top_k,
                min_similarity=min_similarity
            )

            retrieval_time = time.time() - retrieval_start

            # Convert to RetrievedChunk objects
            retrieved_chunks = []
            for result in retrieved_results:
                chunk = RetrievedChunk(
                    id=result["id"],
                    title=result["title"],
                    url=result["url"],
                    content=result["content"],
                    section=result["section"],
                    tags=result["tags"],
                    score=result["score"],
                    similarity=result["similarity"]
                )
                retrieved_chunks.append(chunk)

            # Step 2: Format context from retrieved content and conversation history
            context = self._format_context_for_llm(retrieved_chunks, conversation_context, sanitized_selected_text)

            # Step 3: Generate response using LLM
            generation_start = time.time()

            # Create the full prompt with context
            full_prompt = self._create_enhanced_prompt(sanitized_query, context, sanitized_selected_text)

            # Generate response
            response = await self._generate_response(full_prompt, temperature)

            generation_time = time.time() - generation_start

            # Step 4: Update conversation history if session_id is provided
            if session_id:
                self._update_conversation_history(session_id, sanitized_query, response)

            # Step 5: Prepare response with enhanced metadata
            response_data = {
                "response": response,
                "sources": [chunk.dict() for chunk in retrieved_chunks],
                "session_id": session_id or f"session_{int(time.time())}",  # Generate if not provided
                "metadata": self._generate_enhanced_metadata(retrieved_chunks, sanitized_query, sanitized_selected_text),
                "retrieval_time": retrieval_time,
                "generation_time": generation_time
            }

            total_time = time.time() - start_time
            logger.info(f"Query processed successfully in {total_time:.2f}s, session_id={response_data['session_id']}")

            # Log response metrics
            logger.debug(f"Response metrics - sources_count={len(retrieved_chunks)}, retrieval_time={retrieval_time:.2f}s, generation_time={generation_time:.2f}s")

            return response_data

        except ValueError as ve:
            # Handle validation errors
            logger.error(f"Validation error in agent: {str(ve)}")
            raise
        except Exception as e:
            # Handle other errors
            logger.error(f"Error in agent: {str(e)}", exc_info=True)
            raise

    def _sanitize_input(self, text: str) -> str:
        """
        Sanitize input text by removing potentially harmful content.

        Args:
            text: Input text to sanitize

        Returns:
            Sanitized text
        """
        if not text:
            return text

        import html
        import re

        # Remove HTML tags
        text = re.sub(r'<[^>]+>', '', text)

        # Unescape HTML entities
        text = html.unescape(text)

        # Remove potentially dangerous characters/sequences
        dangerous_patterns = [
            r'javascript:',  # JavaScript URLs
            r'on\w+\s*=',    # Event handlers
            r'data:',        # Data URLs
            r'vbscript:',    # VBScript
            r'expression\(', # CSS expressions
        ]

        for pattern in dangerous_patterns:
            text = re.sub(pattern, '', text, flags=re.IGNORECASE)

        return text.strip()

    def _get_conversation_context(self, session_id: str) -> str:
        """
        Get conversation context for the given session.

        Args:
            session_id: Session identifier

        Returns:
            Formatted conversation context string
        """
        if session_id not in self.conversation_history:
            return ""

        history = self.conversation_history[session_id][-5:]  # Get last 5 exchanges
        context_parts = ["Previous conversation context:"]

        for i, (query, response) in enumerate(history):
            context_parts.append(f"\n{i+1}. User: {query[:200]}{'...' if len(query) > 200 else ''}")
            context_parts.append(f"   Assistant: {response[:300]}{'...' if len(response) > 300 else ''}")

        return "\n".join(context_parts)

    def _enhance_query(self, query: str, conversation_context: str, selected_text: str) -> str:
        """
        Enhance the query with conversation context and selected text.

        Args:
            query: Original query
            conversation_context: Previous conversation context
            selected_text: Selected text to bias retrieval

        Returns:
            Enhanced query string
        """
        enhanced_parts = [query]

        if selected_text:
            enhanced_parts.append(f"Context from selected text: {selected_text[:500]}")

        if conversation_context:
            enhanced_parts.append(f"Previous context: {conversation_context[-500:]}")  # Limit length

        return " ".join(enhanced_parts)

    def _format_context_for_llm(self, retrieved_chunks: List[RetrievedChunk], conversation_context: str = "", selected_text: str = "") -> str:
        """
        Format retrieved content into a context string for the LLM with conversation and selected text context.

        Args:
            retrieved_chunks: List of retrieved content chunks
            conversation_context: Previous conversation context
            selected_text: Selected text for additional context

        Returns:
            Formatted context string
        """
        if not retrieved_chunks:
            context_parts = ["No relevant content found in the Physical AI textbook. Please try rephrasing your question or consult the textbook directly."]
        else:
            context_parts = [f"Found {len(retrieved_chunks)} relevant sections from the Physical AI textbook:\n"]

            for i, chunk in enumerate(retrieved_chunks, 1):
                context_parts.append(f"\n{i}. Source: {chunk.title or 'Untitled'}")
                context_parts.append(f"   URL: {chunk.url}")
                context_parts.append(f"   Section: {chunk.section}")
                context_parts.append(f"   Content Preview: {chunk.content[:500]}{'...' if len(chunk.content) > 500 else ''}")

                if chunk.tags:
                    context_parts.append(f"   Tags: {', '.join(chunk.tags)}")
                context_parts.append(f"   Similarity: {chunk.similarity:.3f}\n")

        # Add selected text context if provided
        if selected_text:
            context_parts.insert(0, f"USER SELECTED TEXT FOR CONTEXT:\n{selected_text}\n")
            context_parts.insert(1, "RELATED CONTENT FROM TEXTBOOK:\n")

        # Add conversation context if provided
        if conversation_context:
            context_parts.insert(0 if selected_text else 1, f"\n{conversation_context}\n")

        return "\n".join(context_parts)

    def _create_enhanced_prompt(self, query: str, context: str, selected_text: str = "") -> str:
        """
        Create an enhanced prompt for the LLM with context and selected text awareness.

        Args:
            query: User's query
            context: Formatted context from retrieved content
            selected_text: Selected text for additional context

        Returns:
            Complete enhanced prompt string
        """
        prompt_parts = [
            "You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook. Your purpose is to help students understand complex concepts in physical AI, robotics, and humanoid robotics by providing clear, accurate, and contextual answers based on the textbook content.",
            "",
            "TEXTBOOK CONTENT CONTEXT:",
            context,
            ""
        ]

        if selected_text:
            prompt_parts.extend([
                f"USER SELECTED TEXT CONTEXT (use this to bias your response):",
                selected_text[:1000],  # Limit selected text in prompt
                ""
            ])

        prompt_parts.extend([
            f"USER QUERY:",
            query,
            "",
            "INSTRUCTIONS:",
            "1. Provide a comprehensive, accurate answer based on the textbook content provided in the context.",
            "2. Always cite specific sources from the context (mention titles, sections, or URLs when possible).",
            "3. If the textbook content doesn't fully address the query, acknowledge this limitation and suggest related concepts that might be helpful.",
            "4. Use clear, educational language appropriate for university-level students.",
            "5. When relevant, provide examples or analogies to help explain complex concepts.",
            "6. If the user's selected text is mentioned in the context, specifically address how it relates to their query.",
            "7. If the user is continuing a conversation, make sure your response connects logically with the previous context.",
            "",
            "RESPONSE:"
        ])

        return "\n".join(prompt_parts)

    async def _generate_response(self, prompt: str, temperature: float = 0.7) -> str:
        """
        Generate a response using the LLM.

        Args:
            prompt: Complete prompt string
            temperature: Temperature for generation

        Returns:
            Generated response string
        """
        try:
            response = await self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "user", "content": prompt}
                ],
                temperature=temperature,
                max_tokens=2000  # Adjust based on needs
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return f"I encountered an error while generating a response. Please try rephrasing your question. Error details: {str(e)}"

    def _generate_enhanced_metadata(self, retrieved_chunks: List[RetrievedChunk], query: str, selected_text: str) -> Dict[str, Any]:
        """
        Generate enhanced metadata for the response.

        Args:
            retrieved_chunks: List of retrieved content chunks
            query: Original user query
            selected_text: Selected text used for biasing

        Returns:
            Dictionary with enhanced metadata
        """
        # Calculate confidence score based on retrieval quality
        confidence_score = self._calculate_confidence_score(retrieved_chunks)

        # Generate adaptive prompts based on query and retrieved content
        adaptive_prompts = self._generate_adaptive_prompts(query, retrieved_chunks)

        # Identify visualization opportunities
        visualization_opportunities = self._identify_visualization_opportunities(retrieved_chunks)

        # Identify related topics
        related_topics = self._extract_related_topics(retrieved_chunks)

        return {
            "adaptive_prompt_hint": f"Based on your query about '{query[:30]}...', here are relevant concepts...",
            "confidence_score": confidence_score,
            "retrieval_success": len(retrieved_chunks) > 0,
            "adaptive_prompts": adaptive_prompts,
            "ui_enhancement_metadata": {
                "has_visualization_opportunities": len(visualization_opportunities) > 0,
                "suggest_follow_up_questions": True,
                "suggest_related_topics": related_topics,
                "suggest_content_format": self._suggest_content_format(retrieved_chunks),
                "visualization_suggestions": visualization_opportunities,
                "has_selected_text_context": bool(selected_text),
                "selected_text_length": len(selected_text) if selected_text else 0
            },
            "analytics_metadata": {
                "query_length": len(query),
                "selected_text_used": bool(selected_text),
                "retrieved_chunks_count": len(retrieved_chunks),
                "avg_similarity_score": sum(c.similarity for c in retrieved_chunks) / len(retrieved_chunks) if retrieved_chunks else 0.0
            }
        }

    def _calculate_confidence_score(self, retrieved_chunks: List[RetrievedChunk]) -> float:
        """
        Calculate a confidence score based on the retrieved chunks.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            Confidence score between 0.0 and 1.0
        """
        if not retrieved_chunks:
            return 0.1  # Low confidence if no content retrieved

        # Calculate average similarity score
        avg_similarity = sum(chunk.similarity for chunk in retrieved_chunks) / len(retrieved_chunks)

        # Boost slightly if we have multiple high-quality results
        quality_multiplier = min(1.0, len(retrieved_chunks) * 0.3)  # Up to 3x boost for multiple results
        confidence = min(1.0, avg_similarity * (1 + quality_multiplier))

        return confidence

    def _generate_adaptive_prompts(self, query: str, retrieved_chunks: List[RetrievedChunk]) -> List[str]:
        """
        Generate adaptive prompts based on the query and retrieved content.

        Args:
            query: Original user query
            retrieved_chunks: List of retrieved content chunks

        Returns:
            List of suggested follow-up prompts
        """
        if not retrieved_chunks:
            return [
                f"Can you explain more about {query}?",
                "What are the practical applications of this concept?",
                "Can you provide an example of this?"
            ]

        # Extract key terms from the retrieved content
        content_text = " ".join([chunk.content for chunk in retrieved_chunks])
        import re
        words = re.findall(r'\b\w+\b', content_text.lower())
        technical_terms = [word for word in words if len(word) > 5 and word.isalpha() and word not in ['which', 'their', 'about', 'other', 'these', 'also', 'when', 'where', 'would', 'could', 'should', 'might', 'have', 'been', 'from', 'with', 'this', 'that', 'than', 'will', 'more', 'most', 'some', 'such', 'into', 'only', 'upon', 'after', 'before', 'since', 'until', 'while', 'both', 'each', 'few', 'more', 'some', 'such', 'than', 'either', 'neither', 'only', 'own', 'same', 'than', 'very']]

        prompts = []

        # Generate follow-up suggestions based on content
        if any(term in content_text.lower() for term in ['algorithm', 'method', 'approach', 'technique']):
            term = technical_terms[0] if technical_terms else 'approach'
            prompts.extend([
                f"How does the {term} work in practice?",
                "What are the limitations of this approach?"
            ])

        if any(term in content_text.lower() for term in ['implementation', 'apply', 'use', 'practice']):
            prompts.extend([
                "What are the practical considerations for implementing this?",
                "Are there any code examples available?"
            ])

        if any(term in content_text.lower() for term in ['advantage', 'benefit', 'better', 'improve']):
            prompts.extend([
                "What are the potential drawbacks of this approach?",
                "How does this compare to alternative methods?"
            ])

        if any(term in content_text.lower() for term in ['example', 'case', 'scenario', 'application']):
            prompts.extend([
                "Can you provide more examples of this concept?",
                "What are some real-world applications?"
            ])

        # Add generic follow-ups if not enough specific ones
        if len(prompts) < 2:
            prompts.extend([
                "What are some practical applications of this?",
                "Are there any limitations to consider?",
                "Where can I find more information about this topic?"
            ])

        # Limit to 5 suggestions and ensure they're unique
        unique_prompts = list(dict.fromkeys(prompts))  # Remove duplicates while preserving order
        return unique_prompts[:5]

    def _identify_visualization_opportunities(self, retrieved_chunks: List[RetrievedChunk]) -> List[str]:
        """
        Identify opportunities for visualizations based on content.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            List of visualization opportunity types
        """
        content_text = " ".join([chunk.content for chunk in retrieved_chunks]).lower()

        visualization_types = []
        if any(keyword in content_text for keyword in ['diagram', 'figure', 'graph', 'chart', 'plot', 'visual']):
            visualization_types.append('diagram_or_figure')
        if any(keyword in content_text for keyword in ['algorithm', 'process', 'workflow', 'steps', 'procedure']):
            visualization_types.append('flowchart')
        if any(keyword in content_text for keyword in ['comparison', 'vs', 'versus', 'difference', 'alternative']):
            visualization_types.append('comparison_chart')
        if any(keyword in content_text for keyword in ['data', 'metric', 'measurement', 'performance', 'results']):
            visualization_types.append('data_visualization')
        if any(keyword in content_text for keyword in ['structure', 'architecture', 'component', 'system']):
            visualization_types.append('system_diagram')
        if any(keyword in content_text for keyword in ['equation', 'formula', 'math', 'derivation']):
            visualization_types.append('mathematical_representation')

        return visualization_types

    def _extract_related_topics(self, retrieved_chunks: List[RetrievedChunk]) -> List[str]:
        """
        Extract related topics from the retrieved content.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            List of related topics
        """
        content_text = " ".join([chunk.content for chunk in retrieved_chunks]).lower()

        # Define common robotics and AI topics
        topics = []
        topic_keywords = {
            'kinematics': ['kinematics', 'position', 'orientation', 'motion', 'joints', 'links'],
            'dynamics': ['dynamics', 'force', 'torque', 'acceleration', 'inertia', 'mass'],
            'control': ['control', 'feedback', 'stability', 'tracking', 'pid', 'regulator'],
            'perception': ['perception', 'vision', 'sensor', 'detection', 'recognition', 'camera'],
            'planning': ['planning', 'path', 'trajectory', 'navigation', 'motion planning', 'route'],
            'learning': ['learning', 'adaptation', 'training', 'optimization', 'reinforcement', 'supervised'],
            'humanoids': ['humanoid', 'bipedal', 'walking', 'balance', 'locomotion', 'gait'],
            'ai': ['intelligence', 'reasoning', 'decision', 'cognition', 'neural', 'network'],
            'ethics': ['ethics', 'safety', 'responsibility', 'bias', 'fairness', 'trustworthy']
        }

        for topic, keywords in topic_keywords.items():
            if any(keyword in content_text for keyword in keywords):
                topics.append(topic)

        return topics[:3]  # Return top 3 related topics

    def _suggest_content_format(self, retrieved_chunks: List[RetrievedChunk]) -> str:
        """
        Suggest an appropriate content format based on retrieved content.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            Suggested content format
        """
        content_text = " ".join([chunk.content for chunk in retrieved_chunks]).lower()

        if any(keyword in content_text for keyword in ['algorithm', 'pseudocode', 'steps', 'procedure']):
            return 'algorithm_with_steps'
        elif any(keyword in content_text for keyword in ['example', 'case study', 'demonstration', 'illustration']):
            return 'example_based'
        elif any(keyword in content_text for keyword in ['equation', 'formula', 'math', 'derivation']):
            return 'mathematical_with_derivations'
        elif any(keyword in content_text for keyword in ['concept', 'theory', 'principle', 'foundation']):
            return 'conceptual_explanation'
        else:
            return 'text_with_examples'

    def _update_conversation_history(self, session_id: str, query: str, response: str):
        """
        Update conversation history for the given session.

        Args:
            session_id: Session identifier
            query: User's query
            response: Agent's response
        """
        if session_id not in self.conversation_history:
            self.conversation_history[session_id] = []

        # Add the new exchange to history
        self.conversation_history[session_id].append((query, response))

        # Keep only the last 10 exchanges to prevent memory growth
        if len(self.conversation_history[session_id]) > 10:
            self.conversation_history[session_id] = self.conversation_history[session_id][-10:]
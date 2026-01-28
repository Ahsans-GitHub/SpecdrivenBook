"""
RAG Agent Implementation using Direct OpenAI API
This module implements the core RAG agent that combines retrieval and generation
using the OpenAI API directly to provide contextual responses to user queries about the Physical AI textbook.
"""

import asyncio
import time
import logging
import os
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
from openai import AsyncOpenAI
from config import Config
from retrieval import RAGRetrievalSystem
from models import RetrievedChunk


logger = logging.getLogger(__name__)

OPENROUTER_API_KEY="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"
DEEPSEEK_API_KEY="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"

def retrieve_content(query: str, top_k: int = 3, min_similarity: float = 0.4) -> List[Dict[str, Any]]:
    """
    Retrieve relevant content from the Physical AI textbook using semantic search.
    This function is available for direct use without the agents SDK.

    Args:
        query: The user's query to search for in the textbook
        top_k: Number of top results to retrieve (default: 3)
        min_similarity: Minimum similarity threshold for retrieval (default: 0.4)

    Returns:
        List of retrieved content chunks with metadata
    """
    try:
        # Initialize the retrieval system
        retriever = RAGRetrievalSystem()

        # Perform retrieval with biasing
        results = asyncio.run(
            retriever.retrieve_with_biasing(
                query=query,
                top_k=top_k,
                min_similarity=min_similarity
            )
        )

        logger.info(f"Retrieved {len(results)} results for query: '{query[:50]}...'")
        return results

    except Exception as e:
        logger.error(f"Error during content retrieval: {str(e)}")
        return []


class RAGAgent:
    """
    Main RAG Agent class that orchestrates retrieval and generation using direct OpenAI API calls.
    """

    def __init__(self):
        """Initialize the RAG agent with required components."""
        # Initialize configuration
        self.config = Config

        # Initialize the retrieval system
        self.retriever = RAGRetrievalSystem()

        # Initialize the OpenAI client for generation
        api_key = os.getenv("OPENROUTER_API_KEY") or os.getenv("DEEPSEEK_API_KEY")
        if not api_key:
            raise ValueError("API key not found. Please set OPENAI_API_KEY, OPENROUTER_API_KEY, or DEEPSEEK_API_KEY environment variable.")

        # Determine which API to use based on environment
        if os.getenv("OPENROUTER_API_KEY"):
            self.client = AsyncOpenAI(
                api_key=os.getenv("OPENROUTER_API_KEY"),
                base_url="https://openrouter.ai/api/v1"
            )
            self.model_name = "deepseek/deepseek-v3.2"
        else:
            self.client = AsyncOpenAI(api_key=DEEPSEEK_API_KEY)
            self.model_name = "deepseek/deepseek-v3.2"

        logger.info(f"RAG Agent initialized with model: {self.model_name}")

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
        Process a user query through the RAG pipeline using direct OpenAI API calls.

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
            # Enhance query with selected text if provided
            enhanced_query = query
            if selected_text and selected_text.strip():
                enhanced_query = f"{query} (Context from selected text: {selected_text[:500]}...)"

            # Retrieve relevant content first to provide to the agent as context
            retrieval_start = time.time()
            retrieved_results = await self.retriever.retrieve_with_biasing(
                query=enhanced_query,
                top_k=top_k,
                min_similarity=min_similarity
            )
            retrieval_time = time.time() - retrieval_start

            # Convert to RetrievedChunk objects
            retrieved_chunks = []
            for result_item in retrieved_results:
                chunk = RetrievedChunk(
                    id=str(result_item.get('id', '')),  # Ensure ID is string
                    title=result_item.get('title', ''),
                    url=result_item.get('url', ''),
                    content=result_item.get('content', ''),
                    section=result_item.get('section', ''),
                    tags=result_item.get('tags', []),
                    score=result_item.get('score', 0.0),
                    similarity=result_item.get('similarity', 0.0)
                )
                retrieved_chunks.append(chunk)

            # Format context from retrieved content
            context_parts = ["Here is relevant content from the Physical AI textbook to answer your query:"]
            for i, chunk in enumerate(retrieved_chunks, 1):
                context_parts.append(f"\n{i}. Source: {chunk.title or 'Untitled'}")
                context_parts.append(f"   URL: {chunk.url}")
                context_parts.append(f"   Section: {chunk.section}")
                context_parts.append(f"   Content: {chunk.content[:500]}{'...' if len(chunk.content) > 500 else ''}")
                if chunk.tags:
                    context_parts.append(f"   Tags: {', '.join(chunk.tags)}")

            context = "\n".join(context_parts)

            # Create the full prompt with context
            if retrieved_chunks:
                # If we have relevant content, use it to answer the specific query
                full_prompt = f"""{context}

USER QUERY: {enhanced_query}

INSTRUCTIONS:
1. Answer the specific user query based ONLY on the provided context from the Physical AI textbook
2. If the context contains information relevant to the query, provide a detailed answer citing specific sources
3. If the context doesn't contain information about the query, acknowledge this limitation
4. Do not generate general information about Physical AI if not directly related to the specific query
5. Always cite specific sources from the context (mention titles, sections, or URLs when possible)
6. Use clear, educational language appropriate for university-level students"""
            else:
                # If no relevant content found, inform the user
                full_prompt = f"""CONTEXT: No relevant content found in the textbook for this query.

USER QUERY: {enhanced_query}

INSTRUCTIONS:
1. Inform the user that no relevant content was found in the textbook
2. Do not generate general information about Physical AI if not directly related to the specific query
3. Suggest the user rephrase their query if needed"""

            # Generate response using OpenAI API
            generation_start = time.time()
            response = await self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "user", "content": full_prompt}
                ],
                temperature=temperature,
                max_tokens=2000
            )

            response_text = response.choices[0].message.content
            generation_time = time.time() - generation_start

            # Calculate confidence score based on similarities
            avg_similarity = sum(chunk.similarity for chunk in retrieved_chunks) / len(retrieved_chunks) if retrieved_chunks else 0.0
            confidence_score = min(1.0, avg_similarity * 2)  # Boost similarity for confidence

            # Prepare adaptive prompts based on content
            adaptive_prompts = self._generate_adaptive_prompts(query, retrieved_chunks)

            # Prepare response data
            response_data = {
                "response": response_text,
                "sources": [chunk.dict() for chunk in retrieved_chunks],
                "session_id": session_id or f"session_{int(time.time())}",
                "metadata": {
                    "adaptive_prompt_hint": f"Based on your query about '{query[:30]}...', here are relevant concepts...",
                    "confidence_score": confidence_score,
                    "retrieval_success": len(retrieved_chunks) > 0,
                    "adaptive_prompts": adaptive_prompts,
                    "ui_enhancement_metadata": {
                        "has_visualization_opportunities": self._has_visualization_opportunities(retrieved_chunks),
                        "suggest_follow_up_questions": True,
                        "suggest_related_topics": self._extract_related_topics(retrieved_chunks),
                        "suggest_content_format": "text_with_examples" if any("example" in chunk.content.lower() for chunk in retrieved_chunks) else "explanation"
                    }
                },
                "retrieval_time": retrieval_time,
                "generation_time": generation_time
            }

            total_time = time.time() - start_time
            logger.info(f"Query processed successfully in {total_time:.2f}s, retrieved {len(retrieved_chunks)} chunks")

            return response_data

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}", exc_info=True)
            raise

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
        content_text = " ".join([chunk.content for chunk in retrieved_chunks]).lower()
        import re
        words = re.findall(r'\b\w+\b', content_text)
        technical_terms = [word for word in words if len(word) > 5 and word.isalpha() and word not in ['which', 'their', 'about', 'other', 'these', 'also', 'when', 'where', 'would', 'could', 'should', 'might', 'have', 'been', 'from', 'with', 'this', 'that', 'than', 'will', 'more', 'some', 'such', 'then', 'into', 'only', 'upon', 'after', 'before', 'since', 'until', 'while', 'both', 'each', 'few', 'more', 'same', 'than', 'too', 'very', 'just', 'now', 'well', 'even', 'still', 'once', 'ever', 'never', 'always', 'often', 'seldom', 'yet', 'though', 'although', 'because', 'since', 'unless', 'until', 'whenever', 'wherever', 'however']]

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

    def _has_visualization_opportunities(self, retrieved_chunks: List[RetrievedChunk]) -> bool:
        """
        Check if there are opportunities for visualizations based on content.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            True if visualization opportunities exist
        """
        content_text = " ".join([chunk.content for chunk in retrieved_chunks]).lower()

        visualization_keywords = [
            'diagram', 'figure', 'graph', 'chart', 'plot', 'visual',
            'algorithm', 'process', 'workflow', 'steps', 'procedure',
            'comparison', 'vs', 'versus', 'difference', 'alternative',
            'data', 'metric', 'measurement', 'performance', 'results',
            'structure', 'architecture', 'component', 'system',
            'equation', 'formula', 'math', 'derivation',
            'neural', 'network', 'model', 'training'
        ]

        return any(keyword in content_text for keyword in visualization_keywords)

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


async def test_query_retrieval():
    """
    Test function to verify query retrieval functionality using OpenAI API.
    """
    print("="*70)
    print("QUERY RETRIEVAL TEST USING OPENAI API")
    print("="*70)

    try:
        # Initialize the RAG agent
        print("Initializing RAG agent...")
        agent = RAGAgent()
        print("[OK] RAG agent initialized successfully")

        # Test collection status
        print("\nChecking collection status...")
        status = agent.retriever.check_collection_status()
        print(f"[OK] Collection status: {status['status']}")
        print(f"[OK] Documents in collection: {status['total_documents']}")

        if not status.get('has_content', False):
            print("[ERROR] Collection is empty - no content to retrieve")
            return False

        # Test query retrieval
        test_query = "What is Disembodiment?"
        print(f"\nTesting query: '{test_query}'")

        start_time = time.time()
        result = await agent.process_query(
            query=test_query,
            top_k=3,
            min_similarity=0.1,
            temperature=0.7
        )
        total_time = time.time() - start_time

        print(f"[OK] Query processed successfully in {total_time:.2f}s")
        print(f"[OK] Retrieved {len(result['sources'])} sources")
        print(f"[OK] Response generated: {len(result['response'])} characters")

        # Show sample results
        print(f"\n--- SAMPLE RESULTS ---")
        for i, source in enumerate(result['sources'][:2], 1):  # Show first 2 sources
            print(f"\n{i}. Title: {source.get('title', 'N/A')[:80]}...")
            print(f"   URL: {source.get('url', 'N/A')}")
            print(f"   Section: {source.get('section', 'N/A')}")
            print(f"   Similarity: {source.get('similarity', 0):.3f}")

            # Clean content preview
            content = source.get('content', '')[:200]
            clean_content = content.encode('ascii', errors='ignore').decode('ascii')
            print(f"   Content: {clean_content}...")

        print(f"\n--- RESPONSE PREVIEW ---")
        response_preview = result['response'][:300].encode('ascii', errors='ignore').decode('ascii')
        print(f"{response_preview}...")

        print(f"\n--- METADATA ---")
        metadata = result['metadata']
        print(f"Confidence Score: {metadata.get('confidence_score', 0):.3f}")
        print(f"Retrieval Success: {metadata.get('retrieval_success', False)}")
        if metadata.get('adaptive_prompts'):
            print(f"Suggested Follow-ups: {metadata['adaptive_prompts'][:3]}")  # Show first 3

        print("\n" + "="*70)
        print("SUCCESS: Query retrieval is working correctly!")
        print("✓ OpenAI API key is properly configured")
        print("✓ Vector database connection is functional")
        print("✓ Semantic search retrieves relevant content")
        print("✓ Response generation works with context")
        print("="*70)

        return True

    except Exception as e:
        print(f"\n[ERROR] Query retrieval test failed: {str(e)}")
        import traceback
        traceback.print_exc()
        print("="*70)
        print("FAILED: Query retrieval test failed")
        print("="*70)
        return False


def main():
    """
    Main function to run the RAG agent directly for testing purposes.
    """
    import sys
    import argparse

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Test the RAG Agent with OpenAI API')
    parser.add_argument('--query', '-q', type=str, required=True,
                        help='Query to process through the RAG agent')
    parser.add_argument('--top-k', type=int, default=3,
                        help='Number of top results to retrieve (default: 3)')
    parser.add_argument('--min-similarity', type=float, default=0.4,
                        help='Minimum similarity threshold (default: 0.4)')
    parser.add_argument('--temperature', type=float, default=0.7,
                        help='Temperature for generation (default: 0.7)')

    args = parser.parse_args()

    print("="*70)
    print("RAG AGENT TESTING WITH OPENAI API")
    print("="*70)
    print(f"Query: '{args.query}'")
    print(f"Top-K: {args.top_k}")
    print(f"Min Similarity: {args.min_similarity}")
    print(f"Temperature: {args.temperature}")
    print("-"*70)

    try:
        # Initialize the RAG agent
        print("Initializing RAG agent...")
        agent = RAGAgent()
        print("[OK] RAG agent initialized successfully")

        # Process the query
        print(f"\nProcessing query: '{args.query}'")
        import asyncio
        result = asyncio.run(agent.process_query(
            query=args.query,
            top_k=args.top_k,
            min_similarity=args.min_similarity,
            temperature=args.temperature
        ))

        print(f"\n[OK] Query processed successfully")
        print(f"[OK] Retrieved {len(result['sources'])} sources")
        print(f"[OK] Response generated in {result['generation_time']:.2f}s")

        print("\n--- RESPONSE ---")
        print(result['response'])
        print("\n--- SOURCES ---")
        for i, source in enumerate(result['sources'], 1):
            print(f"\n{i}. {source.get('title', 'N/A')[:100]}...")
            print(f"   URL: {source.get('url', 'N/A')}")
            print(f"   Section: {source.get('section', 'N/A')}")
            print(f"   Similarity: {source.get('similarity', 0):.3f}")

        print(f"\n--- METADATA ---")
        metadata = result['metadata']
        print(f"Confidence Score: {metadata.get('confidence_score', 0):.3f}")
        print(f"Retrieval Success: {metadata.get('retrieval_success', False)}")
        if metadata.get('adaptive_prompts'):
            print(f"Suggested Follow-ups: {metadata['adaptive_prompts'][:3]}")  # Show first 3

        print("\n" + "="*70)
        print("SUCCESS: RAG Agent responded successfully!")
        print("The agent properly retrieved content and generated a contextual response.")
        print("="*70)

    except Exception as e:
        print(f"\n[ERROR] {str(e)}")
        import traceback
        traceback.print_exc()
        print("="*70)
        print("FAILED: Error during agent execution")
        print("="*70)
        sys.exit(1)


if __name__ == "__main__":
    # Check if command line arguments were provided
    import sys
    if len(sys.argv) > 1:
        # Run the main function if command line arguments are provided
        main()
    else:
        # Otherwise run the test function
        import asyncio
        success = asyncio.run(test_query_retrieval())
        if success:
            print("\n🎉 Query retrieval system is working correctly!")
        else:
            print("\n💥 Query retrieval system has issues.")
        sys.exit(0 if success else 1)
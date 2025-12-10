import os
import google.generativeai as genai
from typing import List, Union
from dotenv import load_dotenv

load_dotenv()

class GoogleEmbeddingProvider:
    def __init__(self):
        self.api_key = os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        genai.configure(api_key=self.api_key)
        # Use the embedding model
        self.model = "models/embedding-001"  # Google's embedding model

    async def embed_text(self, texts: Union[str, List[str]]) -> List[List[float]]:
        """
        Generate embeddings for one or more texts using Google's embedding API
        """
        if isinstance(texts, str):
            texts = [texts]

        try:
            # Process each text individually as Google's API expects
            embeddings = []
            for text in texts:
                # Truncate text if too long (Gemini has limits)
                if len(text) > 10000:  # Conservative limit
                    text = text[:10000]

                # Use the embedding API
                result = genai.embed_content(
                    model=self.model,
                    content=text,
                    task_type="RETRIEVAL_DOCUMENT"  # or "RETRIEVAL_QUERY" for queries
                )
                embeddings.append(result['embedding'])

            return embeddings
        except Exception as e:
            print(f"Error generating embeddings: {e}")
            # Return mock embeddings in case of error
            return [[0.1] * 768 for _ in texts]

# Alternative implementation using a different embedding provider
class OpenAIEmbeddingProvider:
    def __init__(self):
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        import openai
        self.openai_client = openai.OpenAI(api_key=self.api_key)

    async def embed_text(self, texts: Union[str, List[str]]) -> List[List[float]]:
        """
        Generate embeddings using OpenAI API as an alternative
        """
        import asyncio
        if isinstance(texts, str):
            texts = [texts]

        # Process with OpenAI's async approach
        try:
            response = self.openai_client.embeddings.create(
                input=texts,
                model="text-embedding-ada-002"
            )
            embeddings = [item.embedding for item in response.data]
            return embeddings
        except Exception as e:
            print(f"Error generating OpenAI embeddings: {e}")
            return [[0.1] * 1536 for _ in texts]  # OpenAI embeddings are 1536-dim

# Universal embedding provider that can use different backends
class UniversalEmbeddingProvider:
    def __init__(self):
        # Check which provider to use
        if os.getenv("GEMINI_API_KEY"):
            self.provider = GoogleEmbeddingProvider()
        elif os.getenv("OPENAI_API_KEY"):
            self.provider = OpenAIEmbeddingProvider()
        else:
            raise ValueError("Either GEMINI_API_KEY or OPENAI_API_KEY must be set")

    async def embed_text(self, texts: Union[str, List[str]]) -> List[List[float]]:
        """
        Generate embeddings using the configured provider
        """
        return await self.provider.embed_text(texts)

# Singleton instance
embedding_provider = UniversalEmbeddingProvider()
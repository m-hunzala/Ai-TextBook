import google.generativeai as genai
from typing import List, Dict, Tuple, Optional, AsyncGenerator
import os
import re
import time
from dotenv import load_dotenv
from vector_store import QdrantVectorStore
from ingestion_pipeline import IngestionPipeline
from datastore import datastore

# Import subagents
from subagents import registry

load_dotenv()

class CodeExtractionTool:
    """
    Tool to extract code examples or YAML snippets from retrieved documents
    """
    @staticmethod
    def extract_code_snippets(text: str) -> List[Dict[str, str]]:
        """
        Extract code blocks from text with their language detection
        """
        code_blocks = []

        # Pattern to match code blocks with language specification
        pattern = r'```(\w*)\n(.*?)\n```'
        matches = re.finditer(pattern, text, re.DOTALL)

        for match in matches:
            language = match.group(1).strip() or 'text'
            code = match.group(2).strip()

            if code:  # Only add non-empty code blocks
                code_blocks.append({
                    'language': language,
                    'code': code,
                    'type': 'code_block'
                })

        # Pattern to match YAML blocks
        yaml_pattern = r'---\n(.*?\n)---'
        yaml_matches = re.finditer(yaml_pattern, text, re.DOTALL)

        for match in yaml_matches:
            yaml_content = match.group(1).strip()
            if yaml_content:
                code_blocks.append({
                    'language': 'yaml',
                    'code': yaml_content,
                    'type': 'yaml_block'
                })

        return code_blocks

class RAGSystem:
    def __init__(self):
        self.vector_store = QdrantVectorStore()
        self.ingestion_pipeline = IngestionPipeline()
        self.code_tool = CodeExtractionTool()
        self.subagent_registry = registry  # Add the subagent registry

        # Initialize Google Generative AI if API key is provided
        api_key = os.getenv("GEMINI_API_KEY")
        if api_key:
            genai.configure(api_key=api_key)

            # Get model configuration from environment variables
            model_name = os.getenv("GEMINI_MODEL_NAME", "gemini-pro")
            temperature = float(os.getenv("GEMINI_TEMPERATURE", "0.7"))
            max_output_tokens = int(os.getenv("GEMINI_MAX_TOKENS", "2048"))

            self.generative_model = genai.GenerativeModel(
                model_name=model_name,
                generation_config={
                    "temperature": temperature,
                    "max_output_tokens": max_output_tokens
                }
            )
        else:
            self.generative_model = None

    async def query(self, query: str, top_k: int = 5, context_ids: Optional[List[str]] = None) -> Tuple[List[Dict], str]:
        """
        Query the vector store to retrieve relevant passages
        """
        results = await self.vector_store.search(query, top_k, context_ids)

        # Extract the text from results
        passages = []
        for result in results:
            passages.append({
                'id': result['id'],
                'text': result['text'],
                'metadata': result['metadata'],
                'score': result['score']
            })

        # Combine all retrieved texts
        combined_context = "\n\n".join([p['text'] for p in passages])

        return passages, combined_context

    async def embed_and_upsert(self) -> int:
        """
        Run the embedding and upsert pipeline
        """
        return await self.ingestion_pipeline.run_ingestion_pipeline()

    async def answer(self, query: str, selected_text: Optional[str] = None, user_id: Optional[str] = None, session_id: Optional[str] = None, force_subagent: Optional[str] = None) -> Tuple[str, List[Dict], Dict]:
        """
        Generate an answer using RAG - either with selected text or full retrieval
        Returns: (answer, retrieved_context, trace)
        """
        # Start timing for response time measurement
        start_time = time.time()

        # Check if this query should be handled by a subagent
        subagent_name = force_subagent or self.detect_subagent_intent(query)

        if subagent_name:
            # Use the subagent for this query
            subagent_result = await self.call_subagent(subagent_name, query)

            # Log the query with response time
            response_time_ms = int((time.time() - start_time) * 1000)
            query_id = await datastore.log_query(query, user_id, session_id, response_time_ms)

            # Return subagent result
            answer = f"Expert assistance from {subagent_name}: {str(subagent_result)}"

            # Create trace for debugging
            trace = {
                'retrieved_docs': [],
                'model_response': answer,
                'code_snippets': [],
                'query': query,
                'context_used': 'subagent processed',
                'query_id': query_id,
                'subagent_used': subagent_name,
                'subagent_result': subagent_result
            }

            return answer, [], trace

        # If not using a subagent, proceed with regular RAG
        if selected_text:
            # If selected_text is provided, use it directly
            retrieved_context = [{
                'text': selected_text,
                'metadata': {'source': 'user_provided'},
                'score': 1.0
            }]
            context = selected_text
        else:
            # Otherwise, retrieve from the vector store
            passages, context = await self.query(query, top_k=5)
            retrieved_context = passages

        if not context.strip():
            # Log the query even if no context is found
            response_time_ms = int((time.time() - start_time) * 1000)
            query_id = await datastore.log_query(query, user_id, session_id, response_time_ms)

            return "No relevant context found to answer the question.", retrieved_context, {
                'retrieved_docs': [],
                'model_response': "No relevant context found to answer the question.",
                'code_snippets': [],
                'query_id': query_id
            }

        # Extract code snippets from the context
        code_snippets = self.code_tool.extract_code_snippets(context)

        # Generate answer using the LLM with enforced context
        if self.generative_model:
            try:
                # Create system prompt that enforces using only provided context
                system_prompt = f"""
                You are a helpful documentation assistant. Answer only using the provided context.
                If the answer is not in the provided context, respond with "I don't know — check these docs".

                Context: {context}

                Question: {query}

                Answer concisely and accurately based only on the context provided.
                If the context contains relevant code examples or configuration snippets,
                include them in your response when appropriate.
                """

                response = await self.generative_model.generate_content_async(
                    system_prompt,
                    stream=False  # For now, using non-streaming
                )

                answer = response.text if response.text else "I don't know — check these docs"
            except Exception as e:
                answer = f"Error generating answer: {str(e)}"
        else:
            # Fallback if no LLM is configured
            answer = f"Context: {context[:500]}... (truncated)\n\nQuestion: {query}\n\n[No LLM configured - unable to generate answer]"

        # Log the query with response time
        response_time_ms = int((time.time() - start_time) * 1000)
        query_id = await datastore.log_query(query, user_id, session_id, response_time_ms)

        # Create trace for debugging
        trace = {
            'retrieved_docs': retrieved_context,
            'model_response': answer,
            'code_snippets': code_snippets,
            'query': query,
            'context_used': context,
            'query_id': query_id
        }

        return answer, retrieved_context, trace

    async def answer_streaming(self, query: str, selected_text: Optional[str] = None, user_id: Optional[str] = None, session_id: Optional[str] = None, force_subagent: Optional[str] = None) -> AsyncGenerator[Dict, None]:
        """
        Generate an answer with streaming response
        Yields chunks of the response as they become available
        """
        start_time = time.time()

        # Check if this query should be handled by a subagent
        subagent_name = force_subagent or self.detect_subagent_intent(query)

        if subagent_name:
            # Use the subagent for this query
            subagent_result = await self.call_subagent(subagent_name, query)

            # Log the query with response time
            response_time_ms = int((time.time() - start_time) * 1000)
            query_id = await datastore.log_query(query, user_id, session_id, response_time_ms)

            # Yield subagent result
            answer = f"Expert assistance from {subagent_name}: {str(subagent_result)}"

            trace = {
                'retrieved_docs': [],
                'model_response': answer,
                'code_snippets': [],
                'query': query,
                'context_used': 'subagent processed',
                'query_id': query_id,
                'subagent_used': subagent_name,
                'subagent_result': subagent_result
            }

            yield {
                'type': 'complete',
                'answer': answer,
                'retrieved_context': [],
                'trace': trace
            }
            return

        # If not using a subagent, proceed with regular RAG
        if selected_text:
            retrieved_context = [{
                'text': selected_text,
                'metadata': {'source': 'user_provided'},
                'score': 1.0
            }]
            context = selected_text
        else:
            passages, context = await self.query(query, top_k=5)
            retrieved_context = passages

        if not context.strip():
            # Log the query even if no context is found
            response_time_ms = int((time.time() - start_time) * 1000)
            query_id = await datastore.log_query(query, user_id, session_id, response_time_ms)

            yield {
                'type': 'complete',
                'answer': "No relevant context found to answer the question.",
                'retrieved_context': retrieved_context,
                'trace': {
                    'retrieved_docs': [],
                    'model_response': "No relevant context found to answer the question.",
                    'code_snippets': [],
                    'query_id': query_id
                }
            }
            return

        # Extract code snippets from the context
        code_snippets = self.code_tool.extract_code_snippets(context)

        # Create system prompt that enforces using only provided context
        system_prompt = f"""
        You are a helpful documentation assistant. Answer only using the provided context.
        If the answer is not in the provided context, respond with "I don't know — check these docs".

        Context: {context}

        Question: {query}

        Answer concisely and accurately based only on the context provided.
        If the context contains relevant code examples or configuration snippets,
        include them in your response when appropriate.
        """

        if self.generative_model:
            try:
                # Use streaming generation
                response = await self.generative_model.generate_content_async(
                    system_prompt,
                    stream=True
                )

                # Log the query with response time
                response_time_ms = int((time.time() - start_time) * 1000)
                query_id = await datastore.log_query(query, user_id, session_id, response_time_ms)

                # Yield retrieved context first
                yield {
                    'type': 'context',
                    'retrieved_context': retrieved_context,
                    'code_snippets': code_snippets
                }

                # Stream the response
                full_answer = ""
                async for chunk in response:
                    text_chunk = chunk.text
                    if text_chunk:
                        full_answer += text_chunk
                        yield {
                            'type': 'stream',
                            'chunk': text_chunk,
                            'full_answer': full_answer
                        }

                # Send completion with trace
                trace = {
                    'retrieved_docs': retrieved_context,
                    'model_response': full_answer,
                    'code_snippets': code_snippets,
                    'query': query,
                    'context_used': context,
                    'query_id': query_id
                }

                yield {
                    'type': 'complete',
                    'answer': full_answer,
                    'retrieved_context': retrieved_context,
                    'trace': trace
                }

            except Exception as e:
                # Log the query with response time even if there's an error
                response_time_ms = int((time.time() - start_time) * 1000)
                query_id = await datastore.log_query(query, user_id, session_id, response_time_ms)

                error_msg = f"Error generating answer: {str(e)}"

                trace = {
                    'retrieved_docs': retrieved_context,
                    'model_response': error_msg,
                    'code_snippets': code_snippets,
                    'query': query,
                    'context_used': context,
                    'query_id': query_id
                }

                yield {
                    'type': 'complete',
                    'answer': error_msg,
                    'retrieved_context': retrieved_context,
                    'trace': trace
                }
        else:
            # Log the query with response time
            response_time_ms = int((time.time() - start_time) * 1000)
            query_id = await datastore.log_query(query, user_id, session_id, response_time_ms)

            error_msg = "[No LLM configured - unable to generate answer]"

            trace = {
                'retrieved_docs': retrieved_context,
                'model_response': error_msg,
                'code_snippets': code_snippets,
                'query': query,
                'context_used': context,
                'query_id': query_id
            }

            yield {
                'type': 'complete',
                'answer': error_msg,
                'retrieved_context': retrieved_context,
                'trace': trace
            }

    async def log_source_click(self, query_id: int, source_url: str, source_title: Optional[str] = None,
                              user_id: Optional[str] = None, session_id: Optional[str] = None) -> None:
        """
        Log when a user clicks on a source link
        """
        await datastore.log_source_click(
            query_id=query_id,
            source_url=source_url,
            source_title=source_title,
            user_id=user_id,
            session_id=session_id
        )

    async def call_subagent(self, subagent_name: str, query: str, context: Optional[Dict] = None) -> Dict:
        """
        Call a specific subagent with a query
        """
        return await self.subagent_registry.execute_subagent(subagent_name, query, context)

    def detect_subagent_intent(self, query: str) -> Optional[str]:
        """
        Detect if a query should be handled by a specific subagent
        """
        query_lower = query.lower()

        # Check if query is about ROS
        ros_keywords = ["ros", "ros2", "robot operating system", "rclpy", "rclcpp", "colcon", "ament", "rosbag"]
        if any(keyword in query_lower for keyword in ros_keywords):
            return "ros-helper"

        # Check if query is about simulation
        sim_keywords = ["gazebo", "ignition", "simulation", "world file", "sdf", "urdf", "physics engine", "robot simulation"]
        if any(keyword in query_lower for keyword in sim_keywords):
            return "sim-helper"

        return None
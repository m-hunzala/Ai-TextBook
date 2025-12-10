import os
import asyncio
from pathlib import Path
from typing import List
import markdown
from bs4 import BeautifulSoup
import re
from langchain.text_splitter import RecursiveCharacterTextSplitter

from .rag_agent import RAGAgent


class IngestionPipeline:
    def __init__(self):
        self.rag_agent = RAGAgent()
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=600,
            chunk_overlap=100,
            length_function=len,
            separators=["\n\n", "\n", " ", ""]
        )

    def extract_text_from_markdown(self, markdown_content: str) -> str:
        """Convert markdown to plain text"""
        # Convert markdown to HTML
        html = markdown.markdown(markdown_content)
        # Extract plain text from HTML
        soup = BeautifulSoup(html, 'html.parser')
        text = soup.get_text()
        # Clean up extra whitespace
        text = re.sub(r'\n\s*\n', '\n\n', text)
        return text.strip()

    def read_markdown_files(self, directory_path: str) -> List[dict]:
        """Read all markdown files from a directory and its subdirectories"""
        directory = Path(directory_path)
        documents = []

        for md_file in directory.rglob('*.md*'):  # Find all .md and .mdx files
            try:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    # Extract title from first heading or filename
                    title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
                    title = title_match.group(1) if title_match else md_file.stem

                    documents.append({
                        'content': content,
                        'title': title,
                        'source': str(md_file.relative_to(directory))
                    })
            except Exception as e:
                print(f"Error reading file {md_file}: {e}")

        return documents

    async def process_documents(self, documents: List[dict]):
        """Process and add documents to the vector store"""
        for doc in documents:
            print(f"Processing document: {doc['title']}")
            try:
                # Extract text from markdown
                text_content = self.extract_text_from_markdown(doc['content'])

                # Add to vector store
                await self.rag_agent.add_document(
                    document_content=text_content,
                    document_title=doc['title'],
                    document_source=doc['source']
                )
                print(f"Successfully added: {doc['title']}")
            except Exception as e:
                print(f"Error processing document {doc['title']}: {e}")

    async def run_ingestion(self, source_directory: str):
        """Run the full ingestion pipeline"""
        print(f"Starting ingestion from: {source_directory}")

        # Read markdown files
        documents = self.read_markdown_files(source_directory)
        print(f"Found {len(documents)} documents")

        # Process documents
        await self.process_documents(documents)

        print("Ingestion completed!")


# Example usage
async def main():
    # Set environment variables or load from .env
    import os
    from dotenv import load_dotenv
    load_dotenv()

    # Create ingestion pipeline
    ingestion = IngestionPipeline()

    # Run ingestion - adjust the path to your docs directory
    docs_path = os.getenv("DOCS_PATH", "docs")  # Default to docs directory
    await ingestion.run_ingestion(docs_path)


if __name__ == "__main__":
    asyncio.run(main())
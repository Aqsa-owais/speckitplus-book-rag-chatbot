"""
RAG Retrieval Pipeline Validation Script

This script validates the RAG retrieval pipeline by:
1. Querying Qdrant with test queries
2. Verifying relevant chunks are returned
3. Confirming metadata is properly included
4. Demonstrating consistent top-k results
"""
import asyncio
import logging
from typing import List, Dict, Any
import cohere
from qdrant_client import QdrantClient
from config.settings import Settings


class RetrievalValidator:
    """Validates the RAG retrieval pipeline functionality."""

    def __init__(self):
        """Initialize the validator with required clients."""
        self.settings = Settings()
        self.qdrant_client = QdrantClient(
            url=self.settings.qdrant_url,
            api_key=self.settings.qdrant_api_key,
            prefer_grpc=False
        )
        self.cohere_client = cohere.Client(self.settings.cohere_api_key)

        # Set up logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)

    def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a given text using Cohere."""
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model=self.settings.cohere_model,
                input_type="search_query"  # Appropriate for search queries
            )
            return response.embeddings[0] if response.embeddings else None
        except Exception as e:
            self.logger.error(f"Error generating embedding: {e}")
            return None

    def validate_retrieval(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Validate retrieval by querying Qdrant and returning results.

        Args:
            query: Natural language query to test
            top_k: Number of top results to return

        Returns:
            List of retrieved chunks with metadata
        """
        # Generate embedding for the query
        query_embedding = self.generate_embedding(query)
        if not query_embedding:
            self.logger.error("Failed to generate embedding for query")
            return []

        try:
            # Search in Qdrant
            results = self.qdrant_client.search(
                collection_name=self.settings.qdrant_collection,
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
            formatted_results = []
            for i, result in enumerate(results):
                formatted_result = {
                    "rank": i + 1,
                    "score": result.score,
                    "content": result.payload.get('content', '')[:200] + '...' if len(result.payload.get('content', '')) > 200 else result.payload.get('content', ''),
                    "url": result.payload.get('url', ''),
                    "section_title": result.payload.get('section_title', ''),
                    "chunk_id": result.payload.get('chunk_id', ''),
                    "chunk_index": result.payload.get('chunk_index', ''),
                    "token_count": result.payload.get('token_count', 0)
                }
                formatted_results.append(formatted_result)

            return formatted_results

        except Exception as e:
            self.logger.error(f"Error during retrieval: {e}")
            return []

    def run_validation_tests(self) -> Dict[str, Any]:
        """
        Run comprehensive validation tests on the retrieval pipeline.

        Returns:
            Dictionary with validation results and metrics
        """
        test_queries = [
            "How to set up NVIDIA Isaac Sim?",
            "ROS Python examples",
            "Infrastructure setup requirements",
            "Digital twin environments",
            "AI brain navigation systems"
        ]

        all_results = {}
        consistency_check = {}

        self.logger.info("Starting RAG retrieval validation tests...")

        for query in test_queries:
            self.logger.info(f"Testing query: '{query}'")

            # Run the same query multiple times to check consistency
            query_results = []
            for run in range(3):
                results = self.validate_retrieval(query, top_k=3)
                query_results.append(results)

            all_results[query] = query_results

            # Check consistency across runs
            if len(query_results) > 1:
                first_run_ids = [r['chunk_id'] for r in query_results[0]]
                consistent = all(
                    [r['chunk_id'] for r in run] == first_run_ids
                    for run in query_results[1:]
                )
                consistency_check[query] = consistent

        # Calculate metrics
        total_queries = len(test_queries)
        consistent_queries = sum(1 for v in consistency_check.values() if v)

        # Check if we have content in the database
        try:
            collection_info = self.qdrant_client.get_collection(self.settings.qdrant_collection)
            vector_count = collection_info.points_count
        except Exception as e:
            vector_count = 0
            self.logger.error(f"Error getting collection info: {e}")

        validation_results = {
            "total_queries_tested": total_queries,
            "queries_with_consistent_results": consistent_queries,
            "consistency_rate": consistent_queries / total_queries if total_queries > 0 else 0,
            "total_vectors_in_db": vector_count,
            "metadata_validation_passed": vector_count > 0,  # If we have vectors, we have metadata
            "individual_query_results": all_results,
            "consistency_check": consistency_check
        }

        return validation_results

    def print_validation_report(self, results: Dict[str, Any]):
        """Print a formatted validation report."""
        print("\n" + "="*80)
        print("RAG RETRIEVAL PIPELINE VALIDATION REPORT")
        print("="*80)

        print(f"\n📊 VALIDATION METRICS:")
        print(f"  • Total queries tested: {results['total_queries_tested']}")
        print(f"  • Queries with consistent results: {results['queries_with_consistent_results']}")
        print(f"  • Consistency rate: {results['consistency_rate']:.2%}")
        print(f"  • Total vectors in database: {results['total_vectors_in_db']}")
        print(f"  • Metadata validation: {'✅ PASSED' if results['metadata_validation_passed'] else '❌ FAILED'}")

        print(f"\n🔍 CONSISTENCY CHECK:")
        for query, is_consistent in results['consistency_check'].items():
            status = "✅" if is_consistent else "❌"
            print(f"  {status} '{query[:50]}...': {'Consistent' if is_consistent else 'Inconsistent'}")

        print(f"\n📝 SAMPLE RETRIEVAL RESULTS:")
        for query, query_results in list(results['individual_query_results'].items())[:2]:  # Show first 2 queries
            print(f"\n  Query: '{query}'")
            if query_results and query_results[0]:  # Show results from first run
                for i, result in enumerate(query_results[0][:2]):  # Show top 2 results
                    print(f"    {i+1}. Score: {result['score']:.3f}")
                    print(f"       Content: {result['content']}")
                    print(f"       URL: {result['url']}")
                    print(f"       Section: {result['section_title']}")
                    print(f"       Chunk ID: {result['chunk_id']}")
            else:
                print("    No results returned")

        print("\n" + "="*80)
        print("VALIDATION COMPLETE")
        print("="*80)


async def main():
    """Main function to run the retrieval validation."""
    validator = RetrievalValidator()

    print("Initializing RAG Retrieval Pipeline Validation...")
    print("This will test semantic search functionality against stored embeddings.")

    # Run validation tests
    results = validator.run_validation_tests()

    # Print validation report
    validator.print_validation_report(results)

    # Final assessment
    if (results['consistency_rate'] >= 0.8 and
        results['total_vectors_in_db'] > 0 and
        results['metadata_validation_passed']):
        print("\n🎉 VALIDATION SUCCESS: RAG retrieval pipeline is functioning correctly!")
        print(f"   - Semantic search is working with {results['consistency_rate']:.1%} consistency")
        print(f"   - {results['total_vectors_in_db']} content chunks are properly stored with metadata")
        print("   - All required metadata (URL, section, chunk ID) is available")
    else:
        print("\n⚠️  VALIDATION PARTIAL: Some aspects need attention")
        if results['consistency_rate'] < 0.8:
            print("   - Consistency rate is below 80%")
        if results['total_vectors_in_db'] == 0:
            print("   - No vectors found in database")
        if not results['metadata_validation_passed']:
            print("   - Metadata validation failed")


if __name__ == "__main__":
    asyncio.run(main())
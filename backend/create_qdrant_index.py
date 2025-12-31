import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Qdrant client
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

client = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key,
    https=True
)

print(f"Connecting to Qdrant at {qdrant_url}")
print(f"Using collection: {collection_name}")

try:
    # Create an index on the "url" field to fix the filtering issue
    client.create_payload_index(
        collection_name=collection_name,
        field_name="url",
        field_schema="keyword"  # or "text" depending on your needs
    )
    print("Successfully created index on 'url' field")
except Exception as e:
    print(f"Error creating index: {e}")

# Check collection info
try:
    collection_info = client.get_collection(collection_name)
    print(f"Collection info: {collection_info}")
except Exception as e:
    print(f"Error getting collection info: {e}")
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
    # Get current collection info
    collection_info = client.get_collection(collection_name)
    print(f"Current collection info: {collection_info}")
    print(f"Current points count: {collection_info.points_count}")

    # Clear all points in the collection
    client.delete_collection(collection_name)
    print(f"Deleted collection: {collection_name}")

    # Recreate the collection
    from qdrant_client.http.models import Distance, VectorParams
    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1024, distance=Distance.COSINE)  # Assuming Cohere embeddings size
    )
    print(f"Recreated collection: {collection_name}")

    # Create the index on 'url' field
    client.create_payload_index(
        collection_name=collection_name,
        field_name="url",
        field_schema="keyword"
    )
    print("Successfully created index on 'url' field")

except Exception as e:
    print(f"Error: {e}")
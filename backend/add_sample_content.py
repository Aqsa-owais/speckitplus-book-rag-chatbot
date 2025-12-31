import os
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
import uuid
from dotenv import load_dotenv
import cohere

# Load environment variables
load_dotenv()

# Initialize clients
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")
cohere_api_key = os.getenv("COHERE_API_KEY")

co = cohere.Client(cohere_api_key)
client = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key,
    https=True
)

print(f"Connecting to Qdrant at {qdrant_url}")
print(f"Using collection: {collection_name}")

# Sample content about ROS2 (since that was your query example)
sample_content = [
    {
        'content': "ROS2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. ROS2 provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. It is designed to support multiple robot platforms and operating systems.",
        'url': 'https://example.com/ros2-introduction',
        'chunk_id': 'ros2-intro-001'
    },
    {
        'content': "ROS2 architecture is based on a distributed system of processes that communicate with each other through topics, services, and actions. It uses DDS (Data Distribution Service) as the middleware for communication between nodes. Key concepts include Nodes (processes that perform computation), Topics (named buses over which nodes exchange messages), Services (synchronous request/response communication), and Actions (asynchronous goal-oriented communication).",
        'url': 'https://example.com/ros2-architecture',
        'chunk_id': 'ros2-arch-001'
    },
    {
        'content': "ROS2 packages are collections of nodes, libraries, and other files organized in a standard way. A package typically contains source code, configuration files, data files, and documentation. ROS2 uses ament as its build system, which supports CMake for C++ packages and colcon for building multiple packages together. Each package has a package.xml manifest file that describes dependencies and other metadata.",
        'url': 'https://example.com/ros2-packages',
        'chunk_id': 'ros2-pkg-001'
    },
    {
        'content': "ROS2 launch files allow you to start multiple nodes at once, set parameters, and configure your environment. They are written in Python and provide a more flexible alternative to ROS1 launch files. You can define conditions for starting nodes, pass arguments, and configure parameters in a centralized way. This makes it easier to manage complex robot applications with many interconnected nodes.",
        'url': 'https://example.com/ros2-launch',
        'chunk_id': 'ros2-launch-001'
    }
]

try:
    # Generate embeddings for the sample content
    print("Generating embeddings for sample content...")
    texts = [item['content'] for item in sample_content]
    response = co.embed(texts=texts, model='embed-english-v3.0', input_type="search_document")

    # Prepare points for Qdrant
    points = []
    for i, item in enumerate(sample_content):
        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=response.embeddings[i],
            payload={
                'content': item['content'],
                'url': item['url'],
                'chunk_id': item['chunk_id']
            }
        )
        points.append(point)

    # Upload to Qdrant
    print(f"Uploading {len(points)} points to Qdrant...")
    client.upsert(
        collection_name=collection_name,
        points=points
    )

    print("Sample content successfully added to Qdrant!")

    # Verify the upload
    count = client.count(collection_name=collection_name)
    print(f"Total points in collection: {count.count}")

except Exception as e:
    print(f"Error: {e}")
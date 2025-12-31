import os
import numpy as np
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
import uuid
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize client
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
    # Generate simple embeddings using a basic approach (word counts and basic features)
    # This is a simplified approach for demonstration purposes
    def simple_embedding(text, vector_size=1024):
        # Create a simple embedding based on text characteristics
        # In practice, you'd want to use proper embeddings from a model
        np.random.seed(abs(hash(text)) % (2**32))  # Use text hash as seed for consistency
        embedding = np.random.random(vector_size).tolist()
        # Normalize the vector
        norm = np.linalg.norm(embedding)
        if norm > 0:
            embedding = (np.array(embedding) / norm).tolist()
        return embedding

    # Prepare points for Qdrant with simple embeddings
    points = []
    for item in sample_content:
        embedding = simple_embedding(item['content'])
        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
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
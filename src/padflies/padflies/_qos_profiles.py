from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Create a QoS profile for maximum performance
qos_profile_performance = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Minimal latency, no retries
    durability=DurabilityPolicy.VOLATILE,  # Only delivers data to currently available subscribers
    history=HistoryPolicy.KEEP_LAST,  # Keeps only the last N messages
    depth=1,  # Keeps a short history to reduce memory use
)

qos_profile_simple = 10

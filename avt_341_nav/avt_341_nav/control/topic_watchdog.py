class TopicWatchdog:
    """Subscribes to a topic and keeps its most recent message."""

    def __init__(self, node, topic, msg_type, qos=10):
        self._msg = None
        self._sub = node.create_subscription(msg_type, topic, self._callback, qos)

    def _callback(self, msg):
        self._msg = msg

    def value(self):
        """Most recent message, or None if no message has been received."""
        return self._msg

    def is_valid(self):
        """True once at least one message has been received."""
        return self._msg is not None

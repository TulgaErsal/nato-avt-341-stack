# @param nh: Node handle for ROS2 node
# @param topic: Topic name to monitor
# @param topic_class: ROS message class for the topic
# @param field_checks: Optional list of message field checks (e.g. ["msg.data == True"] to check if msg.data is True)
# @param watchdog_rate: Rate at which to check the topic (in Hz). If no new message received in 1/rate period, topic marked as invalid.
class TopicWatchdog:
    def __init__(self, nh, topic, topic_class, field_checks=[], watchdog_rate=5.0):
        self.nh = nh
        self.topic = topic
        self.topic_class = topic_class
        self.field_checks = field_checks
        self.watchdog_rate = watchdog_rate

        # Subscribers
        self.topic_sub = self.nh.create_subscription(topic_class, topic, self.topic_callback, 10)

        # Timers
        self.nh.create_timer(1.0/self.watchdog_rate, self.watchdog_loop)

        # Class Vars
        self.msg = None
        self.rcvd_time = None
        self.valid = False

    def topic_callback(self, msg):
        self.msg = msg
        self.rcvd_time = self.nh.get_clock().now()

    def watchdog_loop(self):
        if self.msg is None or self.rcvd_time is None:
            self.valid = False # No communication received yet
            return

        time_diff = self.nh.get_clock().now() - self.rcvd_time
        if time_diff.nanoseconds * 1e-9 > 1.0 / self.watchdog_rate:
            self.valid = False # Communication timeout
            return

        for check in self.field_checks:
            eval_str = check.replace('msg', 'self.msg')
            if not eval(eval_str):
                self.valid = False # Field check failed
                return

        # Communication is active
        self.valid = True

    def is_valid(self):
        return self.valid

    def value(self):
        return self.msg
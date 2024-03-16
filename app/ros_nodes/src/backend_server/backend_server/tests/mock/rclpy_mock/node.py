
class Node:
    def __init__(self, node_name):
        pass

    def create_publisher(self, msg, topic, qos_profile):
        pass

    def create_node(self, node_name):
        pass

    def create_rate(self, rate):
        pass

    def create_timer(self, rate, callback):
        pass

    def get_logger(self):
        pass

    def spin_once(self, node, timeout_sec):
        pass

    def destroy_node(self):
        pass

    def shutdown(self):
        pass

    def create_client(self, srv, topic):
        pass

    def spin_until_future_complete(self, node, future):
        pass

    def create_subscription(self, msg, topic, callback, qos_profile):
        pass
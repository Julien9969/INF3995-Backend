import time


class Log:
    _TYPE = 'rcl_interfaces/msg/Log'
    __slots__ = ['stamp', 'level', 'name', 'msg', 'file', 'function', 'line', 'topics', 'type']

    def __init__(self):
        self.stamp = int(time.time())
        self.level = 0
        self.name = ''
        self.msg = ''
        self.file = ''
        self.function = ''
        self.line = 0
        self.topics = []
        self.type = ''

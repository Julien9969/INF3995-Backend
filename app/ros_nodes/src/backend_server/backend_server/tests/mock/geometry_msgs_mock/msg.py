class Twist():
    def __init__(self):
        self.linear = coord()
        self.angular = coord()


class coord():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0
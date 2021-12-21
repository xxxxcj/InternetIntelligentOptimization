class Thief:
    def __init__(self):
        self.path = list()
        self.location = None
        self.value = 0  # 偷窃到的价值

    def next(self, position):
        self.path.append(position)
        self.location = position
        


class MyMap:
    def __init__(self):
        self.x_range = 200  # size of background
        self.y_range = 200
        self.motions = [(-1, 0), (0, 1),
                        (1, 0),  (0, -1)]
        self.obs = set()

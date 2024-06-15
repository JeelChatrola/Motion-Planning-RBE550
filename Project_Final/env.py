class Env:
    def __init__(self):
        self.sx = 15
        self.sy = 15
        self.x_range = (0, self.sx)
        self.y_range = (0, self.sy)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()
        self.goal_rectangle = self.goal_rectangle()

    @staticmethod
    def obs_boundary():
        sx = 15
        sy = 15
        obs_boundary = [
            [0, 0, 1, sy],
            [0, sy, sx, 1],
            [1, 0, sx, 1],
            [sx, 1, 1, sy]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            # [15,15, 10, 10],
            # [45,10, 4, 20]
            [6,4,2.5,2.5]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [11,10,1.5],
            [4,10,1.5],
        ]

        return obs_cir
    
    @staticmethod
    def goal_rectangle():
        # goal locations in the order you want to visit them
        goal_locations = [
            [2,2,2,2],
            [10,2,2,2],
            [6.5,12,2,2],
            # [30,25,3,3],
            # [55,15,3,3],
            # [10,35,3,3],
        ]
        return goal_locations

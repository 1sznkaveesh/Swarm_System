CELL = 40
GRID_W = 15
GRID_H = 15


class Grid:

    def __init__(self):

        self.obstacles = set()


    def in_bounds(self,pos):

        x,y = pos
        return 0 <= x < GRID_W and 0 <= y < GRID_H


    def neighbors(self,pos):

        x,y = pos

        moves = [
            (x+1,y),
            (x-1,y),
            (x,y+1),
            (x,y-1)
        ]

        valid = []

        for m in moves:

            if self.in_bounds(m) and m not in self.obstacles:
                valid.append(m)

        return valid
import sys # used for debug/error messages to mms without interferring the communication protocol
import API
from collections import deque # queue used by BFS

# --DIRECTION CONSTANTS--------------------------------------------------------
# Direction encoding used throughout the entire program:
# 0 = North
# 1 = East
# 2 = Sout
# 3 = West

# change in x-coordinate when moving one cell in each direction
X_DIRECTION_OFFSET = [0,1,0,-1]

# change in y-coordinate when moving one cell in each direction
Y_DIRECTION_OFFSET = [1,0,-1,0]

LEFT_DIRECTION = {0:3, 1:0, 2:1, 3:2}
RIGHT_DIRECTION = {0:1, 1:2, 2:3, 3:0}
OPPOSITE_DIRECTION = {0:2, 1:3, 2:0, 3:1}

class FloodfillMicromouse:
    def __init__(self):
        self.maze_width = API.mazeWidth()
        self.maze_height = API.mazeHeight()

        # Internal map of every wall discovered so far
        # Each maze cell stores a set containing the blocked directions:
        #   discovered_walls[x][y] is a set of directions known to be blocked
        self.discovered_walls = [[set() for _in range(self.maze_height)] for _in in range(self.maze_width)]

        # Physical maze edge (outer boundary) is always known in advance
        for x in range(self.maze_width):
            self.discovered_walls[x][self.maze_height - 1].add(0) # North edge
            self.discovered_walls[x][0].add(2) # South edge

        for y in range(self.maze_height):
            self.discovered_walls[self.maze_width - 1][y].add(1) # East edge
            self.discovered_walls[0][y].add(3) # West edge

        self.x, self.y, self.heading = 0,0,0 # start bottom-left, facing north
        self.goal_cells = self._compute_goal_cells()
        self.flood_values = self._flood_fill()

    # --GOAL DEFINITION--------------------------------------------------------
    def _compute_goal_cells(self):
        # Standard micromouse goal: the 2x2 block at the center of the maze
        left_center_x, right_center_x = (self.maze_width - 1) // 2, (self.maze_width) // 2
        bottom_center_y, top_center_y = (self.maze_height - 1) // 2, (self.maze_height) // 2
        return {(x,y) for x in {left_center_x, right_center_x} for y in {bottom_center_y, top_center_y}}

# --CORE FLOODFILL
#   BFS from goal, using only known walls
    def _flood_fill(self):
        INF = float("inf")
        fload_values = [[INF] * self.maze_height for _ in range(self.maze_width)]
        cell_queue = deque()

        for(goal_x,goal_y) in self.goal_cells:
            fload_values[goal_y][goal_x] = 0
            cell_queue.append((goal_x,goal_y))

        while cell_queue:
            current_x, current_y = cell_queue.popleft()
            for direction in range(4):
                if direction in self.discovered_walls[current_x][current_y]:
                    continue # known walls block this direction

                next_x, next_y = current_x + X_DIRECTION_OFFSET[direction], current_y + Y_DIRECTION_OFFSET[direction]

                if not (0 <= next_x < self.maze_width and 0 <= next_y < self.maze_height):
                    continue

                if fload_values[next_x][next_y] > fload_values[current_x][current_y] + 1:
                    fload_values[next_x][next_y] = fload_values[current_x][current_y] + 1
                    cell_queue.append((next_x,next_y))

        return fload_values

    # --SENSING
    def _sense_and_record(self):
        """
        Detects surrounding walls using the front, left and right sensors.

        If any newly discovered walls are found, they are stored in the mouses's internal grid
        and mirror to neighboring cells.

        Returns:
             bool: True if new map data was recorded
                    False otherwise
        """
        new_wall_found = False
        sensor_readings = {
            self.heading: API.wallFront(),
            LEFT_DIRECTION[self.heading]:API.wallLeft(),
            RIGHT_DIRECTION[self.heading]:API.wallRight(),
        }

        for direction, is_wall in sensor_readings.items():
            if is_wall and direction not in self.discovered_walls[self.x][self.y]:
                self.discovered_walls[self.x][self.y].add(direction)

                next_x, next_y = self.x + X_DIRECTION_OFFSET[direction], self.y + Y_DIRECTION_OFFSET[direction]

                # reflect the newly discovered wall onto the adjacent neighbor
                if 0 <= next_x < self.maze_width and 0 <= next_y < self.maze_height:
                    self.discovered_walls[next_x][next_y].add(OPPOSITE_DIRECTION[direction])
                API.setWall(self.x, self.y, self._dir_letter(direction))
                new_wall_found = True
        return new_wall_found

    @staticmethod
    def _dir_letter(direction):
        return {0: "n", 1: "e", 2: "s", 3: "w"}[direction]

    def _face(self, target_direction):
        required_turn = (target_direction - self.heading) % 4

        if required_turn == 1:
            API.turnRight()
        elif required_turn == 3:
            API.turnLeft()
        elif required_turn == 2:
            API.turnRight()
            API.turnLeft()

        self.heading = target_direction


    def _step(self, direction):
        self._face(direction)
        if API.wallFront():
            if direction not in self.discovered_walls[self.x][self.y]:
                self.discovered_walls[self.x][self.y].add(direction)
                next_x, next_y = self.x + X_DIRECTION_OFFSET[direction], self.y + Y_DIRECTION_OFFSET[direction]

                if 0 <= next_x < self.maze_width and 0 <= next_y < self.maze_height:
                    self.discovered_walls[next_x][next_y].add(OPPOSITE_DIRECTION[direction])

                API.setWall(self.x, self.y, self._dir_letter(direction))
            self.flood = self._flood_fill()
            return False

        move_result = API.moveForward()
        if move_result == "crash":
            # the mms itself reports the move failed (crash)
            self.flood = self._flood_fill()
            return False

        self.x += X_DIRECTION_OFFSET[direction]
        self.y += Y_DIRECTION_OFFSET[direction]
        return True
    

    def run(self):
        API.setColor(self.x, self.y, "g") # mark start of the maze
        for (goal_x, goal_y) in self.goal_cells:
            API.setColor(goal_x, goal_y, "r")  # mark goal region of the maze

        visited_cells = 0
        max_steps = self.maze_width * self.maze_height * 20 

        for _ in range(max_steps):
            if API.wasReset():
                self.x, self.y, self.heading = 0,0,0
                API.ackReset()
                continue
            
            visited_cells += 1
            API.setText(self.x, self.y, str(self.flood[self.x][self.y]))
            
            if (self.x, self.y) in self.goal_cells:
                API.setColor(self.x, self.y, "b") 
                sys.stderr.write(f"GOAL REACHED IN {visited_cells} CELL-VISITS\n")
                sys.stderr.flush()
                return True
            
            newly_discovered = self._sense_and_record()
            if newly_discovered:
                self.flood = self._flood_fill()
                
            best_direction, lowest_flood_value = None, self.flood[self.x][self.y]
            for direction in range(4):
                if direction in self.discovered_walls[self.x][self.y]:
                    continue 
                next_x, next_y = self.x + X_DIRECTION_OFFSET[direction], self.y + Y_DIRECTION_OFFSET[direction]
                if not (0 <= next_x < self.maze_width and 0 <= next_y < self.maze_height):
                    continue
                if self.flood[next_x][next_y] < lowest_flood_value:
                    lowest_flood_value = self.flood[next_x][next_y]
                    best_direction = direction
                        
            if best_direction is None:                
                self.flood = self._flood_fill()
                if self.flood[self.x][self.y] == float("inf"):
                    sys.stderr.write("NO PATH TO GOAL FOUND WITH CURRENT KNOWLEDGE. STOPPING...")
                    sys.stderr.flush()
                    return False
                continue
            self._step(best_direction)
                
        sys.stderr.write("STEP LIMIT REACHED....STOPPING\n")
        sys.stderr.flush()
        return False
def main():
    micromouse = FloodfillMicromouse()
    micromouse.run()
    
if __name__ == "__main__":
    main()


import heapq
import numpy as np

class PuzzleState:
    def __init__(self, state, parent=None, move="Initial", depth=0, cost=0):
        # Initialize PuzzleState with given parameters
        self.state = state
        self.parent = parent
        self.move = move
        self.depth = depth
        self.cost = cost
        self.goal_state = np.array([[2, 0, 3], [1, 4, 5], [6, 7, 8]])

    def __eq__(self, other):
        # Define equality for PuzzleState objects
        return np.array_equal(self.state, other.state)

    def __lt__(self, other):
        # Define less than for PuzzleState objects based on cost and heuristic
        return (self.cost + self.heuristic()) < (other.cost + other.heuristic())

    def __hash__(self):
        # Define hash function for PuzzleState objects
        return hash(str(self.state))

    def heuristic(self):
        # Define the Manhattan distance heuristic
        return np.sum(np.abs(np.subtract(np.where(self.state != 0), np.where(self.goal_state != 0))))

    def get_children(self):
        # Generate possible child states by moving the zero tile
        children = []
        zero_position = np.where(self.state == 0)
        moves = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # Left, Right, Up, Down
        for move in moves:
            new_position = (zero_position[0][0] + move[0], zero_position[1][0] + move[1])
            if 0 <= new_position[0] < 3 and 0 <= new_position[1] < 3:
                new_state = np.copy(self.state)
                new_state[zero_position], new_state[new_position] = new_state[new_position], new_state[zero_position]
                children.append(PuzzleState(new_state, self, move, self.depth + 1, self.cost + 1))
        return children

    def print_path(self):
        # Recursively print the path from initial state to current state
        if self.parent is None:
            print("Initial State")
        else:
            self.parent.print_path()
            print("Move:", self.move)
        print(self.state)
        print()

def astar(start_state):
    # A* algorithm to find the goal state from the initial state
    open_list = []
    closed_list = set()
    heapq.heappush(open_list, start_state)

    while open_list:
        current_state = heapq.heappop(open_list)
        if np.array_equal(current_state.state, current_state.goal_state):
            return current_state  # Goal state found
        closed_list.add(current_state)

        for child in current_state.get_children():
            if child in closed_list:
                continue
            if child not in open_list:
                heapq.heappush(open_list, child)
            else:
                for open_state in open_list:
                    if open_state == child and open_state > child:
                        open_state = child
                        heapq.heapify(open_list)

    return None  # Goal state not found

# Example usage:
start_state = np.array([[1, 2, 3], [4, 0, 5], [6, 7, 8]])
start_state = PuzzleState(start_state)
goal_state = astar(start_state)
goal_state.print_path()


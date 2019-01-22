from collections import deque
import heapq
import time






def Manhattan_distance(board_string):

    heuristic = 0

    for i in range(len(board_string)):
        # (s[i] - 48) --> to convert from a string number to numerical number (using ascii table)
        X_distance = abs(int(i /3) - int(int(board_string[i]) / 3))
        Y_distance = abs(int(i % 3) - int(int(board_string[i]) % 3))
        total_distance = X_distance + Y_distance
        heuristic += total_distance
    return heuristic


class Board:

    def __init__(self,board_string = "" , depth = 0 ,action = "",parent = None):
        self.board_string = board_string
        self.depth = depth
        self.action = action
        self.parent = parent

    def generate_children(self):

        children = []
        # get boards zero position
        zero_position = self.board_string.index('0')
        # get all valid direction (up down left right)
        # if it's a valid move create a new (child) & append it to the children list


        if zero_position % 3 + 1 < 3:
            new_board_string = copy_and_swap(self.board_string,zero_position, zero_position + 1)
            new_child = Board(new_board_string,self.depth + 1,"Right",self)
            children.append(new_child)

        if zero_position % 3 - 1 >= 0:
            new_board_string = copy_and_swap(self.board_string,zero_position, zero_position - 1)
            new_child = Board(new_board_string,self.depth + 1,"Left",self)
            children.append(new_child)

        if zero_position + 3 <= 8:
            new_board_string = copy_and_swap(self.board_string,zero_position,zero_position+3)
            new_child = Board(new_board_string,self.depth + 1 ,"Down",self)
            children.append(new_child)

        if zero_position - 3 >= 0:
            new_board_string = copy_and_swap(self.board_string,zero_position,zero_position-3)
            new_child = Board(new_board_string,self.depth + 1,"Up",self)
            children.append(new_child)

        return children
    # we use this function to make the board class orderable.
    def __lt__(self, other):
        if isinstance(other, self.__class__):
            return self.board_string < other.board_string



def copy_and_swap( board_string,zero_position,updated_zero_position):
    new_board_string = list(board_string)
    new_board_string[zero_position] = new_board_string[updated_zero_position]
    new_board_string[updated_zero_position] = '0'
    return ''.join(new_board_string)

class eight_puzzle_solver:
    # marking visited boards
    visited = set()
    # create a stack for DFS
    stack = deque()
    # create a queue for BFS and A_star
    queue = deque()
    goal = "012345678"
    goal_found = False
    max_depth = 0
    nodes_expanded = 0
    search_depth = 0

    def __init__(self,method,init_board):
        self.initial_board = init_board
        if method == "dfs":
            self.DFS()
        elif method == "bfs":
            self.BFS()
        elif method == "A_star":
            self.A_star()

    def get_path(self, goal_board):

        path = []

        child = goal_board

        while child.board_string != self.initial_board.board_string:
            path.append(child.action)
            child = child.parent
        path.reverse()
        return path


    def DFS(self):

        algorithm_start_time = time.time()
        # initialize stack and visited dict
        self.stack.append(self.initial_board)
        self.visited.add(self.initial_board.board_string)

        # repeat while stack is not empty
        while len(self.stack) > 0:

            current_board = self.stack.pop()
            if current_board.board_string == self.goal:
                path_to_goal = self.get_path(current_board)

                self.goal_found = True
                self.search_depth = current_board.depth
                print("path to goal: "+str(path_to_goal))
                print("nodes expanded: " + str(self.nodes_expanded))
                print("search depth: " + str(self.search_depth))
                print("max search depth: " + str(self.max_depth))
                # cost of path is the search depth because the cost of a step is 1
                print("cost of path: "+ str(self.search_depth))
                print("running time: " + str(time.time() - algorithm_start_time))
                break
            else:
                self.nodes_expanded = self.nodes_expanded +1
               # generate children
                children = current_board.generate_children()

                for child in children:

                    if child.board_string not in self.visited:
                        self.visited.add(child.board_string)
                        self.stack.append(child)

                    self.max_depth = max(self.max_depth, child.depth)

        if not self.goal_found:
            print("no solution")

    def BFS(self):

        algorithm_start_time = time.time()
        # initialize queue and visited dict
        self.queue.append(self.initial_board)
        self.visited.add(self.initial_board.board_string)

        # repeat while queue is not empty
        while len(self.queue) > 0:

            current_board = self.queue.popleft()

            if current_board.board_string == self.goal:
                path_to_goal = self.get_path(current_board)

                self.goal_found = True
                self.search_depth = current_board.depth
                print("path to goal: " + str(path_to_goal))
                print("nodes expanded: " + str(self.nodes_expanded))
                print("search depth: " + str(self.search_depth))
                print("max search depth: " + str(self.max_depth))
                # cost of path is the search depth because the cost of a step is 1
                print("cost of path: " + str(self.search_depth))
                print("running time: " + str(time.time() - algorithm_start_time))
                break
            else:
                self.nodes_expanded = self.nodes_expanded + 1
                # generate children
                children = current_board.generate_children()

                for child in children:

                    if child.board_string not in self.visited:
                        self.visited.add(child.board_string)
                        self.queue.append(child)

                    self.max_depth = max(self.max_depth, child.depth)

        if not self.goal_found:
            print("no solution")

    def A_star(self):

        algorithm_start_time = time.time()
     # creating a container of f data of each node (f = g + h) , g is the path cost from source to node , h is the Manhattan distance
        A_star_f = dict()
        self.queue = []
        # initialize priority queue
        # get the heuristic of the initial board
        initial_heuristic = Manhattan_distance(self.initial_board.board_string)
        A_star_f[self.initial_board.board_string] = initial_heuristic

        self.visited.add(self.initial_board.board_string)
        heapq.heappush(self.queue,( A_star_f[self.initial_board.board_string] , self.initial_board) )
        # repeat while queue is not empty
        while len(self.queue) > 0:

            current_board = heapq.heappop(self.queue)[1]

            if current_board.board_string == self.goal:

                path_to_goal = self.get_path(current_board)
                self.goal_found = True
                self.search_depth = current_board.depth
                print("path to goal: " + str(path_to_goal))
                print("nodes expanded: " + str(self.nodes_expanded))
                print("search depth: " + str(self.search_depth))
                print("max search depth: " + str(self.max_depth))
                # cost of path is the search depth because the cost of a step is 1
                print("cost of path: " + str(self.search_depth))
                print("running time: " + str(time.time() - algorithm_start_time))
                break
            else:
                self.nodes_expanded = self.nodes_expanded + 1
                # generate children
                children = current_board.generate_children()

                for child in children:

                    child_manhattan = Manhattan_distance(child.board_string)
                    if child.board_string not in A_star_f.keys() or A_star_f[child.board_string] > child_manhattan + child.depth:
                        A_star_f[child.board_string] = child.depth + child_manhattan
                        heapq.heappush(self.queue, ( A_star_f[child.board_string], child))
                    self.max_depth = max(self.max_depth, child.depth)

        if not self.goal_found:
            print("running time: "+str(time.time() - algorithm_start_time))
            print("no solution")


# initial_board = Board("012345678")
# initial_board = Board("867254301")
#initial_board = Board("102345678")

# no solution case
# initial_board = Board("021345678")

initial_board = Board("618402735")
#initial_board = Board("864213570")
#initial_board = Board("123456780")

solver = eight_puzzle_solver("A_star",initial_board)










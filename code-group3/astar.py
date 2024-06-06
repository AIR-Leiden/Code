import math
from typing import List, Optional
import numpy as np

_world = """        #   
           #
   #       #
   #       #
F  #  R     
   #  #    #""".split("\n")


def find_position(world, character):
    for y in range(world.shape[0]):
        for x in range(world.shape[1]):
            if world[y][x] == character:
                return x, y
    raise "character not found"


def find_all_positions(world, character):
    positions = []
    for y in range(world.shape[0]):
        for x in range(world.shape[1]):
            if world[y][x] == character:
                positions.append((x, y))
    return positions


def add(position_a, position_b):
    return position_a[0] + position_b[0], position_a[1] + position_b[1]


def does_position_exist(world, position):
    return 0 <= position[1] < len(world) \
        and 0 <= position[0] < len(world[0])


def free_neighbours(world, position):
    potential_neighbours = [add(position, delta) for delta in [
        (1, 0),(-1, 0),(0, 1),(0, -1)
        #(1, 1), (1, 0), (1, -1),
        #(0, 1), (0, -1),
        #(-1, 1), (-1, 0), (-1, -1)
    ]]
    return [neighbour
            for neighbour in potential_neighbours
            if does_position_exist(world, neighbour) and
            world[neighbour[1]][neighbour[0]] <= 0]

neighbour_deltas = [(1, 0),(-1, 0),(0, 1),(0, -1)]
def for_free_neighbour(world, position, callback):
    for delta in neighbour_deltas:
        neighbour = add(position, delta)
        if does_position_exist(world, neighbour) and world[neighbour[1]][neighbour[0]] <= 0:
            callback(neighbour)



def distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
    #return math.sqrt((float(a[0]) - b[0]) ** 2 + (float(a[1]) - b[1]) ** 2)


def astar_old(world, from_position, to_position):
    # type: (np.ndarray, tuple, tuple) -> Optional[List[tuple]]
    """
    :param world: a list with indexes [y][x]
    """

    print("ASTAR START")

    visited = set()
    queue = [(from_position, [from_position], 0)]

    iterations = 0

    while len(queue) != 0:
        current_position, current_path, question_marks = queue.pop()

        iterations += 1
        if iterations > 5000:
            print("TOO MANY ITERATIONS")
            print("ASTAR FAILED")
            return None

        if current_position == to_position:
            print("ASTAR END")
            error_correct = 0

            if current_path[0] != from_position:
                error_correct = distance(from_position, current_path[0])
            print("path distance: " + str(get_length_of_path(current_path) + error_correct) + " visited: " + str(len(visited)))
            return current_path

        if current_position in visited:
            continue

        def callback(neighbour):
            d = 1 if world[neighbour[1]][neighbour[0]] == -1 else 0
            QUESTION_MARK_LIMIT = 3
            if question_marks + d <= QUESTION_MARK_LIMIT:
                queue.append((neighbour, current_path + [neighbour], question_marks + d))
        for_free_neighbour(world, current_position, callback)

        # probably the sorting step is one of the most critical parts of a star, could very well be that this way
        # of doing it is not the most efficient
        # in this case every step is counted as length of 1 and diagonal steps aren't more efficient
        # instead of len(path) the length of the path should probably be the sum of the distances between all the steps
        queue.sort(key=lambda entry:
        get_length_of_path(entry[1]) + distance(entry[0], to_position),
                   reverse=True)
        visited.add(current_position)

    print("ASTAR FAILED; NO PATH")
    return None


def get_length_of_path(path):
    return len(path)
    s = 0
    for i in range(1, len(path)):
        previous = path[i - 1]
        current = path[i]
        s += distance(previous, current)
    return s

"""

def print_world(world):
    for line in world:
        print(line)

print_world(_world)

path = astar(_world, find_position(_world, "R"), find_position(_world, "F"))
print(path)

# show the path in the world:
for position in path:
    _world[position[1]] = list(_world[position[1]])
    _world[position[1]][position[0]] = str(path.index(position) % 10)
    _world[position[1]] = "".join(_world[position[1]])
print_world(_world)
"""


def astar_fast(world, from_position, to_position):
    # type: (np.ndarray, tuple, tuple) -> Optional[List[tuple]]
    """
    :param world: a list with indexes [y][x]
    """


astar = astar_old
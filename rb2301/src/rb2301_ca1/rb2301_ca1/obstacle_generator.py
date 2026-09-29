import numpy as np
import xml.etree.ElementTree as ET
import os

height = 20
size_div = 6
width = 22
randomise = True # if false it will not change

workspace_directory = os.path.dirname(os.path.realpath(__file__))[:-22]
overwrite_file =  workspace_directory + '/rb2301_gz/worlds/obstacle_world_ca1.sdf'
obstacle_model = f'file:///{workspace_directory}/rb2301_gz/meshes/coke/6'

def generate_maze():
    maze_arr = np.zeros((height, width))
    maze_arr[2, int(width/2)] = 1
    for x in range(2, height-2):
        for y in range(1, width-1):
            chance = np.sum(maze_arr[x-1:x+2, y-1:y+2])
            roll = np.random.random()
            if roll < 0.5 - 0.4*2**chance + x/(3*height): # Tweak the chance of cans spawning here
                maze_arr[x,y] = 1

    maze_arr[:2, int(width/2)-1:int(width/2)+2] = 0 # Clear the starting area
    return maze_arr

def generate_edge_case(scenario):
    """Create reproducible obstacle layouts around the spawn position."""
    maze_arr = np.zeros((height, width))
    centre_x = int(width / 2)
    centre_y = int(height / 2)

    # Original L-shaped cases.
    if scenario in {'front_left', 'front_right'}:
        maze_arr[2:7, centre_x - 4:centre_x + 5] = 1
        if scenario == 'front_left':
            maze_arr[0:3, centre_x + 4] = 1
        elif scenario == 'front_right':
            maze_arr[0:3, centre_x - 4] = 1
        return maze_arr

    # Single left diagonal wall pair: two mirrored diagonal lines forming a corridor.
    # The robot should drive between the two lines and stay inside the gap.
    if scenario in {'left_diagonal_track', 'diagonal_left', 'left_track'}:
        for x in range(height):
            for y in range(width):
                dx = x - centre_y
                dy = y - centre_x
                # left-side corridor walls
                if abs((dx + dy) + 13) < 2:
                    maze_arr[x, y] = 1
                if abs((dx + dy) + 10) < 2:
                    maze_arr[x, y] = 1
                if abs((dx + dy) + 7) < 2:
                    maze_arr[x, y] = 1
        # leave a clear pathway between the two diagonal lines
        for x in range(height):
            for y in range(width):
                dx = x - centre_y
                dy = y - centre_x
                if abs(dx + dy + 10) < 3.5:
                    maze_arr[x, y] = 0
        return maze_arr

    raise ValueError(
        f"Unknown obstacle scenario {scenario!r}; "
        "expected 'random', 'front_left', 'front_right', or 'left_diagonal_track'"
    )

def add_coke_element(x, y, n):
    obstacle = ET.Element("include")
    uri = ET.Element("uri")
    uri.text = obstacle_model
    obstacle.append(uri)
    name = ET.Element("name")
    name.text = f'coke{n}'
    obstacle.append(name)
    pose = ET.Element("pose")
    pose.text = f'{x} {y} 0 0 0 0'
    obstacle.append(pose)
    return obstacle

def generate_sdf_file(scenario='random'):
    if not randomise and scenario == 'random':
        return

    if scenario == 'random':
        maze_arr = generate_maze()
    else:
        maze_arr = generate_edge_case(scenario)

    n = 1
    print(f"Generating {scenario} obstacle world...")
    tree = ET.parse(overwrite_file)
    root = tree.getroot()
    world = root[0]
    for element in reversed(world):  # Remove all coke obstacles
        if element.tag == 'include':
            world.remove(element)
    tree.write(overwrite_file)

    for x in range(height):  # Add in new obstacles
        for y in range(width):
            if maze_arr[x, y] == 1:
                x_pos = x * 2 / size_div
                y_pos = (y - width / 2) / size_div
                world.append(add_coke_element(x_pos, y_pos, n))
                n += 1

    tree.write(overwrite_file)

if __name__ == '__main__':
    generate_sdf_file(scenario= 'front_left')

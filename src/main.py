import pygame
import pymunk
from sys import exit
from random import randint

pygame.init()

SCREEN_HEIGHT = 800
SCREEN_WIDTH = 800

# pygame setup variables
screen = pygame.display.set_mode((SCREEN_HEIGHT, SCREEN_WIDTH))
pygame.display.set_caption("Cloth Simulation")
clock = pygame.time.Clock()
FPS = 200
show_FPS = True

# pymunk variables
space = pymunk.Space()
space.gravity = (0, -800)

# colors
WHITE = (255, 255, 255)
GREY = (200, 200, 200)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
ORANGE = (255, 255, 0)
YELLOW = (235, 255, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PURPLE = (200, 0, 255)

color_list = [RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE]

# fonts
selected_font = pygame.font.Font('assets/font.ttf',
                                 30)  # https://fonts.google.com/specimen/Inconsolata


def clear_space(space):
    for body in space.bodies:
        space.remove(body)
    for constraint in space.constraints:
        space.remove(constraint)
    for shape in space.shapes:
        space.remove(shape)


def convert_coords(point):  # convert to int, transform to pygame y convention
    return int(point[0]), int(SCREEN_HEIGHT - point[1])


def render_text(text, position, color=BLACK):
    rendered_text = selected_font.render(text, True, color)
    screen.blit(rendered_text, position)


def print_fps():
    render_text(str(f'FPS: {int(clock.get_fps())}'),
                (10, 10))  # https://stackoverflow.com/questions/67946230/show-fps-in-pygame


class PendulumConnector:
    def __init__(self, body, link, link_type='body'):
        self.body = body
        if link_type == 'body':
            self.link = link
        elif link_type == 'static_body':
            self.link = pymunk.Body(body_type=pymunk.Body.STATIC)
            self.link.position = link
        joint = pymunk.PinJoint(self.body, self.link)
        joint.collide_bodies = False
        space.add(joint)

    def draw(self):
        pos1 = convert_coords(self.body.position)
        pos2 = convert_coords(self.link.position)
        pygame.draw.aaline(screen, BLACK, pos1, pos2, 5)
        pygame.draw.circle(screen, BLACK, convert_coords(self.link.position), 5)


class PendulumPoint:
    def __init__(self, x, y, number=5):
        # pymunk properties
        self.body = pymunk.Body()
        self.body.position = x, y
        self.shape = pymunk.Circle(self.body, 4)
        self.shape.density = 1
        self.shape.elasticity = 0.8
        self.shape.collision_type = 2
        self.radius = 4
        self.color = color_list[number]

        # trail
        self.draw_trail = False
        self.trail_color = BLACK
        self.position_log = []
        self.position_trail_radius = 2
        self.position_log_depth = 20
        self.steps = 0

        space.add(self.body, self.shape)

    def draw(self):
        if self.draw_trail:
            # draw trail
            if self.steps >= self.position_log_depth:
                self.position_log[self.steps % self.position_log_depth] = convert_coords(self.body.position)
            elif self.steps < self.position_log_depth:
                self.position_log.append(convert_coords(self.body.position))
                print(len(self.position_log))
            self.steps += 1
            for i in range(len(self.position_log)):
                pygame.draw.circle(screen, self.trail_color, self.position_log[i],
                                   self.position_trail_radius)

        # draw body
        converted_position = convert_coords(self.body.position)
        pygame.draw.circle(screen, self.color, converted_position, self.radius)

    def toggle_trail(self):
        self.draw_trail = not self.draw_trail


class Net:
    def __init__(self, x, y, size, separation):
        self.n_rows = size
        self.n_cols = size
        self.sep = separation
        self.connectors = []
        self.grid = []
        self.free_nodes = []
        self.fixed_nodes = []

        # initialize empty grid
        grid_line = []
        for i in range(self.n_rows):
            grid_line.append((0, 0))
        for i in range(self.n_cols):
            self.grid.append(list(grid_line))

        # fill in grid positions
        for i in range(self.n_rows):
            for j in range(self.n_cols):
                self.grid[i][j] = (x + i * self.sep, y - j * self.sep)

        # create net with first row fixed
        for i in range(self.n_rows):
            for j in range(1, self.n_cols):
                point = PendulumPoint(self.grid[i][j][0], self.grid[i][j][1])
                self.free_nodes.append(point)

        for i in range(self.n_cols):
            fixed_node = PendulumConnector(self.free_nodes[i * (self.n_rows - 1)].body, (x + i * self.sep, y),
                                           link_type='static_body')
            # note: the above free node grid is actually n_cols x n_rows - 1 since we exclude the fixed points from the grid
            self.fixed_nodes.append(fixed_node)

        # set connections for each free node to each NSEW neighbor
        for i in range(len(self.free_nodes)):
            # Propagate connections from top left to bottom right (only S and E)
            # Vertical
            if i % (self.n_rows - 1) == self.n_rows - 2:  # TODO make this not demand an n x n grid
                pass
            elif i % (self.n_rows - 1) != self.n_rows - 2:
                connection = PendulumConnector(self.free_nodes[i].body, self.free_nodes[i + 1].body, link_type='body')
                self.connectors.append(connection)
            # horizontal
            if i >= len(self.free_nodes) - (
                    self.n_rows - 1):  # ignore last column to the right (since we're connecting to the right)
                pass
            elif i < len(self.free_nodes) - (self.n_rows - 1):  # don't try to connect last column to the right
                connection = PendulumConnector(self.free_nodes[i].body, self.free_nodes[i + (self.n_rows - 1)].body,
                                               link_type='body')
                self.connectors.append(connection)

    def draw(self):
        for connector in self.connectors:
            connector.draw()
        for fixed_node in self.fixed_nodes:
            fixed_node.draw()
        for point in self.free_nodes:
            point.draw()


    def apply_force(self):
        pass

    def apply_disturbance(self):
        for node in self.free_nodes:
            node.body.apply_impulse_at_local_point((1000, 0))

    def toggle_trails(self):
        for node in self.free_nodes:
            node.toggle_trail()


test_params = [100, 800, 30, 20]
test_net = Net(*test_params)

run = True
play = True
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        # pause and play button
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                play = not play
            if event.key == pygame.K_a and FPS > 0:
                FPS -= 1
                print(f'FPS decreased to {FPS}')
            if event.key == pygame.K_d:
                FPS += 1
                print(f'FPS increased to {FPS}')
            if event.key == pygame.K_r:
                space.remove(*space.bodies, *space.shapes, *space.constraints)
                test_net = Net(*test_params)
                print(f'Simulation reset')
            if event.key == pygame.K_i:
                test_net.apply_disturbance()
            if event.key == pygame.K_t:
                test_net.toggle_trails()
                print(f'Node trails toggled')

    if play:
        screen.fill(WHITE)

        space.step(1 / FPS)
        test_net.draw()
        if show_FPS:
            print_fps()

    else:
        screen.fill(GREY)
        test_net.draw()

        if show_FPS:
            print_fps()

    clock.tick(FPS)
    pygame.display.update()

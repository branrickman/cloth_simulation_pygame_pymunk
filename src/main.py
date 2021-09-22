import pygame
import pymunk
from sys import exit

pygame.init()

SCREEN_HEIGHT = 800
SCREEN_WIDTH = 800

# pygame setup variables
screen = pygame.display.set_mode((SCREEN_HEIGHT, SCREEN_WIDTH))
pygame.display.set_caption("Cloth Simulation")
clock = pygame.time.Clock()
FPS = 50
show_FPS = True

# pymunk variables
space = pymunk.Space()
space.gravity = (0, -900)

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
        self.shape = pymunk.Circle(self.body, 10)
        self.shape.density = 1
        self.shape.elasticity = 1
        self.shape.collision_type = 2
        self.position_log = []
        self.position_trail_radius = 5

        self.radius = 10
        self.color = color_list[number]

        space.add(self.body, self.shape)

    def draw(self):
        converted_position = convert_coords(self.body.position)
        self.position_log.append(converted_position)
        pygame.draw.circle(screen, self.color, converted_position, self.radius)


class Pendulum:
    def __init__(self, x1, y1, x2, y2, number, x3=(SCREEN_WIDTH // 2), y3=(SCREEN_HEIGHT // 2), mode=0):
        self.pend_point1 = PendulumPoint(x1, y1, number)
        self.pend_point2 = PendulumPoint(x2, y2, number + 4)
        self.pend_conn1 = PendulumConnector(self.pend_point1.body, (x3, y3), link_type='static_body')
        self.pend_conn2 = PendulumConnector(self.pend_point1.body, self.pend_point2.body, link_type='body')
        self.static_x = x3
        self.static_y = y3
        self.mode = mode
        self.draw_trail = True

    def draw(self):
        if self.draw_trail:
            for i in range(len(self.pend_point1.position_log)):
                pygame.draw.circle(screen, self.pend_point1.color, self.pend_point1.position_log[i],
                                   self.pend_point1.position_trail_radius)
            for i in range(len(self.pend_point2.position_log)):
                pygame.draw.circle(screen, self.pend_point2.color, self.pend_point2.position_log[i],
                                   self.pend_point2.position_trail_radius)
        if self.mode:
            self.pend_conn1.draw()
            self.pend_conn2.draw()
        self.pend_point1.draw()
        self.pend_point2.draw()


class Net:
    def __init__(self, x, y, size, separation, pinned):
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
            # Propogate connections from top left to bottom right (only S and E)
            # Vertical
            if i % (self.n_rows - 1) == self.n_rows - 2:  # TODO make this not demand an n x n grid
                pass
            elif i % (self.n_rows - 1) != self.n_rows - 2:
                connection = PendulumConnector(self.free_nodes[i].body, self.free_nodes[i + 1].body, link_type='body')
                self.connectors.append(connection)
            # horizontal
            if i >= len(self.free_nodes) - (
                    self.n_rows - 1):  # ignore last column to the right (since we're connecting to the right)
                print(i)
            elif i < len(self.free_nodes) - (self.n_rows - 1):  # don't try to connect last column to the right
                connection = PendulumConnector(self.free_nodes[i].body, self.free_nodes[i + (self.n_rows - 1)].body,
                                               link_type='body')
                self.connectors.append(connection)

        # initialize free nodes
        # for (x, y) in self.free:
        #     pass
        # # initialize pinned nodes
        # for (x, y) in self.pinned:
        #     pass
        #     pinned_point = PendulumConnector

    def draw(self):
        for connector in self.connectors:
            connector.draw()
        for point in self.free_nodes:
            point.draw()
        for fixed_node in self.fixed_nodes:
            fixed_node.draw()


test_net = Net(300, 500, 4, 100, 0)
# pendulum1 = Pendulum(200, 500, 300, 700, number=1, mode=True)

# pendulum_group = [pendulum1]

# pp1 = PendulumPoint(100, 500, 3)
# pend_conn1 = PendulumConnector(pp1.body, (200, 700), link_type='static_body')

click_counter = 1  # tracks odd and even clicks to generate new pendulums
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
                # initialize_pendulums()
                print(f'Pendulums reset')
            # if event.key == pygame.K_t:
            #     for p in pendulum_group:
            #         p.draw_trail = not p.draw_trail
            #     print(f'Pendulum trails toggled')
            # if event.key == pygame.K_c:
            #     for p in pendulum_group:
            #         p.pend_point1.position_log = []
            #         p.pend_point2.position_log = []
            #     print(f'Pendulum trails cleared')
            # if event.key == pygame.K_m:
            #     for p in pendulum_group:
            #         p.mode = not p.mode
            #     print(f'Pendulum trails toggled')
        # mouse_event = pygame.mouse.get_pressed()
        # if mouse_event[0]:
        #     pos1 = pygame.mouse.get_pos()
        #     pendulum1.pend_point2.body.position = convert_coords(pos1)
        #     pendulum1.pend_point1.body.position = convert_coords(
        #         ((pos1[0] + pendulum1.static_x) // 2, (pos1[1] + pendulum1.static_y) // 2))
        #     # DEBUG
        #     # print(f'static: {(pendulum1.static_x, pendulum1.static_y)} \n'
        #     #       f'point2: {pos1}\n'
        #     #       f'point1: {pendulum1.pend_point1.body.position}')
    if play:
        screen.fill(WHITE)
        space.step(1 / FPS)
        test_net.draw()
        if show_FPS:
            print_fps()

    else:
        screen.fill(GREY)
        # for pendulum in pendulum_group:
        #     pendulum.draw()

        if show_FPS:
            print_fps()
    clock.tick(FPS)
    pygame.display.update()

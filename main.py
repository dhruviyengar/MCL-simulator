import pygame
import sys
import numpy as np
from robot import Robot

def ray_collision(x, y, heading, noise=False):

    # Convert heading to radians
    rad = np.deg2rad(heading)
    # Define ray direction vector
    dx = np.sin(rad)
    dy = np.cos(rad)
    # Avoid division by zero: set very small component to a very small number
    if abs(dx) < 1e-10:
        dx = 1e-10 if dx >= 0 else -1e-10
    if abs(dy) < 1e-10:
        dy = 1e-10 if dy >= 0 else -1e-10
    # Compute distances to vertical walls x = +/-400
    if dx > 0:
        dist_x_pos = (400 - x) / dx
    else:
        dist_x_pos = (-400 - x) / dx
    # Compute distances to horizontal walls y = +/-400
    if dy > 0:
        dist_y_pos = (400 - y) / dy
    else:
        dist_y_pos = (-400 - y) / dy

    dist = min(dist_x_pos, dist_y_pos)
    if noise:
        dist += np.random.uniform(-1, 1)
    return dist

x_range = (-400, 400)
y_range = (-400, 400)

rob = Robot(0, 0, 0)

pygame.init()

screen = pygame.display.set_mode((800, 800))
pygame.display.set_caption("MCL Simulator")

RED = (255, 0, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)

n_particles = 500

particles = np.column_stack((
    np.random.uniform(*x_range, n_particles),
    np.random.uniform(*y_range, n_particles)
))

clock = pygame.time.Clock()  # For framerate and dt

font = pygame.font.Font(None, 36)

def screenCordX(coord):
    return int(coord + screen.get_width() / 2)

def screenCordY(coord):
    return int(-coord + screen.get_height() / 2)  # flip Y

prevRay = 0

running = True
while running:
    dt = clock.tick(240) / 1000
    screen.fill(BLACK)

    # Compute angle based on robotZ
    rad = (np.pi / 180) * rob.z if rob.z != 0 else 0
    endX = 15 * np.sin(rad) + rob.x
    endY = 15 * np.cos(rad) + rob.y

    pygame.draw.line(
        screen, RED,
        (screenCordX(rob.x), screenCordY(rob.y)),
        (screenCordX(endX), screenCordY(endY)),
        2
    )

    pygame.draw.circle(
        screen, RED,
        (screenCordX(rob.x) + 1, screenCordY(rob.y)),
        4
    )
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
   

    keys = pygame.key.get_pressed()

    speed = 50  # forward speed in units/sec
    rotation_speed = 90  # degrees/sec

    if keys[pygame.K_w]:
        rob.x += speed * np.sin(rad) * dt
        rob.y += speed * np.cos(rad) * dt

    if keys[pygame.K_a]:
        rob.z -= rotation_speed * dt

    if keys[pygame.K_d]:
        rob.z += rotation_speed * dt
    ##DELTA CALC GOES HERE

    rayfwd = ray_collision(rob.x, rob.y, rob.z, True)
    rayright = ray_collision(rob.x, rob.y, rob.z + 90, True)
    rayback = ray_collision(rob.x, rob.y, rob.z + 180, True)
    rayleft = ray_collision(rob.x, rob.y, rob.z + 270, True)


    for i in range(particles.shape[0]):
        particles[i, 0] += rob.deltaX() + np.random.uniform(-1, 1)
        particles[i, 1] += rob.deltaY() + np.random.uniform(-1, 1)

        if np.fabs(particles[i, 0]) > 400 or np.fabs(particles[i, 0]) > 400:
            particles[i, 0] = np.random.uniform(-400, 400)
            particles[i, 1] = np.random.uniform(-400, 400)

    for x, y in particles:
        pygame.draw.circle(screen, (255, 255, 255), (screenCordX(x), screenCordY(y)), 2)  # White dot

    weights = np.empty(n_particles)

    for i, (x, y) in enumerate(particles):
        angle = rob.z + np.random.uniform(-2.5, 2.5)
        sigma = 10000
        weights[i] = np.exp(-0.5 * ((ray_collision(x, y, angle) - rayfwd) / sigma)**2) * np.exp(-0.5 * ((ray_collision(x, y, angle + 90) - rayright) / sigma)**2) * np.exp(-0.5 * ((ray_collision(x, y, angle + 180) - rayback) / sigma)**2) * np.exp(-0.5 * ((ray_collision(x, y, angle + 270) - rayleft) / sigma)**2)

    weighted = np.empty_like(particles)

    xSum = 0
    ySum = 0

    avgWeight = np.sum(weights) / n_particles
    randWeight = np.random.uniform(0, avgWeight)

    totalWeight = 0
    
    j = 0

    for i, (x, y) in enumerate(weighted):
        weight = i * avgWeight + randWeight

        while totalWeight < weight:
            if j > np.size(weights):
                break
            totalWeight += weights[j]
            j += 1
        weighted[i] = particles[j-1]

        xSum += weighted[i, 0]
        ySum += weighted[i, 1]

    predictedX = xSum / n_particles
    predictedY = ySum / n_particles

    particles[:] = weighted[:]

    pygame.draw.circle(
        screen, BLUE,
        (screenCordX(predictedX), screenCordY(predictedY)),
        4
    )
    
    text_surface = font.render(f"FWD {round(rayfwd, -1)}, RIGHT, {round(rayright, -1)} X: {rob.x} Y: {rob.y} Z: {rob.z}", True, (255, 255, 255))
    screen.blit(text_surface, (20, 20))


    rob.recordPrev()

    pygame.display.flip()


pygame.quit()
sys.exit()

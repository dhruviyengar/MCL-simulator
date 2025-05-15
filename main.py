import pygame
import sys
import numpy as np
from robot import Robot

def ray_collision(x, y, z):
  rad = (np.pi / 180) * z if z != 0 else 0
  rad = np.atan(np.sin(rad) / np.cos(rad))
  xDist = np.abs(400 - x) if rad > 0 else np.abs(-400 - x)
  xDist = xDist / (np.abs(np.sin(rad)) if np.abs(np.sin(rad)) != 0 else 0.00000000001)
  yDist = np.abs(400 - y) if (rad > np.pi / 2 or rad < np.pi / 2) else np.abs(-400 - y)
  yDist = yDist / (np.abs(np.cos(rad)) if np.abs(np.cos(rad)) != 0 else 0.00000000001)
  return np.fmin(xDist, yDist) + np.random.uniform(-1, 1)

x_range = (-100, 100)
y_range = (-100, 100)

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

    rayfwd = ray_collision(rob.x, rob.y, rob.z)
    rayright = ray_collision(rob.x, rob.y, rob.z + 90)
    rayback = ray_collision(rob.x, rob.y, rob.z + 180)
    rayleft = ray_collision(rob.x, rob.y, rob.z + 270)


    particles[:, 0] += rob.deltaX() + np.random.uniform(-0.5, 0.5)
    particles[:, 1] += rob.deltaY() + np.random.uniform(-0.5, 0.5)

    for x, y in particles:
        pygame.draw.circle(screen, (255, 255, 255), (screenCordX(x), screenCordY(y)), 2)  # White dot

    weights = np.empty(n_particles)

    for i, (x, y) in enumerate(particles):
        angle = rob.z + np.random.uniform(-5, 5)
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

    particles = weighted

    pygame.draw.circle(
        screen, BLUE,
        (screenCordX(predictedX), screenCordY(predictedY)),
        4
    )

    rob.recordPrev()

    pygame.display.flip()


pygame.quit()
sys.exit()

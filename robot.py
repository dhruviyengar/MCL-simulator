import numpy as np

def ray_collision(self, x, y, z):
  xDist = np.abs(np.fmin(400 - x, -400 - x)) / np.abs(np.sin(z))
  yDist = np.abs(np.fmin(400 - y, -400 - y)) / np.abs(np.cos(z))
  return np.fmin(xDist, yDist)


class Robot:
  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z
    self.prevX = x
    self.prevY = y

  def recordPrev(self):
    self.prevX = self.x
    self.prevY = self.y

  def deltaX(self):
    return self.x - self.prevX
  
  def deltaY(self):
    return self.y - self.prevY

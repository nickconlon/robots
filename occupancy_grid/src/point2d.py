#!/usr/bin/env python

from math import sqrt

class point2d:
  def __init__(self, x, y):
    self.x = x
    self.y = y

  def getX(self):
    return self.x

  def getY(self):
    return self.y

  def getDistance(self, x, y):
    return sqrt((self.x -x)**2 + (self.y-y)**2)

  def getDistance(self, otherPoint):
    return sqrt((self.x-otherPoint.x)**2 + (self.y-otherPoint.y)**2)


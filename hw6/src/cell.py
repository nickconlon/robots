#!/usr/bin/env python

import Point as point

class Cell:
  def __init__(self, point):
    self.point = point
    self.probability = 0.5

  def getPoint(self):
    return self.point

  def getProbability(self):
    return self.probability

  def setProbability(self, prob):
    self.probability = prob



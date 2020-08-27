import sys
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.image as mp1mg
import numpy as np

# file 1 is the original, 2 is the constructed map.
file2 = sys.argv[1]

map1 = Image.open('map.png', "r") 
map2 = Image.open(file2, "r") 

print "Comparing: ", map1.filename, " ", map2.filename



pix1 = map1.load()
pix2 = map2.load()

map1copy = map1.copy()
map2copy = map2.copy()

p1 = map1copy.load()
p2 = map2copy.load()

for i in range(300):
  for j in range(300):

    if(pix1[i,j] == 205 or pix1[i,j] != 255): #white
      p1[i,j] = 0 # black
    else:
      p1[i,j] = 255 # white

    if(pix2[i,j] == 205 or pix2[i,j] == 0): #white
      p2[i,j] = 0 # black
    else:
      p2[i,j] = 255 # white

alike = 0
total = 0
for i in range(300):
  for j in range(300):
    if(p1[i,j] == p2[i,j]):
      alike = alike+1

    total = total+1
print "alike ", alike
print "total ", total
print float(alike/float(total))

map1copy.save("output1.png")
map2copy.save("output2.png")



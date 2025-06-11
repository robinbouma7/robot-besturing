"""tafelcontrol controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

tafel = Robot()

table = tafel.getDevice("tafel-motor")

#table.setPosition(float('inf'))

table.setVelocity(2 * math.pi / 75)

timestep = int(tafel.getBasicTimeStep())

while tafel.step(timestep) != -1:
   
    pass

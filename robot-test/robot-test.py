"""robot-test controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

discrot = robot.getDevice("disc-rotate")
armvert = robot.getDevice("arm-vertical")
armhor = robot.getDevice("arm-horizontal")

clawrot = robot.getDevice("claw-rotate")
clawrothor = robot.getDevice("claw-rotate-hor")

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

discrot.setPosition(float('inf'))
discrot.setVelocity(1)

armheight = 0
goingup = True

armxpos = 0

goingforward = True

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    if goingup:
        armheight = armheight + 0.01
        if armheight > 0.1:
            goingup = False
    else:    
        armheight = armheight - 0.01
        if armheight < -0.1:
            goingup = True
    
    if goingforward:
        armxpos = armxpos + 0.01
        if armxpos > 0.35:
            goingforward = False
    else:    
        armxpos = armxpos - 0.01
        if armxpos < -0.35:
            goingforward = True
    
    armvert.setPosition(armheight)
    armhor.setPosition(armxpos)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

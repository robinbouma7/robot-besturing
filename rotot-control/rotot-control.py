"""rotot-control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject
import servocontrol as control
import math
# create the Robot instance.
robot = Robot()

control.init(robot)

camera = Camera("robo-cam")
camera.recognitionEnable(100)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

def getposition(id):
    """get the position of the target"""
    loop = 0
    while True:
        recognised = camera.getRecognitionObjects()
        for data in recognised:
            if data.getId() == id:
                return data.getPositionOnImage()
        loop += 1
        if loop > 20:
            print("could not find position of id: " + str(id))
            return None

def test():
    control.Rotate(1, 1)
    robot.step(1000)
    control.Rotate(-2, 1)
    robot.step(1000)
    control.Rotate(1, 1)
    robot.step(1000)
    control.Armhorizontal(0.3, 1)
    robot.step(1000)
    control.Armhorizontal(-0.3, 1)
    robot.step(1000)
    control.TightenClaw(1, 1)
    robot.step(1000)
    control.OpenClaw(1)
    robot.step(1000)
    control.RotateClawHorizontal(0.5, 1)
    robot.step(1000)
    control.RotateClawHorizontal(-1, 1)
    robot.step(1000)
    control.RotateClawHorizontal(0.5, 1)
    robot.step(1000)
    control.RotateClawVertical(0.5, 1)
    robot.step(1000)
    control.RotateClawVertical(-1, 1)
    robot.step(1000)
    control.RotateClawVertical(0.5, 1)
    robot.step(1000)

def simplecontrol():

    #variables for logic
    lockId = -1
    position = [-1, -1]
    size = [-1, -1]
    retryAttempts = 0


    #calibration values
    imageMid = 320
    vertOffset = 280
    grabSize = 480
    centerMargin = 8

    idlerotate = 0.02

    horDistMult = 0.02
    vertDistMult = 0.08
    armDistMult = 0.002
    clawDistMult = 0.3

    horSpeed = 1
    vertSpeed = 1
    armspeed = 1
    clawSpeed = 1



    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        
        #print("loop!")
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        recognised = camera.getRecognitionObjects()
        #camera.getImage()
        recCount = 0
        recCount = len(recognised)
        found = False
        lookingforid = False
        if recCount <= 0 and lockId == -1:
            #print("test")
            control.Rotate(idlerotate)
        else :
            #print("test2")
            for data in recognised:
                if lockId == -1:
                    lookingforid = True
                    #no id set
                    lockId = data.getId()
                    found = True
                elif lookingforid:
                    if data.getId() > lockId:
                        #found a newer id
                        lockId = data.getId()
                else:
                    if lockId == data.getId():
                        found = True
                        position = data.getPositionOnImage()
                        size = data.getSizeOnImage()
                        #get location of object
                        
            if found:
                print("found object with id: " + str(lockId))
                print("position on image: " + str(position[0]) + ", " + str(position[1]))
                if position[0] < imageMid:
                    control.Rotate(horDistMult * (imageMid - position[0]) / imageMid, horSpeed)
                elif position[0] > imageMid:
                    control.Rotate(-horDistMult * (position[0] - imageMid) / imageMid, horSpeed)
                
                if position[1] < imageMid + vertOffset:
                    control.ArmVertical(vertDistMult * (imageMid - position[1]) / imageMid, vertSpeed)
                elif position[1] > imageMid + vertOffset:
                    control.ArmVertical(-vertDistMult * (position[1] - imageMid) / imageMid, vertSpeed)

                if position[0] >= imageMid - centerMargin and position[0] <= imageMid + centerMargin and position[1] >= imageMid - vertOffset - centerMargin and position[1] <= imageMid + vertOffset + centerMargin:
                    
                    print("object in the middle, moving forward")
                    if size[0] >= grabSize: 
                        control.StopArmHorizontal()
                        control.StopArmVertical()
                        control.StopRotate()                  
                        control.TightenClaw(clawDistMult, clawSpeed)
                    else:
                        control.Armhorizontal(armDistMult, armspeed)

            else:
                retryAttempts += 1
                if retryAttempts > 10:
                    #reset lockId
                    print("lost tracking for too long, resetting lockId")
                    lockId = -1
                    retryAttempts = 0


def advancedcontrolangle():
    #variables for logic
    lockId = -1
    position = [-1, -1]
    size = [-1, -1]
    retryAttempts = 0
    lastPosition = [-1, -1]
    #rotation, armhor, armvert
    lastrobotPosition = [-1, -1, -1]

    firstangle = -1

    #calibration values
    idlerotate = 0.02
    imageMid = 320
    rotatecheckdist = 0.1
    rotateDistMult = -0.00075
    verticalDistMult = -0.03
    rotatespeed = 1
    rotatemult = -0.0001

    clawrotatemult = -0.00025
    clawrotatespeed = 1

    vertspeed = 1
    vertmult = -0.00015

    armoffset = 0.03
    vertoffset = 200

    #math constants
    checkdist = 0.02 # distance between the 2 positions after the check rotation
    camerafov = 1.13446 # 65 degrees in radians
    degreebypixel = camerafov / 640 # 640 is the width of the camera image in pixels


    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        
        #print("loop!")
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        recognised = camera.getRecognitionObjects()
        #camera.getImage()
        recCount = 0
        recCount = len(recognised)
        found = False
        lookingforid = False
        if recCount <= 0 and lockId == -1:
            #print("test")
            control.Rotate(idlerotate)
        else :
            #print("test2")
            for data in recognised:
                if lockId == -1:
                    lookingforid = True
                    #no id set
                    lockId = data.getId()
                elif lookingforid:
                    if data.getId() > lockId:
                        #found a newer id
                        lockId = data.getId()
                
                        
            while position[0] <= imageMid - 3 or position[0] >= imageMid + 3:
                position = data.getPositionOnImage()
                control.Rotate((position[0] - imageMid) * rotatemult, rotatespeed)
                robot.step(timestep)

            control.StopRotate()
            position = data.getPositionOnImage()
            lastPosition = position
            lastrobotPosition = [control.rotation, control.armhorpos, control.armvertpos]
            #rotate towards object
            while position[0] != imageMid:
                control.RotateClawHorizontal((position[0] - imageMid) * clawrotatemult, clawrotatespeed)
                robot.step(timestep)
                position = data.getPositionOnImage()
            
            control.stopClawHorizontal()
            firstangle = control.clawrotposhor
            print("first angle: " + str(firstangle))
            control.Rotate(rotatecheckdist, rotatespeed)
            #move to location
            robot.step(250)
            
            position = data.getPositionOnImage()

            while position[0] != imageMid:
                control.RotateClawHorizontal((position[0] - imageMid) * clawrotatemult, clawrotatespeed)
                robot.step(timestep)
                position = data.getPositionOnImage()

           
            print("second angle: " + str(control.clawrotposhor))

            print("test: " + str(control.clawrotposhor + rotatecheckdist))

            angle1 = 90 # Convert radians to degrees
            angle2 = 90 - abs(math.degrees(control.clawrotposhor + rotatecheckdist))
           
            #gebruik sinus regel

            print("angle1: " + str(angle1))
            print("angle2: " + str(angle2))
            angle3 = 180 - (angle1 + angle2)
            print("angle3: " + str(angle3))

            angle1rad = math.radians(angle1)
            angle2rad = math.radians(angle2)
            angle3rad = math.radians(angle3)

            distance = checkdist / math.sin(angle3rad) * math.sin(angle1rad)

            control.Armhorizontal(distance - armoffset, 1)
                                

            #temprot = distance / math.sin(math.radians(90)) * math.sin(math.radians(90 - angle1 - angle3))
            control.Rotate(-rotatecheckdist, rotatespeed)
            control.RotateClawHorizontal(-control.clawrotposhor, 1)
            print("distance: " + str(distance))

            
                
            #calculate depth by the diffrence of the 2 positions and the defined movement of the robot
            robot.step(150)
            while position[1] <= imageMid + vertoffset - 3 or position[1] >= imageMid + vertoffset + 3:
                position = data.getPositionOnImage()
                control.ArmVertical((position[1] + vertoffset - imageMid) * vertmult, vertspeed)
                robot.step(timestep)

            control.StopArmVertical()
            
            control.TightenClaw(1, 1)

            while True:
                robot.step(timestep)

def advancedcontrol():
    #variables for logic
    lockId = -1
    position = [-1, -1]
    size = [-1, -1]
    retryAttempts = 0
    lastPosition = [-1, -1]
    #rotation, armhor, armvert
    lastrobotPosition = [-1, -1, -1]
    #rotation, armhor, armvert
    startposition = [0, 0, 0]
    traylocations = [
        [-1.5, 0, 0.2],
        [-1.5, 0.1, 0.2]
    ]
    nexttraylocation = 0

    firstangle = -1

    #calibration values
    idlerotate = 0.015
    imageMid = 320
    rotatecheckdist = 0.1
    rotateDistMult = -0.0007
    verticalDistMult = -0.03
    rotatespeed = 0.25
    rotatemult = -0.00005

    clawrotatemult = -0.00025
    clawrotatespeed = 0.25

    vertspeed = 0.1
    vertmult = -0.0002

    armoffset = 0.02
    armspeed = 0.25
    yoffset = 0

    clawclosedist = 0.825

    #math constants
    checkdist = 0.02 # distance between the 2 positions after the check rotation
    camerafov = 1.13446 # 65 degrees in radians
    radialbypixel = camerafov / 640 / 2 # 640 is the width of the camera image in pixels


    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        
        #print("loop!")
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        recognised = camera.getRecognitionObjects()
        #camera.getImage()
        recCount = 0
        recCount = len(recognised)
        lookingforid = False
        if recCount <= 0 and lockId == -1:
            #print("test")
            control.Rotate(idlerotate)
        else :
            #print("test2")
            for data in recognised:
                if lockId == -1:
                    lookingforid = True
                    #no id set
                    lockId = data.getId()
                elif lookingforid:
                    if data.getId() > lockId:
                        #found a newer id
                        lockId = data.getId()
                
            position = getposition(lockId)    
            if position is None:
                lockId = -1
                continue  

            while position[0] < imageMid or position[0] > imageMid + 1:
                position = getposition(lockId) 
                control.Rotate((position[0] - imageMid) * rotatemult, rotatespeed)
                robot.step(timestep)

            control.StopRotate()
            position = getposition(lockId) 
            if position is None:
                lockId = -1
                continue  
                
            ypos = position[1]
            #rotate towards object
            while position[0] != imageMid:
                control.RotateClawHorizontal((position[0] - imageMid) * clawrotatemult, clawrotatespeed)
                robot.step(timestep)
                position = getposition(lockId) 
                if position is None:
                    lockId = -1
                    continue  
                
            
            control.stopClawHorizontal()
            control.Rotate(rotatecheckdist, rotatespeed)
            #move to location
            while control.rotation != control.GetRotation():
                robot.step(timestep)
            
            position = getposition(lockId)
            if position is None:
                lockId = -1
                continue  
                 
            

            angle1 = 90
            angle2 = 90 - math.degrees((position[0] - imageMid) * radialbypixel)
            angle3 = 180 - (angle1 + angle2)

            yangle1 = math.degrees((ypos - imageMid) * radialbypixel)
            yangle2 = 90 - yangle1

            print("yangle1: " + str(yangle1))
            print("yangle2: " + str(yangle2))

            print("angle1: " + str(angle1))
            print("angle2: " + str(angle2))
            print("angle3: " + str(angle3))

            angle1rad = math.radians(angle1)
            angle2rad = math.radians(angle2)
            angle3rad = math.radians(angle3)

            distance = checkdist / math.sin(angle3rad) * math.sin(angle1rad)
            ydist = distance / math.sin(math.radians(yangle2)) * math.sin(math.radians(yangle1))

            print("distance: " + str(distance))
            print("ydist: " + str(ydist))
            
            if distance > 0.3 or ydist > 0.2 or ydist < -0.2:
                print("distance too large, resetting lockId")
                lockId = -1
                continue

            control.Armhorizontal(distance - armoffset, armspeed)
            control.ArmVertical(-ydist, vertspeed)              

            #temprot = distance / math.sin(math.radians(90)) * math.sin(math.radians(90 - angle1 - angle3))
            control.Rotate(-rotatecheckdist, rotatespeed)
            control.RotateClawHorizontal(-control.clawrotposhor, 1)
            

            while control.armhorpos != control.GetArmHorizontal() and control.armvertpos != control.GetArmVertical() and control.rotation != control.GetRotation() and control.clawrotposhor != control.GetClawHorizontal():
                robot.step(timestep)
            
            control.TightenClaw(clawclosedist, 1)

            robot.step(2000)

            control.Armhorizontal(traylocations[nexttraylocation][2] - control.GetArmHorizontal(), 0.2)
            control.Rotate(traylocations[nexttraylocation][0] - control.GetRotation(), 1)
            control.ArmVertical(traylocations[nexttraylocation][1] - control.GetArmVertical(), 1)
            control.RotateClawVertical(-0.1,1)

            while control.armhorpos != control.GetArmHorizontal() and control.armvertpos != control.GetArmVertical() and control.rotation != control.GetRotation():
                robot.step(timestep)

            control.OpenClaw(1)
            nexttraylocation += 1
            if nexttraylocation >= len(traylocations):
                nexttraylocation = 0

            robot.step(2000)

            


            control.Armhorizontal(startposition[2] - control.GetArmHorizontal(), 1)
            control.Rotate(startposition[0] - control.GetRotation(), 1)
            control.ArmVertical(startposition[1] - control.GetArmVertical(), 1)
            

            while control.armhorpos != control.GetArmHorizontal() and control.armvertpos != control.GetArmVertical() and control.rotation != control.GetRotation():
                robot.step(timestep)

            lockId == -1

def movingcontrol():
    
    CurID = -1
    Position = [-1, -1]
    Size = [-1, -1]
    retryAttempts = 0

    imageMid = 320
    vertOffset = 280
    grabSize = 480
    centerMargin = 8

    idlerotate = 0.02
    horDistMult = 0.02
    vertDistMult = 0.08
    armDistMult = 0.002
    clawDistMult = 0.3

    horSpeed = 1
    vertSpeed = 1
    armspeed = 1
    clawSpeed = 1

    # Main loop:
    while robot.step(timestep) != -1:
        recognised = camera.getRecognitionObjects()
        recCount = 0
        recCount = len(recognised)
        found = False
        lookingforid = False
        if recCount <= 0 and CurID == -1:
            control.Rotate(idlerotate)
        else :
            for data in recognised:
                if CurID == -1:
                    lookingforid = True
                    #no id set
                    CurID = data.getId()
                    found = True
                elif lookingforid:
                    if data.getId() > CurID:
                        
                        #found a newer id
                        CurID = data.getId()
                        found = True
                else:
                    if CurID == data.getId():
                        found = True
                        Position = data.getPositionOnImage()
                        Size = data.getSizeOnImage()
                        
            if found:
                print("found object with id: " + str(CurID))
                print("position on image: " + str(Position[0]) + ", " + str(Position[1]))
                if Position[0] < imageMid:
                    control.Rotate(horDistMult * (imageMid - Position[0]) / imageMid, horSpeed)
                elif Position[0] > imageMid:
                    control.Rotate(-horDistMult * (Position[0] - imageMid) / imageMid, horSpeed)
                
                if Position[1] < imageMid + vertOffset:
                    control.ArmVertical(vertDistMult * (imageMid - Position[1]) / imageMid, vertSpeed)
                elif Position[1] > imageMid + vertOffset:
                    control.ArmVertical(-vertDistMult * (Position[1] - imageMid) / imageMid, vertSpeed)

                if Position[0] >= imageMid - centerMargin and Position[0] <= imageMid + centerMargin and Position[1] >= imageMid - vertOffset - centerMargin and Position[1] <= imageMid + vertOffset + centerMargin:
                    
                    print("object in the middle, moving forward")
                    if Size[0] >= grabSize: 
                        control.StopArmHorizontal()
                        control.StopArmVertical()
                        control.StopRotate()                  
                        control.TightenClaw(clawDistMult, clawSpeed)
                    else:
                        control.Armhorizontal(armDistMult, armspeed)

            else:
                retryAttempts += 1
                if retryAttempts > 10:
                    #reset lock
                    print("lost tracking for too long, resetting lockId")
                    CurID = -1
                    retryAttempts = 0



#test()
#simplecontrol()
advancedcontrol()
#movingcontrol()



# voorkeur aan klauw bewegen ipv arm
# statisch plukken op vaste loactie
# klauw omhoog draaien om er over heen te kunnen
# je kan om de aardbei heen draaien voor plukken
# 
#
#
#
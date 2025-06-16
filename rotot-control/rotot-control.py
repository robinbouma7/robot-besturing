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
camera.recognitionEnable(50)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

def getposition(id):
    """get the position of the target"""
    loop = 0
    while True:
        recognised = camera.getRecognitionObjects()
        for data in recognised:
            if data.getId() == id:
                temppos = [-1, -1, -1, -1]
                temppos[0] = data.getPositionOnImage()[0]
                temppos[1] = data.getPositionOnImage()[1]
                temppos[2] = data.getSizeOnImage()[0]
                temppos[3] = data.getSizeOnImage()[1]
                return temppos
        loop += 1
        robot.step(timestep)
        if loop >= 50:
            print("could not find position of id: " + str(id))
            return None

def test():
    control.Rotate(1, 1)
    robot.step(1000)
    control.Rotate(-2, 1)
    robot.step(1000)
    control.Rotate(1, 1)
    robot.step(1000)
    control.ArmHorizontal(0.3, 1)
    robot.step(1000)
    control.ArmHorizontal(-0.3, 1)
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
                        control.ArmHorizontal(armDistMult, armspeed)

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

            control.ArmHorizontal(distance - armoffset, 1)
                                

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
    position = [-1, -1, -1, -1]
    size = [-1, -1]
    retryAttempts = 0
    lastPosition = [-1, -1]
    #rotation, armhor, armvert
    lastrobotPosition = [-1, -1, -1]
    #rotation, armvert, armhor
    startposition = [0, 0, 0]
    traylocations = [
        [-1.5, 0, 0.2],
        [-1.5, 0.09, 0.2]
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

    armoffset = 0.075
    armspeed = 0.25
    yoffset = 0

    clawyankdist = -0.5

    clawclosedist = 0.9

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

        tempint = 0
        i = 0
       
        #print("test2")
        for data in recognised:
           
            if lockId == -1:
                lookingforid = True
                #no id set
                lockId = data.getId()
                tempint = i
            elif lookingforid:
                if data.getId() > lockId:
                    #found a newer id
                    lockId = data.getId()
                    tempint = i
            i += 1

        if lockId == -1 or recCount <= 0:
            print("idlerotate")
            control.Rotate(idlerotate)
            continue

        print("found object with id: " + str(lockId))
        print("object type: " + str(recognised[tempint].getModel()))
            
        position = getposition(lockId)    
        if position is None:
            print("lost1")
            lockId = -1
            continue  

        while position[0] < imageMid or position[0] > imageMid + 1:
            position = getposition(lockId) 
            if position is None:
                lockId = -1
                continue 
            control.Rotate((position[0] - imageMid) * rotatemult, rotatespeed)
            robot.step(timestep)
        if lockId == -1:
            print("lost2")
            continue

        control.StopRotate()
        position = getposition(lockId) 
        if position is None:
            print("lost3")
            lockId = -1
            continue  
            
        print("position1: " + str(position[0]))
        print("position2: " + str(position[1]))
        ypos = position[1]
        
        control.Rotate(rotatecheckdist, rotatespeed)
        #move to location
        while control.rotation != control.GetRotation():
            robot.step(timestep)
        
        position = getposition(lockId)
        if position is None:
            print("lost5")
            lockId = -1
            continue  
                
        

        angle1 = 90
        angle2 = 90 - math.degrees((position[0] - imageMid) * radialbypixel)
        angle3 = 180 - (angle1 + angle2)

        yangle1 = math.degrees((ypos - imageMid) * radialbypixel)
        yangle2 = 90 - yangle1

        if angle3 <= 0 or yangle2 <= 0:
            print("angle3 or yangle2 is less than or equal to 0, resetting lockId")
            lockId = -1
            continue

        print("yangle1: " + str(yangle1))
        print("yangle2: " + str(yangle2))

        print("angle1: " + str(angle1))
        print("angle2: " + str(angle2))
        print("angle3: " + str(angle3))

        print("position1: " + str(position[0]))
        print("position2: " + str(position[1]))

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

        control.ArmHorizontal(distance - armoffset, armspeed)
        control.ArmVertical(-ydist, vertspeed)              

        #temprot = distance / math.sin(math.radians(90)) * math.sin(math.radians(90 - angle1 - angle3))
        control.Rotate(-rotatecheckdist, rotatespeed)
        control.RotateClawHorizontal(-control.clawrotposhor, 1)
        

        while control.armhorpos != control.GetArmHorizontal() and control.armvertpos != control.GetArmVertical() and control.rotation != control.GetRotation() and control.clawrotposhor != control.GetClawHorizontal():
            robot.step(timestep)
        
        control.TightenClaw(clawclosedist, 1)

        while control.claw1pos != control.GetClawTightness():
            robot.step(timestep)

        control.ArmHorizontal(traylocations[nexttraylocation][2] - control.GetArmHorizontal(), 0.2)
        control.Rotate(traylocations[nexttraylocation][0] - control.GetRotation(), 1)
        control.ArmVertical(traylocations[nexttraylocation][1] - control.GetArmVertical(), 1)
        control.RotateClawHorizontal(clawyankdist, 1)

        while control.hordist != control.GetArmHorizontal() or control.vertdist != control.GetArmVertical() or control.rotation != control.GetRotation():
            robot.step(timestep)

        control.OpenClaw(1)
        nexttraylocation += 1
        if nexttraylocation >= len(traylocations):
            nexttraylocation = 0

        while control.claw1pos != control.GetClawTightness():
            robot.step(timestep)

        
        control.ArmHorizontal(startposition[2] - control.GetArmHorizontal(), 1)
        control.Rotate(startposition[0] - control.GetRotation(), 1)
        control.ArmVertical(startposition[1] - control.GetArmVertical(), 1)
        control.RotateClawHorizontal(0 - control.GetClawHorizontal(), 1)
        

        while control.hordist != control.GetArmHorizontal() or control.vertdist != control.GetArmVertical() or control.rotation != control.GetRotation():
            robot.step(timestep)

        lockId == -1
        print("finished moving to tray location, resetting lockId")

def movingcontrol():
    
    #variables for logic
    lockId = -1
    position = [-1, -1, -1, -1]
    retryAttempts = 0
    lastPosition = [-1, -1]
    #rotation, armhor, armvert
    lastrobotPosition = [-1, -1, -1]
    #rotation, armhor, armvert, clawhor
    startposition = [0.5, 0, 0, 0.2]
    #rotation, armhor, armvert, clawhor
    traylocations = [
        [-1.5, 0, 0.2, 0],
        [-1.5, 0.1, 0.2, 0]
    ]
    nexttraylocation = 0

    firstangle = -1

    #calibration values
    idlerotate = 0.015
    imageMid = 320
    rotatecheckdist = 0.1
    rotateDistMult = -0.0007
    verticalDistMult = -0.0001
    rotatespeed = 0.25
    rotatemult = -0.00005
    precisemult = 0.75
    precisearmmult = 0.01

    clawrotatemult = -0.00025
    clawrotatespeed = 0.25
    clawclosespeed = 1

    vertspeed = 0.1
    vertmult = -0.0002

    armoffset = 0.02
    armspeed = 0.25
    armmovementspeed = -0.005
    yoffset = -100

    clawclosedist = 1

    clawyankdist = -0.5

    leftmargin = 100
    rightmargin = 10
    vertoffset = 0

    grabsize = 200
    grabsizemargin = 15
    startmovesize = 30
    vertstartsize = 35



    #math constants
    checkdist = 0.02 # distance between the 2 positions after the check rotation
    camerafov = 1.13446 # 65 degrees in radians
    radialbypixel = camerafov / 640 / 2 # 640 is the width of the camera image in pixels

    control.ArmHorizontal(startposition[2], 1)
    control.Rotate(startposition[0], 1)
    control.ArmVertical(startposition[1], 1)
    control.RotateClawHorizontal(startposition[3], 1)

    while control.armhorpos != control.GetArmHorizontal() and control.armvertpos != control.GetArmVertical() and control.rotation != control.GetRotation():
        robot.step(timestep)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        
        recognised = camera.getRecognitionObjects()
        recCount = 0
        recCount = len(recognised)
        lookingforid = False
       
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
                    

        if lockId == -1 or recCount <= 0:
            #wait for strawberry
            #control.Rotate(idlerotate)
            continue

        print("found object with id: " + str(lockId))
            
        position = getposition(lockId)    
        if position is None:
            lockId = -1
            continue  

        if(position[2] < startmovesize):
            print("object too small, waiting")
            robot.step(timestep)
            continue

        grabbing = False

        while not grabbing:
            position = getposition(lockId)
             
            if position is None:
                lockId = -1
                continue 

            if position[0] < imageMid - leftmargin or position[0] > imageMid + rightmargin:
                control.Rotate((position[0] + leftmargin - imageMid) * rotatemult, rotatespeed)
            else:
                control.StopRotate()
               
            if position[1] < imageMid + yoffset - vertoffset and position[2] >= vertstartsize:
                #omlaag
                if control.GetArmVertical() < control.vertdist:
                    control.StopArmVertical()
                control.ArmVertical((position[1] - imageMid + yoffset) * verticalDistMult, vertspeed)
            elif position[1] > imageMid + yoffset + vertoffset and position[2] >= vertstartsize:
                #omhoog
                if control.GetArmVertical() > control.vertdist:
                    control.StopArmVertical()
                control.ArmVertical((position[1] + vertoffset - imageMid + yoffset) * verticalDistMult, vertspeed)
            else:
                control.StopArmVertical()
            
            if position[2] >= grabsize - grabsizemargin and position[2] <= grabsize + grabsizemargin:
                control.StopArmHorizontal()         
                control.TightenClaw(clawclosedist, clawclosespeed)
                grabbing = True
            elif position[0] >= imageMid - leftmargin and position[0] <= imageMid + rightmargin and position[2] >= startmovesize:
                control.ArmHorizontal((position[2] - grabsize) * armmovementspeed, armspeed)

            robot.step(timestep)
        if lockId == -1:
            continue

        #precise movement to stay with strawberry
        while control.claw1pos != control.GetClawTightness():
            position = getposition(lockId)
             
            if position is None:
                lockId = -1
                continue 

            if position[0] < imageMid - leftmargin or position[0] > imageMid + rightmargin:
                control.Rotate((position[0] + leftmargin - imageMid) * rotatemult * precisemult, rotatespeed)

            if position[1] < imageMid + yoffset - vertoffset:
                #omlaag
                if control.GetArmVertical() < control.vertdist:
                    control.StopArmVertical()
                control.ArmVertical((position[1] - imageMid + yoffset) * verticalDistMult * precisemult, rotatespeed)
            elif position[1] > imageMid + yoffset + vertoffset:
                #omhoog
                if control.GetArmVertical() > control.vertdist:
                    control.StopArmVertical()
                control.ArmVertical((position[1] + vertoffset - imageMid + yoffset) * verticalDistMult * precisemult, rotatespeed)
            
            if position[2] >= grabsize - grabsizemargin and position[2] <= grabsize + grabsizemargin:
                control.StopArmHorizontal()  
            else:
                control.ArmHorizontal((position[2] - grabsize) * armmovementspeed * precisearmmult, armspeed)
            robot.step(timestep)
            
        control.StopArmHorizontal()
        control.StopRotate()
        control.StopArmVertical()
        #move to tray location
        control.RotateClawHorizontal(clawyankdist, 1)
        control.Rotate(traylocations[nexttraylocation][0] - control.GetRotation(), 1)
        while control.clawrotposhor != control.GetClawHorizontal():
            robot.step(timestep)
        control.Rotate(traylocations[nexttraylocation][0] - control.GetRotation(), 1)
        control.ArmVertical(traylocations[nexttraylocation][1] - control.GetArmVertical(), 1)
        control.ArmHorizontal(traylocations[nexttraylocation][2] - control.GetArmHorizontal(), 0.2)
        control.RotateClawHorizontal(traylocations[nexttraylocation][3] - control.GetClawHorizontal(), 1)
        
        

        while control.hordist != control.GetArmHorizontal() or control.vertdist != control.GetArmVertical() or control.rotation != control.GetRotation() or control.clawrotposhor != control.GetClawHorizontal():
            robot.step(timestep)

        control.OpenClaw(1)
        nexttraylocation += 1
        if nexttraylocation >= len(traylocations):
            nexttraylocation = 0

        robot.step(2000)

        

        #move to start position
        control.ArmHorizontal(startposition[2] - control.GetArmHorizontal(), 1)
        control.Rotate(startposition[0] - control.GetRotation(), 1)
        control.ArmVertical(startposition[1] - control.GetArmVertical(), 1)
        control.RotateClawHorizontal(startposition[3] - control.GetClawHorizontal(), 1)

        while control.hordist != control.GetArmHorizontal() or control.vertdist != control.GetArmVertical() or control.rotation != control.GetRotation() or control.clawrotposhor != control.GetClawHorizontal():
            robot.step(timestep)

        lockId == -1
        print("finished moving to tray location, resetting lockId")

def targetcontrol():
    
    #variables for logic
    lockId = -1
    position = [-1, -1, -1, -1]
    retryAttempts = 0
    lastPosition = [-1, -1]
    #rotation, armhor, armvert
    lastrobotPosition = [-1, -1, -1]
    #rotation, armvert, armhor, clawhor, clawvert
    startposition = [0.5, -0.03, 0.05, 0, 0.25]

    #calibration values
    idlerotate = 0.015
    imageMid = 320
    rotatecheckdist = 0.1
    rotateDistMult = -0.0007
    verticalDistMult = -0.0001
    rotatespeed = 0.25
    rotatemult = -0.00005
    precisemult = 0.75
    precisearmmult = 0.01

    clawrotatemult = -0.00025
    clawrotatespeed = 0.25
    clawclosespeed = 1

    vertspeed = 0.1
    vertmult = -0.0002

    vertclawmult = 0.0002

    armoffset = 0.02
    armspeed = 0.25
    armmovementspeed = -0.005
    yoffset = -100

    clawclosedist = 1

    clawyankdist = -0.5

    leftmargin = 10
    rightmargin = 100
    vertoffset = 0

    grabsize = 180
    grabsizemargin = 15
    startmovesize = 40
    vertstartsize = 70

    fullclawrot = 1.5708

    armmax = 0.25

    clawWiggleDist = 0.1



    #math constants
    checkdist = 0.02 # distance between the 2 positions after the check rotation
    camerafov = 1.13446 # 65 degrees in radians
    radialbypixel = camerafov / 640 / 2 # 640 is the width of the camera image in pixels

    
    control.Rotate(startposition[0], 1)
    control.ArmVertical(startposition[1], 1)
    control.ArmHorizontal(startposition[2], 1)
    control.RotateClawHorizontal(startposition[3], 1)
    control.RotateClawVertical(startposition[4], 1)

    control.TightenClaw(0.7, 1)

    while control.armhorpos != control.GetArmHorizontal() and control.armvertpos != control.GetArmVertical() and control.rotation != control.GetRotation():
        robot.step(timestep)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        
        recognised = camera.getRecognitionObjects()
        recCount = 0
        recCount = len(recognised)
        lookingforid = False
       
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
                    

        if lockId == -1 or recCount <= 0:
            #wait for strawberry
            #control.Rotate(idlerotate)
            continue

        print("found object with id: " + str(lockId))
            
        position = getposition(lockId)    
        if position is None:
            lockId = -1
            continue  

        if(position[2] < startmovesize):
            print("object too small, waiting")
            robot.step(timestep)
            continue
        
        control.RotateClawVertical(0.25, 1)
        control.ArmHorizontal(0.1, 1)
        prikken = False

        while not prikken:
            position = getposition(lockId)
             
            if position is None:
                lockId = -1
                continue 

            if position[0] < imageMid - leftmargin or position[0] > imageMid + rightmargin:
                control.Rotate((position[0] + leftmargin - imageMid) * rotatemult, rotatespeed)
            else:
                control.StopRotate()

            if position[1] < imageMid + yoffset - vertoffset:
                #omlaag
                if control.GetArmHorizontal() < control.vertdist:
                    control.StopArmHorizontal()
                control.ArmHorizontal((position[1] - imageMid + yoffset) * verticalDistMult, vertspeed)
            elif position[1] > imageMid + yoffset + vertoffset:
                #omhoog
                if control.GetArmHorizontal() > control.vertdist:
                    control.StopArmHorizontal()
                control.ArmHorizontal((position[1] + vertoffset - imageMid + yoffset) * verticalDistMult, vertspeed)
            else:
                control.StopArmHorizontal()

            #TODO: change this to specificly check for strawberry size and not tagret size
            if position[2] >= grabsize - grabsizemargin and position[2] <= grabsize + grabsizemargin:
                prikken = True
                print("prikken!")

            robot.step(timestep)
        if lockId == -1:
            continue

        #precise movement to stay with strawberry
        godown = True

        while prikken:
            position = getposition(lockId)
             
            if position[0] < imageMid - leftmargin or position[0] > imageMid + rightmargin:
                control.Rotate((position[0] + leftmargin - imageMid) * rotatemult, rotatespeed)
            else:
                control.StopRotate()

            if position[1] < imageMid + yoffset - vertoffset:
                #omlaag
                if control.GetArmHorizontal() < control.vertdist:
                    control.StopArmHorizontal()
                control.ArmHorizontal((position[1] - imageMid + yoffset) * verticalDistMult, vertspeed)
            elif position[1] > imageMid + yoffset + vertoffset:
                #omhoog
                if control.GetArmHorizontal() > control.vertdist:
                    control.StopArmHorizontal()
                control.ArmHorizontal((position[1] + vertoffset - imageMid + yoffset) * verticalDistMult, vertspeed)
            else:
                control.StopArmHorizontal()

            if control.GetArmHorizontal() >= armmax and control.GetRotation() <= 0:
                print("arm max reached, stopping")
                prikken = False
                continue

            if control.clawrotpos == control.GetClawVertical():
                if godown:
                    control.RotateClawVertical(-clawWiggleDist, 10)
                else:
                    control.RotateClawVertical(clawWiggleDist, 10)
                godown = not godown
            robot.step(timestep)
        
        print("returning to start position")
        control.StopClawVertical()
        control.StopArmHorizontal()
        control.StopRotate()
        control.StopArmVertical()
        
        #move to start position
        control.ArmHorizontal(startposition[2] - control.GetArmHorizontal(), 1)
        control.Rotate(startposition[0] - control.GetRotation(), 1)
        control.ArmVertical(startposition[1] - control.GetArmVertical(), 1)
        control.RotateClawHorizontal(startposition[3] - control.GetClawHorizontal(), 1)
        control.RotateClawVertical(startposition[4] - control.GetClawVertical(), 1)

        while control.hordist != control.GetArmHorizontal() or control.vertdist != control.GetArmVertical() or control.rotation != control.GetRotation() or control.clawrotposhor != control.GetClawHorizontal() or control.clawrotpos != control.GetClawVertical():
            robot.step(timestep)

        print("finished moving to start position, resetting lockId")
        lockId = -1

#test()
#simplecontrol()
advancedcontrol()
#movingcontrol()
#targetcontrol()



# voorkeur aan klauw bewegen ipv arm
# statisch plukken op vaste locatie
# klauw omhoog draaien om er over heen te kunnen
# je kan om de aardbei heen draaien voor plukken
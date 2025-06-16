from controller import Robot

# uint32 angle
# uint32 height
# uint32 distance
# uint32 gripper_angle
# uint32 closing_angle

discrot = 0
discrotpos = 0
rotation = 0
armvert = 0
armvertpos = 0

vertdist = 0
armhor = 0
armhorpos = 0
hordist = 0

clawrot = 0
clawrotpos = 0
clawrotpossens = 0

clawrothor = 0
clawrotposhor = 0
clawrotpossenshor = 0



claw1 = 0
claw1pos = -1
claw2 = 0
claw2pos = 1
clawpos = -1

def init(robot):
    """
    Initializes the robot's servos
    """
    global discrot, armvert, armhor, clawrot, claw1, claw2, discrotpos, armvertpos, armhorpos, clawrothor, clawrotpossens, clawrotpossenshor, clawpos
    discrot = robot.getDevice("disc-rotate")
    discrotpos = robot.getDevice("discrot-pos")
    discrotpos.enable(50)  # Enable position sensor for disc rotation
    armvert = robot.getDevice("arm-vertical")
    armvertpos = robot.getDevice("armvert-pos")
    armvertpos.enable(50)  # Enable position sensor for arm vertical
    armhor = robot.getDevice("arm-horizontal")
    armhorpos = robot.getDevice("armhor-pos")
    armhorpos.enable(50)  # Enable position sensor for arm horizontal
    clawrot = robot.getDevice("claw-rotate")
    clawrotpossens = robot.getDevice("claw-rotatepos")
    clawrotpossens.enable(50)  # Enable position sensor for claw rotation
    clawrothor = robot.getDevice("claw-rotate-hor")
    clawrotpossenshor = robot.getDevice("claw-rotatepos-hor")
    clawrotpossenshor.enable(50)  # Enable position sensor for claw horizontal rotation
    claw1 = robot.getDevice("claw1")
    claw2 = robot.getDevice("claw2")
    clawpos = robot.getDevice("claw-position")
    clawpos.enable(50)  # Enable position sensor for claw tightness


def Rotate(Rad, speed=1):
    """
    Rotates the base of the robot by a given angle
    """
    global rotation, discrot
    rotation += Rad
    discrot.setVelocity(speed)
    discrot.setPosition(rotation)

def GetRotation():
    """
    Returns the current rotation of the base
    """
    global discrotpos
    return discrotpos.getValue()  # Get current position from position sensor

def StopRotate():
    """
    Stops the rotation of the base
    """
    global discrot, discrotpos, rotation
    discrot.setVelocity(0)
    rotation = discrotpos.getValue()  # Keep current position
    discrot.setPosition(rotation)  # Keep current position    

def SetRotateSpeed(speed):
    """
    Sets the speed of the rotation
    """
    global discrot
    discrot.setVelocity(speed)


def ArmVertical(Dist, speed=1):
    """
    Moves the arm vertically by a given distance in m
    """
    global vertdist, armvert, clawrotpos
    limit = -0.1

    tier1height = 0.4
    tier2height = 1
    if GetClawVertical() >= tier1height or clawrotpos >= tier1height:
        limit = -0.08
        if GetClawVertical() >= tier2height or clawrotpos >= tier2height:
            limit = -0.05

    vertdist += Dist

    if vertdist < limit:
        print("Arm vertical distance is too low, setting to limit")
        vertdist = limit

    armvert.setVelocity(speed)
    armvert.setPosition(vertdist)

def GetArmVertical():
    """
    Returns the current vertical position of the arm
    """
    global armvertpos
    return armvertpos.getValue()  # Get current position from position sensor

def StopArmVertical():
    """
    Stops the vertical movement of the arm
    """
    global armvert, armvertpos, vertdist
    armvert.setVelocity(0)
    vertdist = armvertpos.getValue()  # Keep current position
    armvert.setPosition(vertdist)  # Keep current position
    
def SetArmVerticalSpeed(speed):
    """
    Sets the speed of the vertical movement of the arm
    """
    global armvert
    armvert.setVelocity(speed)


def ArmHorizontal(Dist, speed=1):
    """
    Moves the arm horizontally by a given distance in m
    """
    #TODO: mag niet te ver naar binnen als de claw gedraaid is
    #limit 70 degrees = 1.22 rad
    global hordist, armhor, clawrotposhor
    
    limit = 0
    unsafe = 1.22

    if clawrotposhor >= unsafe or GetClawHorizontal() >= unsafe:
        limit = 0.05

    hordist += Dist
    if hordist < limit:
        hordist = limit

    armhor.setVelocity(speed)
    armhor.setPosition(hordist)

def GetArmHorizontal():
    """
    Returns the current horizontal position of the arm
    """
    global armhorpos
    return armhorpos.getValue()  # Get current position from position sensor

def StopArmHorizontal():
    """
    Stops the horizontal movement of the arm
    """
    global armhor, armhorpos, hordist
    armhor.setVelocity(0)
    hordist = armhorpos.getValue()  # Keep current position
    armhor.setPosition(hordist)  # Keep current position
    
def SetArmHorizontalSpeed(speed):
    """
    Sets the speed of the horizontal movement of the arm
    """
    global armhor
    armhor.setVelocity(speed)


def RotateClawVertical(Rad, speed=1):
    """
    Rotates the claw by a given angle
    """
    global clawrotpos, clawrot, vertdist
    limit = 1.57
    unsafe1 = -0.08
    unsafe2 = -0.05
    if GetArmVertical() <= unsafe2 or vertdist <= unsafe2:
        if GetArmVertical() <= unsafe1 or vertdist <= unsafe1:
            limit = 1
        else:
            limit = 0.4

    clawrotpos += Rad
    if clawrotpos > limit:
        print("Claw vertical position is too high, setting to limit")
        if clawrotpos > 0:
            clawrotpos = limit
        else:
            clawrotpos = -limit
    clawrot.setVelocity(speed)
    clawrot.setPosition(clawrotpos)

def GetClawVertical():
    """
    Returns the current vertical position of the claw
    """
    global clawrotpossens
    return clawrotpossens.getValue()  # Get current position from position sensor

def StopClawVertical():
    """
    Stops the vertical rotation of the claw
    """
    global clawrot, clawrotpos, clawrotpossens
    clawrot.setVelocity(0)
    clawrotpos = clawrotpossens.getValue()  # Keep current position
    clawrot.setPosition(clawrotpos)  # Keep current position
    
def SetClawVerticalSpeed(speed):
    """
    Sets the speed of the vertical rotation of the claw
    """
    global clawrot
    clawrot.setVelocity(speed)


def RotateClawHorizontal(Rad, speed=1):
    """
    Rotates the claw horizontally by a given angle
    """
    #TODO: mag niet als de arm volledig is ingetrokken
    # limit 70 degrees = 1.22 rad
    global clawrotposhor, clawrothor, hordist
    limit = 1.57
    unsafe = 0.05

    if GetArmHorizontal() < limit or hordist < limit:
        limit = 1.22


    clawrotposhor += Rad

    if clawrotposhor > limit or GetClawHorizontal() > limit:
        print("Claw horizontal position is too high, setting to limit")
        if clawrotposhor > 0:
            clawrotposhor = limit
        else:
            clawrotposhor = -limit

    clawrothor.setVelocity(speed)
    clawrothor.setPosition(clawrotposhor)

def GetClawHorizontal():
    """
    Returns the current horizontal position of the claw
    """
    global clawrotpossenshor
    return clawrotpossenshor.getValue()  # Get current position from position sensor

def stopClawHorizontal():
    """
    Stops the horizontal rotation of the claw
    """
    global clawrothor, clawrotposhor, clawrotpossenshor
    clawrothor.setVelocity(0)
    clawrotposhor = clawrotpossenshor.getValue()  # Keep current position
    clawrothor.setPosition(clawrotposhor)  # Keep current position

def SetClawHorizontalSpeed(speed):
    """
    Sets the speed of the horizontal rotation of the claw
    """
    global clawrothor
    clawrothor.setVelocity(speed)

def TightenClaw(Rad, speed=1):
    """
    Tightens the claw by a given angle
    """
    global claw1pos, claw2pos, claw1, claw2
    claw1pos += Rad
    claw2pos -= Rad
    if claw1pos > 0:
        print("claw1pos is too low, setting to 0")
        claw1pos = 0
    if claw2pos < 0:
        print("claw2pos is too high, setting to 0")
        claw2pos = 0
    claw1.setVelocity(speed)
    claw1.setPosition(claw1pos)
    claw2.setVelocity(speed)
    claw2.setPosition(claw2pos)

def GetClawTightness():
    """
    Returns the current tightness of the claw
    """
    global clawpos
    return clawpos.getValue()  # Assuming both claws are at the same position

def OpenClaw(speed=1):
    """
    fully Opens the claw
    """
    global claw1pos, claw2pos, claw1, claw2
    claw1pos = -1
    claw2pos = 1
    claw1.setVelocity(speed)
    claw1.setPosition(claw1pos)
    claw2.setVelocity(speed)
    claw2.setPosition(claw2pos)
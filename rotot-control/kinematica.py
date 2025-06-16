import servocontrol as Servos

# uint32 angle
# uint32 height
# uint32 distance
# uint32 gripper_angle
# uint32 closing_angle



def LookAtPoint(x, y):
    """
    Looks at a point, input is a 2d point
    """
    #TODO: this one is the hardest
    NotImplementedError("lookatpoint not implemented")

def MoveArmVertical(Dist, speed=1):
    """
    Moves the arm vertically by a given distane
    """
    Servos.ArmVertical(Dist, speed)

def MoveArmHorizontal(Dist, speed=1):
    """
    Moves the arm horizontally by a given distance
    """
    Servos.ArmHorizontal(Dist, speed)

def Rotate(Rad, speed=1):
    """
    Rotates the robot by a given angle
    """
    Servos.Rotate(Rad, speed)

def RotateDeg(Deg, speed=1):
    """
    Rotates the robot by a given angle
    """
    Rad = Deg * (3.14 / 180)
    Rotate(Rad, speed)

def Grab(Rad, speed=1):
    """
    Grabs an object
    """
    Servos.TightenClaw(Rad, speed)

def GrabDeg(Deg, speed=1):
    """
    Grabs an object by a given angle in degrees
    """
    Rad = Deg * (3.14 / 180)
    Grab(Rad, speed)

def Release(speed=1):
    """
    Releases an object
    """
    Servos.OpenClaw(speed)

def RotateGrabber(Rad, speed=1):
    """
    Rotates the grabber by a given angle
    """
    Servos.claw_rotate(Rad, speed)

def RotateGrabberDeg(Deg, speed=1):
    """
    Rotates the grabber by a given angle in degrees
    """
    Rad = Deg * (3.14 / 180)
    RotateGrabber(Rad, speed)
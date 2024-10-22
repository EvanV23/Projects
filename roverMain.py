from qset_lib import Rover, AngleReader
from time import sleep
import math
import signal

# flag = 0, not on m-line and not angled at end coordinates, its so over
# flag = 1, on m-line, not angled towards end coordinates
# flag = 2, on m-line, angled toward coordinates, good to go baby
# flag = 3, done asf, were sp back
flag = 1

# initializes rover object to connect code to gazebo
rover = Rover()
angle_reader = AngleReader()

#changes left and right wheel speed
def changeSpeed(l, r):
    rover.send_command(l, r)

# turns rover left
def turnL():
    changeSpeed(-1, 1)

# turns rover right
def turnR():
    changeSpeed(1, -1)

def move():
    changeSpeed(3, 3)

def head(difference, heading2):
    # loop which repeats until the difference in heading is less than 2 degrees
    while (difference>0.4):
        if(heading2>=0):
            turnL()
        else:
            turnR()
        currentHead = rover.heading
        difference = math.fabs(currentHead - heading2)

        sleep(0.001)

def checkM(currentX, currentY, slope):
    # this function will be called during the obstacle avoidance "mode"
    if math.fabs(currentY - slope * currentX) < 0.2:
        flag = 1
        # 0 will be used to exit out of the obstacle
        # avoidance mode and instead start turning towards the m-line
    else:
        flag = 0
        # 1 will be used to continue looping through the obstacle avoidance function

# function which checks the minimum distance in all LiDar regions and updates if the rover is too close to an object
def tooClose():
    for dist in rover.laser_distances:
        if dist < 0.5:
            print("TOO CLOSE")
            changeSpeed(0, 0)
            return 1
        else:
            return 0

def avoid(x2, y2):

    # turns the rover 90 degrees by using the negative reciprocal of the final coordinates
    heading2 = ((math.atan2(-(x2 - rover.x), y2 - rover.y))*360)/6.282
    difference = math.fabs(heading2 - rover.heading)
    head(difference, heading2)

    while True:

        # calculates a new goal heading every 0.01 seconds
        heading2 = ((math.atan2(y2-rover.y, x2-rover.x))*360)/6.282
        print("updated goal heading: " + str(heading2) + " actual heading: " + str(rover.heading))
        # exits loop if the difference between the rover heading and the updated goal heading is small
        if math.fabs(heading2-rover.heading) < 0.4:
            changeSpeed(0,0)
            break

        regions = updateRegions()

        # moves forward if the leftmost LiDar readings are small
        if regions[4] < 0.5:
            move()
        # turns left if the leftmost LiDar beams are large or do not read anything
        elif regions[4] == float('inf') or regions[4]>2:
            turnL()
        # moves forward all other cases
        else:
            move()
        sleep(0.01)
    # assures that the rover is facing the new desired heading before moving again
    head(math.fabs(heading2-rover.heading), heading2)

def updateRegions():
    regions = [
        min(rover.laser_distances[1:5]),  # Right
        min(rover.laser_distances[6:11]),  # Front Right
        min(rover.laser_distances[12:17]),  # Front
        min(rover.laser_distances[18:23]),  # Front Left
        min(rover.laser_distances[24:28]),  # Left
    ]
    return regions

def main():

    signal.signal(signal.SIGINT, signal.default_int_handler)

    # declares final x and y coordinates
    x2 = 7
    y2 = 7

    # assigns initial x and y coordinates to variables
    initx = rover.x
    inity = rover.y
    # calculates required heading to turn to
    heading2 = ((math.atan2(y2, x2))*360)/6.282
    # finds initial rover heading
    initHead = rover.heading
    # finds absolute value of the difference between the current rover heading and the final heading
    difference = math.fabs(heading2 - initHead)

    try:
        changeSpeed(0, 0)
        sleep(1)
        # calls function to turn
        head(difference, heading2)
        # loop which moves the rover forward until it reaches the final set coordinates
        flag = 2

        while True:
            currentX = rover.x
            currentY = rover.y
            currentHead = rover.heading

            # command robot to move forward
            move()

            # check if rover is too close to an obstacle
            for dist in rover.laser_distances:
                if dist < 0.5:
                    print("TOO CLOSE")
                    changeSpeed(0,0)
                    avoid(x2, y2)
                    print("exited!!!!!!!!!!!!!!!!!!!!!!")


            # calculates if the rovers current position is within a certain distance to the final coordinate
            if math.fabs(x2-currentX) < 0.5 and math.fabs(y2-currentY) < 0.5:
                changeSpeed(0,0)
                break

            print("X: " + str(currentX) + " Y: " + str(currentY) + " Heading: " + str(currentHead))
            sleep(0.01)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()



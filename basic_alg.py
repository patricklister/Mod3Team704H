#start of code snippets


import rospy  # this is the module required for all simulation communication
import math
import time
import datetime

# start of wheel control code
from wheel_control.msg import wheelSpeed  # this is a required module for the drive communication


rospy.init_node("controller")

class WheelController:

    def __init__(self):
        self.wheel_pub = rospy.Publisher("/gazebo_wheelControl/wheelSpeedTopic", wheelSpeed, queue_size=1)

    def drive_wheels(self, left, right):
        # type: (float, float) -> None
        # left and right are numbers between -1 and 1
        msg = wheelSpeed()
        msg.left = left
        msg.right = right
        msg.wheelMode = 0
        self.wheel_pub.publish(msg)
        #print(msg)
# end of wheel control code


# start of laser scan code
from sensor_msgs.msg import LaserScan

class LaserListener:

    def __init__(self):
        self.laserSub = rospy.Subscriber("/leddar/leddarData", LaserScan, self.laser_callback, queue_size=1)
        self.laserRanges = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]

    def laser_callback(self, msg):
        # type: (LaserScan) -> None
        self.laserRanges = msg.ranges

# end of laser scan code access laserRanges for an array of all measured distances from the laser sensors


# start of localization stuff
from geometry_msgs.msg import Point
from std_msgs.msg import Float32


class LocationHeading:

    def __init__(self):
        self.fixSub = rospy.Subscriber("/fix/metres", Point, self.fix_callback, queue_size=1)
        self.headingSub = rospy.Subscriber("/heading",Float32, self.heading_callback, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading = 0.0

    def fix_callback(self, msg):
        # type: (Point) -> None
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def heading_callback(self, msg):
        # type: (Float32) -> None
        self.heading = msg.data

# end of localization stuff

class Rover:
    def __init__(self, destCoor, locationHeadingObj, wheelControlObj):


        self.destCoor = destCoor
        self.tempDestCoor = [0,0]

        self.locationHeadingObj = locationHeadingObj
        self.wheelControlObj = wheelControlObj

        self.turnErrorThreshhold = 0.01

    def face_goal(self):
        deltaX = self.destCoor[0] - self.locationHeadingObj.x
        deltaY = self.destCoor[1] - self.locationHeadingObj.y

        if deltaX > 0 and deltaY > 0:         #quad 1
            RAA = math.atan(abs(deltaY)/abs(deltaX))
            angle = RAA

        elif deltaX < 0 and deltaY > 0:         #quad 2
            RAA = math.atan(abs(deltaY)/abs(deltaX))
            angle = math.pi - RAA
            
        elif deltaX < 0 and deltaY < 0:         #quad 3
            RAA = math.atan(abs(deltaY)/abs(deltaX))
            angle = math.pi + RAA

        elif deltaX > 0 and deltaY < 0:         #quad 4
            RAA = math.atan(abs(deltaY)/abs(deltaX))
            angle = 2*(math.pi) - RAA

        elif deltaX == 0 and deltaY > 0:    #goal above it
            angle = (math.pi)/2   #90 deg from postive x-axis
        
        elif deltaX == 0 and deltaY < 0:   #goal below it
            angle = (3*(math.pi))/2   #270 deg from postive x-axis

        elif deltaX > 0 and deltaY == 0:   #goal to right of it
            angle = 0

        elif deltaX < 0 and deltaY == 0:    #goal to left of it
            angle = math.pi     #180 deg from postive x-axis

        elif deltaX == 0 and deltaY == 0:
            return True     #YOU ALREADY AT GOAL

        maxAngle = angle + self.turnErrorThreshhold
        minAngle = angle - self.turnErrorThreshhold

        print(RAA)
        print(angle)
        print(self.locationHeadingObj.heading)
        print(str(minAngle) + " - " + str(maxAngle))

        while self.locationHeadingObj.heading > maxAngle or self.locationHeadingObj.heading < minAngle:
            print("HEY!")
            self.wheelControlObj.drive_wheels(-0.5, 0.5)
            print(self.locationHeadingObj.heading)
            print(str(minAngle) + " - " + str(maxAngle))



    def go_straight_dist (self, distance, forOrBack):
        initCoor = [self.locationHeadingObj.x, self.locationHeadingObj.y]

        distanceTravelled = 0

        speed = 0.25

        while distanceTravelled <= distance and not rospy.is_shutdown():

            if forOrBack == "f":
                velocity = speed
            elif forOrBack == "b":
                velocity = (-1)*speed


            self.wheelControlObj.drive_wheels(velocity, velocity)

            newCoor = [self.locationHeadingObj.x, self.locationHeadingObj.y]
            

            distanceTravelled = ((newCoor[0]-initCoor[0])**2 + (newCoor[1]-initCoor[1])**2)**(0.5)


            print(str(initCoor) + "   " + str(newCoor))
            print(str(distanceTravelled) + ">" + str(distance))




        self.wheelControlObj.drive_wheels(0,0)

        time.sleep(5)

        newCoor = [self.locationHeadingObj.x, self.locationHeadingObj.y]
        distanceTravelled = ((newCoor[0]-initCoor[0])**2 + (newCoor[1]-initCoor[1])**2)**(0.5)

        print(distanceTravelled)



    def rotate(self, desiredAngle):
        if desiredAngle < 90:
            desiredHeading = self.locationHeadingObj.heading - (desiredAngle*((math.pi)/180))
            rw = 0.5
            lw = -0.5

        elif desiredAngle == 90:
            desiredHeading = self.locationHeadingObj.heading
            return True

        elif desiredAngle > 90:
            desiredHeading = self.locationHeadingObj.heading + ((desiredAngle-90)*((math.pi)/180))
            rw = -0.5
            lw = 0.5

        print(self.locationHeadingObj.heading)
        maxHeading = desiredHeading + self.turnErrorThreshhold
        minHeading = desiredHeading - self.turnErrorThreshhold

        print(desiredHeading)
        print(str(minHeading) + " - " + str(maxHeading))             
        
        while (self.locationHeadingObj.heading < minHeading or self.locationHeadingObj.heading > maxHeading) and not rospy.is_shutdown():
            self.wheelControlObj.drive_wheels(rw, lw)

        print("***")
        print(self.locationHeadingObj.heading)

        self.wheelControlObj.drive_wheels(0, 0)

    
    """
    # pass in a magnitude and heading to get the desired movement vector
    # Add this vector to the current coordinates to get the desired coordinates
    def get_desired_vector (self, magnitude, heading):
        vector = [magnitude * math.sin(heading), magnitude * math.cos(heading)]
        return vector

    def rotate (self, desiredHeading):
        # Get the rover's current heading
        currentHeading = self.locationHeadingObj.heading

        # If the current heading is in the desired direction within error, return True
        if (abs(currentHeading - desiredHeading) <= self.turnErrorThreshhold):
            turnIncrement = 1
            lastTurnDirection = 0
            return True
        
        # Get the current desired turn direction (>0 is left, <0 is right)
        currentTurnDirection = (desiredHeading - currentHeading - math.pi)/abs(desiredHeading - currentHeading - math.pi)

        # If turning a different direction than last time, halve the speed at which it turns
        if currentTurnDirection != lastTurnDirection and lastTurnDirection != 0:
            turnIncrement /= 2

        self.wheelControlObj.drivewheels(-1 * currentTurnDirection * turnIncrement, currentTurnDirection * turnIncrement)

        lastTurnDirection = currentTurnDirection

        return False
    """
        
        


#initiallize classes to get and send data to gazebo
locHead  = LocationHeading()
laser = LaserListener()
wheel = WheelController()

xDestCoor = float(input("Enter goal coordinates below...\nX Coordinate: "))
yDestCoor = float(input("Y Coordinate: "))
print("\n")

roverObj = Rover([xDestCoor, yDestCoor], locHead, wheel)
#end of initialization

#####ENSURE ROVER IS AT Z AXIS OF ZERO OR ELSE IT FLIPS OUT


# start of control loop snippet


while not rospy.is_shutdown():  #this will run until gazebo is shut down or CTRL+C is pressed in the ubuntu window that is running this code

    wheel.drive_wheels(0, 0)
    time.sleep(1)
    
    print("Loading...")
    time.sleep(5)

    roverObj.face_goal()

    """
    #roverObj.go_straight()
    print(laser.laserRanges)

    nearestObjectCentre = (laser.laserRanges[7] + laser.laserRanges[8])/2

    if nearestObjectCentre > 2:
        print("TIME TO TURN")
    else:
        wheel.drive_wheels(1,1)
        #roverObj.go_straight("f")
    """
    

    """
    minRange = 99 #initialize minRange to a value larger than what will be recieved
    for x in range(0, 15): #iterate through the ranges list
        if laser.laserRanges[x] < minRange: #if the current range is smaller than the smallest know range
            minRange = laser.laserRanges[x] #update the range
    if minRange < 3: #if there is something closer than 3m infront of the rover
        wheel.drive_wheels(1, -1) #turn
    else:
        wheel.drive_wheels(1, 1) #go staright
    print("Current Heading: ", locHead.heading, "Current x val: ", locHead.x, "RightMostLaser: ", laser.laserRanges[0]) #print some random data to the command line
    """

# end of control loop snippet


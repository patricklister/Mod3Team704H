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

        self.finalCoords = [0, 0]
        self.startCoords = [0, 0]

    def face_goal(self):
        deltaX = self.destCoor[0] - self.locationHeadingObj.x
        deltaY = self.destCoor[1] - self.locationHeadingObj.y
        RAA = 0


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



        direction = 0 #1 is CW, -1 is CCW
        if self.locationHeadingObj.heading < angle:
            if abs(self.locationHeadingObj.heading - angle) < 180:
                direction = -1
            else:
                direction = 1

        else:
            if abs(self.locationHeadingObj.heading - angle) < 180:
                direction = 1
            else:
                direction = -1

        if direction == 1:  #CW
            lw = 0.7
            rw = -0.7
        else:               #CCW
            lw = -0.7
            rw = 0.7

        print(direction)

        maxAngle = angle + self.turnErrorThreshhold
        minAngle = angle - self.turnErrorThreshhold

        print(RAA)
        print(angle)
        print(self.locationHeadingObj.heading)
        print(str(minAngle) + " - " + str(maxAngle))

        
        while (self.locationHeadingObj.heading > maxAngle or self.locationHeadingObj.heading < minAngle) and not rospy.is_shutdown():
            self.wheelControlObj.drive_wheels(lw, rw)

            print("h")
            """
            print(self.locationHeadingObj.heading)
            print(str(minAngle) + " - " + str(maxAngle))
            """
        
        self.wheelControlObj.drive_wheels(0,0)




    def go_straight_dist (self, distance, forOrBack):
        """initCoor = [self.locationHeadingObj.x, self.locationHeadingObj.y]

        distanceTravelled = 0

        speed = 0.25

        while (distanceTravelled <= distance) and not rospy.is_shutdown():

            if forOrBack == "f":
                velocity = speed
            elif forOrBack == "b":
                velocity = (-1)*speed


            self.wheelControlObj.drive_wheels(velocity, velocity)

            newCoor = [self.locationHeadingObj.x, self.locationHeadingObj.y]
            

            distanceTravelled = ((newCoor[0]-initCoor[0])**2 + (newCoor[1]-initCoor[1])**2)**(0.5)

            """
            print(str(initCoor) + "   " + str(newCoor))
            print(str(distanceTravelled) + ">" + str(distance))
            """



        self.wheelControlObj.drive_wheels(0,0)

        time.sleep(5)

        newCoor = [self.locationHeadingObj.x, self.locationHeadingObj.y]
        distanceTravelled = ((newCoor[0]-initCoor[0])**2 + (newCoor[1]-initCoor[1])**2)**(0.5)

        print(distanceTravelled)"""

        self.startCoords = [self.locationHeadingObj.x, self.locationHeadingObj.y]
        self.finalCoords = [startCoords[0] + Mathf.sin(self.locationHeadingObj.heading), startCoords[1] + Mathf.cos(self.locationHeadingObj.heading)]

        arrived = False

        while not arrived and not rospy.is_shutdown():
            if forOrBack == "f":
                    velocity = speed
            elif forOrBack == "b":
                velocity = (-1)*speed

            self.wheelControlObj.drive_wheels(velocity, velocity)

            if (Mathf.abs(self.locationHeadingObj.x) > Mathf.abs(finalCoords[0]) and Mathf.abs(self.locationHeadingObj.y) > Mathf.abs(finalCoords[1])):
                arrived = True
        
        self.wheelControlObj.drive_wheels(0,0)

        self.startCoords = [0, 0]
        self.finalCoords = [0, 0]



    def calc_angle(self, laserRangeIndex):
        angle = (math.pi/4) + (math.pi/2)*laserRangeIndex

        return angle


    def rotate(self, desiredAngle):
        if desiredAngle < 90:
            desiredHeading = self.locationHeadingObj.heading - (desiredAngle*((math.pi)/180))
            lw = 1
            rw = -1

        elif desiredAngle == 90:
            desiredHeading = self.locationHeadingObj.heading
            return True

        elif desiredAngle > 90:
            desiredHeading = self.locationHeadingObj.heading + ((desiredAngle-90)*((math.pi)/180))
            lw = -1
            rw = 1

        print(self.locationHeadingObj.heading)
        maxHeading = desiredHeading + self.turnErrorThreshhold
        minHeading = desiredHeading - self.turnErrorThreshhold

        print(desiredHeading)
        print(str(minHeading) + " - " + str(maxHeading))             
        
        while (self.locationHeadingObj.heading < minHeading or self.locationHeadingObj.heading > maxHeading) and not rospy.is_shutdown():
            self.wheelControlObj.drive_wheels(lw, rw)

        print("***")
        print(self.locationHeadingObj.heading)

        self.wheelControlObj.drive_wheels(0, 0)



    def get_xy(self, laserRanges):
        outputList = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

        comparisonValue = laserRanges[0]
        currentAngle = math.pi/4
        for i in range(16):
            currentAngle = (math.pi/4) + (math.pi/30)*i

            if laserRanges[i] == float("inf"):
                newListItem = [float("inf"), float("inf")]

            else:
                dist = laserRanges[i]

                if currentAngle > (math.pi/2):
                    tempAngle = (math.pi) - currentAngle
                    xComponent = (abs(dist*(math.cos(tempAngle))))*-1
                    yComponent = abs(dist*(math.sin(tempAngle)))

                else:
                    xComponent = abs(dist*(math.cos(currentAngle)))
                    yComponent = abs(dist*(math.sin(currentAngle)))

                newListItem = [xComponent, yComponent]

            #print(i, currentAngle)
            
            outputList[i] = newListItem
            
        
        return outputList


            


    
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

time.sleep(2)
    
wheel.drive_wheels(0, 0)

xDestCoor = float(input("Enter goal coordinates below...\nX Coordinate: "))
yDestCoor = float(input("Y Coordinate: "))
print("\n")

roverObj = Rover([xDestCoor, yDestCoor], locHead, wheel)

#end of initialization

#####ENSURE ROVER IS AT Z AXIS OF ZERO OR ELSE IT FLIPS OUT


# start of control loop snippet

print("Loading...")
time.sleep(3)

roverObj.face_goal()

while not rospy.is_shutdown():  #this will run until gazebo is shut down or CTRL+C is pressed in the ubuntu window that is running this code
    roverObj.wheelControlObj.drive_wheels(0,0)

    roverObj.face_goal()

    laserValues= laser.laserRanges
    laserCoor = roverObj.get_xy(laserValues)

    possiblePath = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1] #1 means no, 0 means yes it is clear
    zeroCount = 0
    oneCount = 16
    infCount = 0
    for i in range(16):
        if laserValues[i] == float("inf") or laserValues[i] > 3:
            possiblePath[i] = 0
            zeroCount += 1
            oneCount -= 1

        if laserValues[i] == float("inf"):
            infCount += 1

    print("***************")
    print(oneCount)

    if zeroCount != 16:

        if oneCount == 0:
            go_straight_dist(1, "b")

        else:

            print(laserValues)
            print(possiblePath)

            pathCount = 0
            bestPathCount = 0
            minIndexBest = 0
            maxIndexBest = 0
            for i in range(16):
                if possiblePath[i] == 0 and i != 15:
                    pathCount += 1
                else:
                    if i != 0:
                        deltaX = abs(laserCoor[i-1][0] - laserCoor[i-pathCount][0])
                        deltaY = abs(laserCoor[i-1][0] - laserCoor[i-pathCount][0])

                        
                        print("dx: " + str(deltaX) + ", pathCount: " + str(pathCount))
                        
                        openSpace = False
                        if not(deltaX == float("nan")):
                            openSpace = True

                        if deltaX > 2 or openSpace:
                            for j in range(pathCount):
                                possiblePath[(i-1)-j] = 0

                        else:
                            for j in range(pathCount):
                                possiblePath[(i-1)-j] = 1
                        
                        if bestPathCount < pathCount:
                            bestPathCount = pathCount
                            maxIndexBest = i
                            minIndexBest = (i-1)-pathCount

                        pathCount = 0
            
            print(possiblePath)

            print("-------")

            idealIndexDirection = round(((maxIndexBest+minIndexBest)/2),2)

            print(idealIndexDirection)

            angle = roverObj.calc_angle(idealIndexDirection)
            
            time.sleep(1)
            roverObj.rotate(angle)
            time.sleep(1)

            roverObj.go_straight_dist((laserValues[minIndexBest]+laserValues[maxIndexBest])/2, "f")


    else:
        roverObj.wheelControlObj.drive_wheels(1, 1)

        time.sleep(0.5)
    
    roverObj.wheelControlObj.drive_wheels(0, 0)

    time.sleep(5)


    # lastX = 0
    # for i in range(16):
    #     if (i+1) <= 15:
    #         if laserValues[i] == float("inf"):
    #             lastX = 0
    #         else:
                
                
    #             if laserValues[i+1] == float("inf"):
                    
    #             deltaX = laserValues[i+1] - laserValues[i]
                
    #             if deltaX > 1.5:




    


    """
    wheel.drive_wheels(0,0)

    roverObj.face_goal()

    currentLaser = laser.laserRanges
    print(currentLaser)
    print(type(currentLaser[0]))

    isPathClear = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1] #1 means no, 0 means yes it is clear

    nearestObjectCentre = (currentLaser[7] + currentLaser[8])/2

    adjust = False
    for i in currentLaser:
        if i <= 5 and i != float('inf'):
            adjust = True
            break


    distanceToGoal = ((xDestCoor-roverObj.locationHeadingObj.x)**2 + (yDestCoor-roverObj.locationHeadingObj.y)**2)**(0.5)
    if 





    if adjust:
        indexL = 6
        deviationsL = 0
        while indexL >= 0:
            diff = currentLaser[indexL] - nearestObjectCentre
            if diff > 1:
                isPathClear[indexL] = 0
                break
            else:
                deviationsL += 1

            indexL -= 1
            
        indexR = 9
        deviationsR = 0
        while indexR <= 15:
            diff = currentLaser[indexR] - nearestObjectCentre
            if diff > 1:
                isPathClear[indexR] = 0
                break
            else:
                deviationsR += 1

            indexR += 1

        angle = 0
        if deviationsL < deviationsR:
            angle = roverObj.calc_angle(indexL)
        elif deviationsR < deviationsL:
            angle = roverObj.calc_angle(indexR)
        

        time.sleep(5)

        if angle != 0:
            roverObj.rotate(angle)

            roverObj.go_straight_dist(nearestObjectCentre, "f")

        else:
            roverObj.rotate(math.pi/2)
            roverObj.go_straight_dist(2, "f")

        
        roverObj.face_goal()



    else:
        wheel.drive_wheels(0.5, 0.5)

    time.sleep(0.01)

            
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


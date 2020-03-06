#start of code snippets


import rospy  # this is the module required for all simulation communication

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
        self.currCoor = [locationHeadingObj.x, locationHeadingObj.y]
        self.currAngle = locationHeadingObj.heading
        self.destCoor = destCor
        self.tempDestCoor = [0,0]

        self.locationHeadingObj = locationHeadingObj
        self.wheelControlObj = wheelControlObj


    def go_straight (self, distance, forOrBack):
        initCoor = [self.currCoor[0], self.currCoor[1]]

        distanceTravelled = 0

        while distanceTravelled > distance:
            if forOrBack == "f":
                wheelControlObj.drive_wheels(1,1)
            elif forOrBack == "b":
                wheelControlObj.drive_wheels(-1,-1)

            newCoor = [self.currCoor[0], self.currCoor[1]]
            distanceTravelled = ( (newCoor[0]-initCoor[0])^2 + (newCoor[1]-initCoor[0])^2 )^(1/2)

        wheelControlObj.drive_wheels(0,0)



        
        


#initiallize classes to get and send data to gazebo
locHead  = LocationHeading()
laser = LaserListener()
wheel = WheelController()

rover = ([0,0], locHead, wheel)
#end of initialization



# start of control loop snippet

while not rospy.is_shutdown():  #this will run until gazebo is shut down or CTRL+C is pressed in the ubuntu window that is running this code
    
    rover.go_straight(10, "b")
    minRange = 99 #initialize minRange to a value larger than what will be recieved
    for x in range(0, 15): #iterate through the ranges list
        if laser.laserRanges[x] < minRange: #if the current range is smaller than the smallest know range
            minRange = laser.laserRanges[x] #update the range
    if minRange < 3: #if there is something closer than 3m infront of the rover
        wheel.drive_wheels(1, -1) #turn
    else:
        wheel.drive_wheels(1, 1) #go staright
    print("Current Heading: ", locHead.heading, "Current x val: ", locHead.x, "RightMostLaser: ", laser.laserRanges[0]) #print some random data to the command line

# end of control loop snippet


#!/usr/bin/env python

# RIGHT EDGE FOLLOWING (FUZZY LOGIC) 

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TrapFunc:  
    
    def __init__(self, Points, label):
        
        self.points = Points
        self.linguistic_label = label 
        
    def getMemberValue(self, input_value):  # to get membership value
        if input_value < self.points[0] or input_value > self.points[3]:
            return 0.0
        elif input_value >= self.points[0] and input_value < self.points[1]:
            return(input_value - self.points[0])/(self.points[1] - self.points[0])
        elif input_value >= self.points[1] and input_value <= self.points[2]:
            return 1.0
        elif input_value > self.points[2] and input_value < self.points[3]:
            return (self.points[3] - input_value) / (self.points[3] - self.points[2])
        
        return 0.0



class rule: 
    
    def __init__(self, i, o):
        self.inputs = i
        self.outputs = o
    
    def getFir(self, list_Values): # to get firing strength
        list_memValues = []
        for i in range(len(self.inputs)):
            list_memValues.append(list_Values[i][self.inputs[i]])
        return min(list_memValues)

#            RULEBASE 

#            [RFS  RBS]       [ TURNING, SPEED]
r1 = rule(['close', 'close'], ['left', 'slow'])
r2 = rule(['close', 'med'],   ['left', 'slow'])
r3 = rule(['close', 'far'],   ['left', 'med'])
r4 = rule(['med', 'close'],   ['right', 'med'])
r5 = rule(['med', 'med'],     ['med', 'med'])
r6 = rule(['med', 'far'],     ['left', 'med'])
r7 = rule(['far', 'close'],   ['right', 'slow'])
r8 = rule(['far', 'med'],     ['right', 'slow'])
r9 = rule(['far', 'far'],     ['right', 'slow'])

# Setting initial values of the sensors

RB_dist = 0.0   
RF_dist = 0.0 

# assigning range to close, medium and fast

close_RB_Dist = TrapFunc([0.0,0.0,0.25,0.5], 'close')
close_RF_Dist = TrapFunc([0.0,0.0,0.25,0.5], 'close')

med_RB_Dist = TrapFunc([0.25,0.5,0.5,0.75], 'med')
med_RF_Dist = TrapFunc([0.25,0.5,0.5,0.75], 'med')

far_RB_Dist = TrapFunc([0.5, 0.75, 1.0 ,1.0], 'far')
far_RF_Dist = TrapFunc([0.5, 0.75, 1.0 ,1.0], 'far')


turn_out = [1, 1, 0.5, -0.5, 0, 0.5, -1, -1, -0.5]

vel_out = [0.2, 0.2, 0.3, 0.3, 0.3, 0.3, 0.2, 0.3, 0.4]

output_vel = {}
output_vel['slow'] = 0.05
output_vel['med'] = 0.2
output_vel['fast'] = 0.4

turning = {}
turning['right'] = -0.5
turning['med'] = 0
turning['left'] = 0.5

lValues = [output_vel, turning]

class rulebase :
    
    def __init__(self, rules):
        self.rules = rules
        
    def Defuz(self, fs, turn, vel):
        
        
        speed = []
        for i in range(len(fs)):
            a = fs[i] * vel[i]
            speed.append(a)
        speed = sum(speed)/sum(fs)
        print(speed)

        turning = []
        for i in range(len(fs)):
            b = fs[i] * turn[i]
            turning.append(b)
        turning = sum(turning)/sum(fs)
        print(turning)

        return(speed,turning)


r_b = rulebase([])

def callback(msg):
    global RB_dist 
    global RF_dist 
    RB_dist  =  msg.ranges[540]   
    RF_dist  =  min([msg.ranges[600], msg.ranges[580]])
    if RB_dist > 1.0 :
        RB_dist = 1.0
    if RF_dist > 1.0 :
        RF_dist = 1.0
        
    rospy.loginfo("RBS %f RFS %f",RB_dist, RF_dist)


sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)


# function to control velocity of robot
def forwards(speed, turn):
	global pub
	vel_x = Twist() # initiates twist message
	vel_x.linear.x = speed  # out_sets linear speed of robot
	vel_x.angular.z = turn  # out_sets angle to turn to pid function output (turns left?)
	pub.publish(vel_x) #publishes velocity
    


def controller():# sending linear and angular velocity after processing

    global RB_dist 
    global RF_dist 
    global close_RBinput
    global close_RFinput
    global med_RBinput
    global med_RFinput
    global far_RBinput
    global far_RFinput
    global r_b
    global turn_out
    global vel_out
    
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if RF_dist != 0 and RB_dist != 0:
            RBS = {}
            RBS['close'] = close_RB_Dist.getMemberValue(RB_dist)
            RBS['med'] = med_RB_Dist.getMemberValue(RB_dist)
            RBS['far'] = far_RB_Dist.getMemberValue(RB_dist)
            RFS = {}
            RFS['close'] = close_RF_Dist.getMemberValue(RF_dist)
            RFS['med'] = med_RF_Dist.getMemberValue(RF_dist)
            RFS['far'] = far_RF_Dist.getMemberValue(RF_dist)   
            list_Values = [RFS, RBS]

            rulebase = [r1.getFir(list_Values),r2.getFir(list_Values),r3.getFir(list_Values),r4.getFir(list_Values),
                        r5.getFir(list_Values),r6.getFir(list_Values),r7.getFir(list_Values),r8.getFir(list_Values),r9.getFir(list_Values)]

            fs= []
            vel =[]
            turn =[]
            for i in range(len(rulebase)):
                if rulebase[i] > 0: # firing strenght > 0
                    fs.append(rulebase[i])
                    vel.append(vel_out[i])
                    turn.append(turn_out[i])

            velocity , turning  = r_b.Defuz(fs,turn ,vel)
            #print ("turning %f vel %f",turning,velocity)
            forwards(velocity ,turning)
        rate.sleep()

def stop() :
    vel = Twist()
    vel.linear.x = 0.0 
    vel.angular.z = 0.0 
    pub.publish(vel)

rospy.on_shutdown(stop)

if __name__ == '__main__':
    try:
        rospy.init_node("fuzzy_node",anonymous = True)
        controller()

    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python

#  FUZZY SUBSUMPTION
  

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# OBSTACLE AVOIDANCE CODE 

class TrapFunc1:
    
    def __init__(self, Points, label):
        
        self.points = Points
        self.linguistic_label = label 
        
    def getMemberValue(self, input_value): # to get membership value
        if input_value < self.points[0] or input_value > self.points[3]:
            return 0.0
        elif input_value >= self.points[0] and input_value < self.points[1]:
            return(input_value - self.points[0])/(self.points[1] - self.points[0])
        elif input_value >= self.points[1] and input_value <= self.points[2]:
            return 1.0
        elif input_value > self.points[2] and input_value < self.points[3]:
            return (self.points[3] - input_value) / (self.points[3] - self.points[2])
        
        return 0.0

class rule1: # to get firing strength
    
    def __init__(self, i, o):
        self.inputs = i
        self.outputs = o
    
    def getFir(self, list_Values):
        list_memValues = []
        for i in range(len(self.inputs)):
            list_memValues.append(list_Values[i][self.inputs[i]])
        return min(list_memValues)

#    RULEBASE        [RFS  RBS]       [ TURNING, SPEED]
s1 = rule1(['close', 'close'], ['left', 'slow'])
s2 = rule1(['close', 'med'],   ['left', 'slow'])
s3 = rule1(['close', 'far'],   ['left', 'med'])
s4 = rule1(['med', 'close'],   ['right', 'med'])
s5 = rule1(['med', 'med'],     ['med', 'med'])
s6 = rule1(['med', 'far'],     ['left', 'med'])
s7 = rule1(['far', 'close'],   ['right', 'slow'])
s8 = rule1(['far', 'med'],     ['right', 'slow'])
s9 = rule1(['far', 'far'],     ['right', 'slow'])


RFS = { 0.1:'close', 0.3:'med', 0.5:'far'}
RBS = { 0.1:'close', 0.3:'med', 0.5:'far'}
Speed = { 0.1:'slow', 0.3:'med', 0.5:'fast'}
Steering = { 0.1:'left', 0:'med', -0.1:'right'}

# Asssigning Values to different ranges
close_RB_Dist = TrapFunc1([0.1,0.1,0.3,0.5], 'close')
close_RF_Dist = TrapFunc1([0.1,0.1,0.3,0.5], 'close')

med_RB_Dist = TrapFunc1([0.3,0.5,0.5,0.8], 'med')
med_RF_Dist = TrapFunc1([0.3,0.5,0.5,0.8], 'med')

far_RB_Dist = TrapFunc1([0.8, 0.8, 1.0 ,1.0], 'far')
far_RF_Dist = TrapFunc1([0.8, 0.8, 1.0 ,1.0], 'far')


turn_out1 = [1, 1, 1, 1, 0, 1, 1, 1, 0] 
vel_out1 = [0, 0, 0, 0, 0.5, 0.5, 0, 0.5, 0.5]



class rulebase1 : # DEFUZZIFICATION
    
    def __init__(self, list_rules):
        self.rules = list_rules
        
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

r_b1 = rulebase1([])

def oaController(RF_dist ,RB_dist): # Controller for Obstacle avoidance

    global close_RBinput
    global close_RFinput
    global med_RBinput
    global med_RFinput
    global far_RBinput
    global far_RFinput
    global r_b
    global turn_out
    global vel_out
    
    velocity , turning = 0.0 , 0.0
    if RF_dist != 0 and RB_dist != 0 :
        RBS = {}
        RBS['close'] = close_RB_Dist.getMemberValue(RB_dist)
        RBS['med'] = med_RB_Dist.getMemberValue(RB_dist)
        RBS['far'] = far_RB_Dist.getMemberValue(RB_dist)
        RFS = {}
        RFS['close'] = close_RF_Dist.getMemberValue(RF_dist)
        RFS['med'] = med_RF_Dist.getMemberValue(RF_dist)
        RFS['far'] = far_RF_Dist.getMemberValue(RF_dist)   
        list_Values = [RFS, RBS]

        rulebase1 = [s1.getFir(list_Values),s2.getFir(list_Values),s3.getFir(list_Values),s4.getFir(list_Values),
                        s5.getFir(list_Values),s6.getFir(list_Values),s7.getFir(list_Values),s8.getFir(list_Values),s9.getFir(list_Values)]
        fs= []
        vel =[]
        turn =[]
        for i in range(len(rulebase1)):
            if rulebase1[i] > 0: 
                fs.append(rulebase1[i])
                vel.append(vel_out1[i])
                turn.append(turn_out1[i])
        velocity,turning = r_b1.Defuz(fs,turn ,vel)
    return (velocity ,turning)



# RIGHT EDGE CODE

class TrapFunc: # to get membership value
    
    def __init__(self, Points, label):
        
        self.points = Points
        self.linguistic_label = label 
        
    def getMemberValue(self, input_value):
        if input_value < self.points[0] or input_value > self.points[3]:
            return 0.0
        elif input_value >= self.points[0] and input_value < self.points[1]:
            return(input_value - self.points[0])/(self.points[1] - self.points[0])
        elif input_value >= self.points[1] and input_value <= self.points[2]:
            return 1.0
        elif input_value > self.points[2] and input_value < self.points[3]:
            return (self.points[3] - input_value) / (self.points[3] - self.points[2])
        
        return 0.0

class rule: # to get firing strength
    
    def __init__(self, i, o):
        self.inputs = i
        self.outputs = o
    
    def getFir(self, list_Values):
        list_memValues = []
        for i in range(len(self.inputs)):
            list_memValues.append(list_Values[i][self.inputs[i]])
        return min(list_memValues)


#  RULEBASE  [RFS  RBS]       [ TURNING, SPEED]
r1 = rule(['close', 'close'], ['left', 'slow'])
r2 = rule(['close', 'med'],   ['left', 'slow'])
r3 = rule(['close', 'far'],   ['left', 'med'])
r4 = rule(['med', 'close'],   ['right', 'med'])
r5 = rule(['med', 'med'],     ['med', 'med'])
r6 = rule(['med', 'far'],     ['left', 'med'])
r7 = rule(['far', 'close'],   ['right', 'slow'])
r8 = rule(['far', 'med'],     ['right', 'slow'])
r9 = rule(['far', 'far'],     ['right', 'slow'])


RFS = { 0.0:'close', 0.5:'med', 0.75:'far'}
RBS = { 0.1:'close', 0.3:'med', 0.5:'far'}
Speed = { 0.1:'slow', 0.3:'med', 0.5:'fast'}
Steering = { 0.1:'left', 0:'med', -0.1:'right'}

# Asssigning Values to different ranges
close_RB_Dist = TrapFunc([0.0,0.0,0.25,0.4], 'close') 
close_RF_Dist = TrapFunc([0.0,0.0,0.25,0.4], 'close')

med_RB_Dist = TrapFunc([0.4,0.5,0.5,0.6], 'med')
med_RF_Dist = TrapFunc([0.4,0.5,0.5,0.6], 'med')

far_RB_Dist = TrapFunc([0.6, 0.75, 1.0 ,1.0], 'far')
far_RF_Dist = TrapFunc([0.6, 0.75, 1.0 ,1.0], 'far')

slow_Dist = TrapFunc([0.0,0.1], 'slow')
med_Dist = TrapFunc([0.1,0.3], 'med')
fast_Dist = TrapFunc([0.3,0.5], 'fast')

turn_out = [0.5, 0.5, 0.5, -0.5, 0, 0.5, -0.5, -0.5, 0.0] 
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

class rulebase : # DEFUZZIFICATION
    
    def __init__(self, list_rules):
        self.rules = list_rules
        
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
    

def wfController(RF_dist,RB_dist): # CONTROLLER FOR RIGHT EDGE
    global close_RBinput
    global close_RFinput
    global med_RBinput
    global med_RFinput
    global far_RBinput
    global far_RFinput
    global r_b
    global turn_out
    global vel_out

    velocity , turning = 0.0 , 0.0
    
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
            if rulebase[i] > 0: 
                fs.append(rulebase[i])
                vel.append(vel_out[i])
                turn.append(turn_out[i])
        velocity, turning  = r_b.Defuz(fs,turn ,vel)

    return ( velocity ,turning )



LF_dist = 0.0
RUF_dist = 0.0 
RF_dist  = 0.0
RB_dist  = 0.0


def callback(msg):
    global LF_dist
    global RUF_dist 
    global RF_dist 
    global RB_dist
    
    LF_dist  =  min([msg.ranges[30], msg.ranges[30]],msg.ranges[30])     # front left sensor range
    RUF_dist =  min([msg.ranges[690], msg.ranges[690]], msg.ranges[690])  # front right sensor range
    RB_dist  =  msg.ranges[540]                                           # right sensor range
    RF_dist  =  min([msg.ranges[600], msg.ranges[580]])                  # front right sensor range


  # maximum values assigning to 1 to deal with infinity

    if RB_dist > 1.0 :
        RB_dist = 1.0
    if RF_dist > 1.0 :
        RF_dist = 1.0
    if LF_dist > 1.0 :
        LF_dist = 1.0 
    if RUF_dist > 1.0 :
        RUF_dist = 1.0 
    
    #rospy.loginfo("RBS %f RFS %f",RB_dist, RF_dist)


sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)


def forwards(speed, turn):
	global pub
	vel_x = Twist() # initiates twist message
	vel_x.linear.x = speed  # out_sets linear speed of robot
	vel_x.angular.z = turn  # out_sets angle to turn 
	pub.publish(vel_x) #publishes velocity
    


def controller(): 
    global LF_dist
    global RUF_dist 
    global RF_dist 
    global RB_dist
    speed , turning = 0.0 , 0.0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

	if LF_dist < 0.8 or RUF_dist < 0.8 :   # front sensor activates only when there is something in front of the rosbot in a specific range
            speed , turning = oaController(LF_dist,RUF_dist)                    
        else :
            speed , turning = wfController(RF_dist,RB_dist)

        forwards(speed,turning)
        rate.sleep()

def stop() : # FUNCTION TO STOP ROSBOT 
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

    

#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TrapFunc:
    
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


class rule:
    
    def __init__(self, i, o):
        self.inputs = i
        self.outputs = o
    
    def getFir(self, list_Values):
        list_memValues = []
        for i in range(len(self.inputs)):
            list_memValues.append(list_Values[i][self.inputs[i]])
        return min(list_memValues)


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


#          [FS       LFS      RFS]    [ TURNING, SPEED]
'''r1 = rule(['close', 'close','close'], ['right', 'slow'])
r2 = rule(['close','close' , 'med'],   ['right', 'slow'])
r3 = rule(['close','close' , 'far'],   ['right', 'med'])
r4 = rule(['far' , 'close' , 'far'],   ['right', 'slow'])
r5 = rule(['close', 'med' , 'far'],     ['right', 'med'])
r6 = rule(['close', 'far' ,'med'],     ['left', 'slow'])
r7 = rule(['close' ,'far' ,'far'],   ['right', 'slow'])
r8 = rule(['close', 'med' ,'med'],     ['right', 'slow'])
r9 = rule(['med', 'close' , 'med'],     ['right', 'slow'])

r10 = rule(['close', 'med' , 'close'], ['left', 'slow'])
r11 = rule(['close','far','close'], ['left', 'slow'])
r12 = rule(['med', 'med' ,'med'],   ['med', 'med'])
r13 = rule(['med', 'med','close'],   ['left', 'med'])
r14 = rule(['med', 'med','far'],   ['right', 'med'])
r15 = rule(['med', 'far','far'],     ['left', 'slow'])
r16 = rule(['med', 'close' ,'close'],     ['med', 'slow'])
r17 = rule(['med', 'far', 'close'],   ['left', 'slow'])
r18 = rule(['med', 'close' ,'far'],     ['med', 'slow'])
r19 = rule(['far', 'far' ,'far'],     ['med', 'med'])

r20 = rule(['med', 'close' ,'med'], ['right', 'slow'])
r21 = rule(['far', 'far', 'close'],   ['left', 'med'])
r22 = rule(['far', 'far' 'med'],   ['med', 'med'])
r23 = rule(['far', 'med' ,'med'],   ['left', 'med'])
r24 = rule(['far', 'close' 'close'],   ['med', 'med'])
r25 = rule(['far', 'med' ,'close'],     ['left', 'med'])
r26 = rule(['far', 'close' ,'med'],     ['right', 'slow'])
r27 = rule(['far', 'med' ,'far'],   ['right', 'slow'])'''



RB_dist = 0.0 
RF_dist = 0.0  

close_RB_Dist = TrapFunc([0.1,0.1,0.3,0.5], 'close')
close_RF_Dist = TrapFunc([0.1,0.1,0.3,0.5], 'close')

med_RB_Dist = TrapFunc([0.3,0.5,0.5,0.8], 'med')
med_RF_Dist = TrapFunc([0.3,0.5,0.5,0.8], 'med')

far_RB_Dist = TrapFunc([0.8, 0.8, 1.0 ,1.0], 'far')
far_RF_Dist = TrapFunc([0.8, 0.8, 1.0 ,1.0], 'far')

slow_Dist = TrapFunc([0.0,0.1], 'slow')
med_Dist = TrapFunc([0.1,0.3], 'med')
fast_Dist = TrapFunc([0.3,0.5], 'fast')

turn_out = [1, -1, -1, 1, 0, -1, 1, 1, 0]
vel_out = [0, 0, 0, 0, 0.5, 0.5, 0, 0.5, 0.5]



class rulebase :
    
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

def callback(msg):
    global RB_dist 
    global RF_dist 
    RB_dist  =  min([msg.ranges[690], msg.ranges[0]], msg.ranges[650])  
    RF_dist  =  min([msg.ranges[30], msg.ranges[0]],msg.ranges[90])    
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
	vel_x = Twist() #  initiates twist message
	vel_x.linear.x = speed  # out_sets linear speed of robot
	vel_x.angular.z = turn  # out_sets angle to turn 
	pub.publish(vel_x) #publishes velocity
    


def controller(): # sending linear and angular velocity after processing

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
                if rulebase[i] > 0: 
                    fs.append(rulebase[i])
                    vel.append(vel_out[i])
                    turn.append(turn_out[i])

            velocity , turning  = r_b.Defuz(fs,turn ,vel)
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


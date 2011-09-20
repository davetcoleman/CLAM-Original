import joint as j
import servo
import time
import sys
import traceback
import subprocess

class Arm(object):
    def __init__(self):
        ''' dev_name - name of serial device of the servo controller (default '/dev/ttyUSB0')
        '''
        
        self.findUSB()
 
        # save original position so that we can go back to sleep later on
        # this is assuming that the arm woke up at a 'docked' position, or off position
        self.original_position = self.read_position()


    def findUSB(self):

        # loop through possible ttyUSBX numbers for X
        for usbNum in range(0,2):

            dev_name = '/dev/ttyUSB'+str(usbNum)

            # establish connection via usb
            print 'Attempting to establish connection with arm via '+dev_name+' serial:'

            try:
                self.makeJoints(dev_name)
                print 'Connection established to ',dev_name,'! \n'
                return True

            except RuntimeError:
                print '\nUnable to connect to arm'

            except Exception as e:
                print 'Unable to connect to arm'
                traceback.print_exc(e)
                sys.exit()
                
        # last resort, output debug info:
        print '\nAvailable USB Devices on your system:'
        command = 'cd /dev && find . -name ttyUSB* -print'
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
        for line in process.stdout:
            print line
        process.wait()

        print '\n\nExiting.'
        sys.exit()

    def makeJoints(self, dev_name):

        self.joints = [self.makeJoint(dev_name, 0, offset=150, min_angle=0, max_angle=300, max_rpm=30),
        self.makeJoint(dev_name, 1, offset=127.1, min_angle=61, max_angle=233.6, max_rpm=30, type='EX-106'),
        self.makeJoint(dev_name, 2, offset=150, min_angle=0, max_angle=300, max_rpm=30),
        self.makeJoint(dev_name, 3, offset=150, min_angle=54.25, max_angle=262, max_rpm=30),
        self.makeJoint(dev_name, 4, offset=150, min_angle=0, max_angle=300, max_rpm=30),
        self.makeJoint(dev_name, 5, offset=150, min_angle=35.2, max_angle=262, max_rpm=30),
        self.makeJoint(dev_name, 6, offset=150, min_angle=0, max_angle=300, max_rpm=30),
        self.makeJoint(dev_name, 7, offset=60, min_angle=45, max_angle=120, max_rpm=30)]
        
        self.all = servo.servo(dev_name,254)
        if set(self.joints) == set([None]):
            raise RuntimeError('Arm: no servos found')
        
        self.claw = len(self.joints)-1
        self.ts=.03
        self.torqued=False
#        self.set_torque_all(True)
#        time.sleep(.1)
        self.set_rpm_all(6)
        self.set_torque_all(False)


    def makeJoint(self, dev_name, id, offset, min_angle, max_angle, max_rpm,type=None):
        try:
            joint = j.single_joint(dev_name, id, offset=offset, min_angle=min_angle,\
                                       max_angle=max_angle, max_rpm=max_rpm, type=type)

            return joint
        except RuntimeError as e:
            pass
            print 'Warning: unable to initialize servo',id,'on',dev_name
            # traceback.print_exc(e)
        return None

    
    def zero(self, rpm=None):
        ''' moves all joints to their zero angle
        '''
        self.position([0]*len(self.joints))

    def write_goal(self, joint, angle, rpm=None, blocking=None):
        ''' move joint to angle
        '''

        if joint > len(self.joints) or joint < 0:
            print "ERROR: Invalid joint number. move_joint_angle command ignored.   joint:", joint
        else:
            if rpm == None:
                rpm = self.rpm[joint]
            else:
                self.rpm[joint] = rpm
            if self.joints[joint]:
                self.joints[joint].goal_angle(angle, rpm, blocking)

    def read_pos(self, joint):
        '''returns joint's present angle
        '''
        if joint > len(self.joints) or joint < 0:
            print "ERROR: Invalid joint number. read_joint_angle command ignored.   joint:", joint
        else:
            if self.joints[joint]:
                return self.joints[joint].read_position()
            else:
                return None
    
    def trajectory(self, traj, blocking=j.velocity, goToEnd=False, timestep=None):
        if timestep == None:
            timestep = self.ts
        if not goToEnd:
            if blocking == j.velocity:
                activeJoints = list(i for i in range(len(self.joints)) if traj[0][i] != None and self.joints[i] != None)
                speeds=[]
                positions=[]
                for i in range(len(traj)-1):
                    speedList=[0]*len(self.joints)
                    posList=[0]*(len(self.joints)-1)+[None]
                    for k in activeJoints:
                        joint = self.joints[k]
                        diff = traj[i+1][k]-traj[i][k]
                        speedList[k] = abs((diff/timestep)/6) # 1 deg/sec == 1/6 rpm
                        if speedList[k] > 25:
                            print '**** excessive speed:',speedList[k]
                            print '     joint:', k
                            print '     trajectory entry:',i
                            print '     diff:',diff
                            print '     timestep:',timestep
                            return
                        if diff < 0:
                            posList[k]=traj[i+1][k]-1
                        else:
                            posList[k]=traj[i+1][k]+1
#                        if diff < 0:
#                            posList[k]=self.joints[k].min_angle+10
#                        else:
#                            posList[k]=self.joints[k].max_angle-10
                    speeds.append(speedList)
                    positions.append(posList)
                
                start=time.time()
                for i in range(len(speeds)):
                    print 'start iteration', i
                    self.set_rpm_list(speeds[i])
                    self.position(positions[i])
                    end=time.time()
#                    print end-start
                    time.sleep(timestep-end+start)
                    start=time.time()
                    
                        
                        
            else:
                for pos in traj:
                    self.position(pos, blocking=blocking)
        self.set_rpm_all(3)
        self.position(traj[-1], blocking=blocking)
    
    def position(self, joint_list, rpm=None, blocking=None):
        ''' sets each joint to its corresponding value in joint_list
        '''
        if not len(joint_list) == len(self.joints):
            print "ERROR: joint list invalid. ignoring position command.   joint_list:", joint_list
        else:
            if not self.torqued:
                self.set_torque_all(True) #workaround for servo bug
                self.torqued = True
#            print 'moving to',joint_list
            activeJoints = list(i for i in range(len(joint_list)) if joint_list[i] != None)
            for i in activeJoints:
#                print '**** waiting on',i 
                self.write_goal(i, joint_list[i], rpm,blocking)
#            print '\rdone         '
    
        
    def read_position(self):
        '''returns list position of arm
        '''
        pos = []
        for jn in self.joints:
            if jn:
                try:
                    pos.append(jn.read_position())
#                except KeyboardInterrupt as k:
#                    raise k
                except RuntimeError:
                    pos.append(None)
            else:
                pos.append(None)
        return pos

    def set_rpm(self, joint, rpm):
        ''' sets joint moving speed to rpm
        '''
        if joint > len(self.joints) or joint < 0:
            print "ERROR: Invalid joint number. set_rpm command ignored.    joint:", joint
        else:
            if self.joints[joint]:
                self.joints[joint].speed(rpm)
            self.rpm[joint]=rpm

    def set_rpm_list(self, rpm_list):
        ''' sets each joint rpm to its corresponding value in rpm_list
        '''
        if not len(rpm_list) == len(self.joints):
            print "ERROR: rpm list invalid. ignoring set_rpm_list command.   rpm_list:", rpm_list
        else:
            self.rpm=rpm_list
            for i in range(len(self.joints)):
                self.set_rpm(i, rpm_list[i])

    def set_rpm_all(self, rpm):
        '''sets all joints moving speed to rpm
        '''
        self.set_rpm_list([rpm]*len(self.joints))

    def set_torque(self, joint, enable):
        '''enables torque if set to True
        disables torques if set to false
        '''
        if joint > len(self.joints) or joint < 0:
            print "ERROR: Invalid joint number. set_torque command ignored."
        else:
            if self.joints[joint]:
                self.joints[joint].torque(enable)
            self.torqued &= enable

    def set_torque_all(self, enable):
        '''enables torque if set to True
        disables torques if set to false
        '''
        for i in range(len(self.joints)):
            self.set_torque(i, enable)

    def claw_open(self):
        '''moves to angle 0 then disables torque
        '''
        if self.joints[self.claw]:
            self.write_goal(self.claw,-10)
            time.sleep(.5)
            while self.joints[self.claw].is_moving():
                time.sleep(.01)
            self.joints[self.claw].servo_write("torque enable", False)

    def claw_soft_close(self):
        '''attemps to completely close claw, then disables torque, then re-enable torque
        '''
        if self.joints[self.claw]:
            self.write_goal(self.claw,60)
            time.sleep(.01)
            while self.joints[self.claw].is_moving():
                time.sleep(.01)
            
    #        self.claw.servo_write("torque enable", False)
    #        self.claw.servo_write("torque enable", True)
            
            self.write_goal(self.claw,self.read_pos(self.claw))

    # Simply outputs to screen all info about the arm's joints
    def print_joint_stats(self):

        print 'Outputting Joint Stats:\n\n'
        for i in range(len( self.joints )):
            joint = self.joints[i]
            print 'Joint Index', i,': -------------------------------'
            print 'Servo ID: ',joint.get_servo_id()
            print 'Offset: ',joint.get_offset()
            print 'Min Angle: ',joint.get_min_angle()
            print 'Max Angle: ',joint.get_max_angle()
            print 'Max RPM: ',joint.get_max_rpm()
            print 'Flipped: ',joint.get_flipped()
            print 'Current Position: ',joint.read_position()
            print 'Is Moving: ',joint.is_moving()
            #joint.servo_print_read_all()
            print '\n'

    # Run this function at the very end. Sends the arm to its original position and turns off torque
    def shutdown(self):

        # send arm back to wake up position
        self.position(self.original_position)

        print '/n'
        print '---------------------------------------------------------'
        print '---------------------------------------------------------'
        print 'ENSURE ROBOT IS IN HOME POSITION OR SECURED'
        print 'Shutdown in 6 seconds...'
        print '---------------------------------------------------------'
        print '---------------------------------------------------------'
        
        for second in range(1,7):
            time.sleep(1)
            print second

        # shutdown arm
        self.set_torque_all(False)
        self.set_torque_all(False)

        print 'Arm shutdown.\n'



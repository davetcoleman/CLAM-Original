import pdArm
import time
import sys
import subprocess
import traceback

arm = None; # this will later be loaded with the robot arm class
original_position = None;

def main():
    # import global variables
    global arm, original_position

    # this is called when program starts
    print '-----------------------------------------------------------------'
    print 'CLAM - Correl Lab Arm Manipulator'
    print 'Diagnostic Tool\n'

    # check if connection to robot estashlished
    print 'Trying to establish connection with arm via USB serial:'
    try:
        arm = pdArm.PdArm()
    except RuntimeError:
        print '\nUnable to connect to arm.'
        print '\nAvailable USB Devices on your system:'
        command = 'cd /dev && find . -name ttyUSB* -print'
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)
        for line in process.stdout:
            print line
        process.wait()

        print '\n\nExiting.'
        sys.exit()

    except Exception as e:
        print 'Unable to connect to arm'
        traceback.print_exc(e)
        sys.exit()

    print 'Connection established to ',arm.joints[0].get_dev_name(),'! \n'

    print_joint_stats()

    # save original position so that we can go back to sleep later on
    # this is assuming that the arm woke up at a 'docked' position, or off position
    original_position = arm.read_position()

    time.sleep(2)
 
    raw_input('Press any key to go to zero position (home)')
    print '\nSending robot to zeros position\n'
    arm.zero()
    
    waitMovingFinish() # pause until last command to arm completes

#    print '\nClose Claw'
    # now make hand open and close for funz
#    arm.claw_soft_close()
#    waitMovingFinish() # pause until last command to arm completes
    
#    print '\nOpen Claw'
#    arm.claw_open()
#    waitMovingFinish() # pause until last command to arm completes

    print 'Looping joint tests'
    joint_tests()

    
    time.sleep(3)

    raw_input('Press any key to send arm back to original position and shutdown')

    shutdown();


# Simply outputs to screen all info about the arm's joints
def print_joint_stats():
    # import global variables
    global arm

    print 'Outputting Joint Stats:\n\n'
    for i in range(len( arm.joints )):
        joint = arm.joints[i]
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

# Loop through each joint and run to max angle
def joint_tests():
    global arm

    print '\nMoving all extreme joints'
    
    i = len( arm.joints ) - 1

    while(i > 1): # loop in reverse order, starting with 7
        print 'controlling joint',i

        joint = arm.joints[i]
        
        max_angle_degrees = joint.get_max_angle()
        min_angle_degrees = joint.get_min_angle()
        print 'max angle degrees:',max_angle_degrees
        print 'min angle degrees:',min_angle_degrees

        max_angle_offset = ( max_angle_degrees - joint.get_offset() ) / joint.get_flipped()
        min_angle_offset = ( min_angle_degrees - joint.get_offset() ) / joint.get_flipped()


        print 'max angle offset:',max_angle_offset
        print 'min angle offset:',min_angle_offset

        raw_input('Press any key to activate move')
    
        arm.write_goal(i, max_angle_offset)
        waitMovingFinish()

        raw_input('Press any key to activate move')

        arm.write_goal(i, min_angle_offset)
        waitMovingFinish()
        # print 'joint position:',joint.read_position()

        raw_input('Press any key to go to zero position (home)')

        # reset to home position
        time.sleep(1)
        arm.zero()
        waitMovingFinish()
 
        print '\n\n\n'
        

        i -= 1

        
# Loop until the current command to arm finishes
def waitMovingFinish():

    # sleep for 10 miliseconds to give the joints time to take last command
    time.sleep(5)

    """
    while True:
        foundMoving = False # no joints are moving as far as we know, yet
        
        for i in range(len( arm.joints )):
            joint = arm.joints[i]
        
            if(joint.is_moving()):
                foundMoving = True
                continue

        if not foundMoving:
            break

        time.sleep(.1) # wait before re-checking all joints        
    """

# Make the arm dynamically pliable so that you can move it and it stays in place
def make_pliable():
    # this function doesn't work :-(
    stop=False
    
    while not stop:
        print 'looping'

        arm.set_torque_all(True);

        try:
            # get the current location and make it the goal position
            deg = arm.read_position()
            arm.position(deg)

            time.sleep(.01)
        except KeyboardInterrupt:
            print 'Ended by keyboard'
            # shutdown()
            stop = True
            # arm.set_torque_all(True);

# Run this function at the very end. Sends the arm to its original position and turns off torque
def shutdown():
    global original_position

    # send arm back to wake up position
    arm.position(original_position)

    waitMovingFinish(); # wait for arm to finish moving
    print '/n'
    print '---------------------------------------------------------'
    print '---------------------------------------------------------'
    print 'ENSURE ROBOT IS IN HOME POSITION OR SECURED'
    print 'Shutdown in 5 seconds...'
    print '---------------------------------------------------------'
    print '---------------------------------------------------------'
    time.sleep(5)

    # shutdown arm
    arm.set_torque_all(False)
    arm.set_torque_all(False)

    print 'Diagnosis Complete.\n'
    sys.exit()

    
# Where the program starts
if __name__ == "__main__":
    try:
        main()
    # Now attempt to prevent the robot from falling limp
    except KeyboardInterrupt:
        shutdown()
    except Exception as e:
        traceback.print_exc(e)        
        shutdown()


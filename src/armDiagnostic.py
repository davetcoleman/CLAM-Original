import pdArm
import time

arm = None; # this will later be loaded with the robot arm class

def main():
    # import global variables
    global arm

    # this is called when program starts
    print 'CLAM - Correl Lab Arm Manipulator\n'
    print 'Diagnostic Tool\n'

    # check if connection to robot estashlished
    print 'Trying to establish connection with arm via USB serial:'
    try:
        arm = pdArm.PdArm()
    except RuntimeError:
        print 'Unable to connect to arm'
    except Exception as e:
        print 'Unable to connect to arm'
        traceback.print_exc(e)

    print 'Connection established to ',arm.joints[0].get_dev_name(),'! \n'

    print_joint_stats()

    # save original position so that we can go back to sleep later on
    # this is assuming that the arm woke up at a 'docked' position, or off position
    original_position = arm.read_position()

    time.sleep(2)

    print '\nSending robot to zeros position\n'
    arm.zero()
    time.sleep(5)

    # now make hand open and close for funz
    # arm.claw_open()
    # time.sleep(3)
    # arm.claw_soft_close()

    # re-print the joint stats
    print_joint_stats()
    
    raw_input('Press any key to send arm back to original position and shutdown')

    # send arm back to wake up position
    arm.position(original_position)
    
    print 'Shutdown in 5 seconds...'
    time.sleep(5)

    # shutdown arm
    arm.set_torque_all(False)

    print 'Diagnosis Complete.'


def print_joint_stats():
    # import global variables
    global arm

    # Simply outputs to screen all info about the arm's joints

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

    
if __name__ == "__main__":
    main()

import arm as a
import time
import sys
import pickle
import traceback

arm = None; # this will later be loaded with the robot arm class

def main():
    # import global variables
    global arm

    # this is called when program starts
    print '-----------------------------------------------------------------'
    print 'CLAM - Correl Lab Arm Manipulator'
    print 'Recorder\n'

    arm = a.Arm()

    f = open('/home/robot/ros/clam/src/output/dump','w')

    stop=False
    
    arm.set_torque_all(False);
    
    coords = []

    while not stop:
        print 'Recording coordinates...'

        try:
            # get the current location and make it the goal position
            pos = arm.read_position()
            # pickle.dump(pos, f)
            coords.append(pos)

            time.sleep(.5)
        except KeyboardInterrupt:
            print 'Ended by keyboard'
            # shutdown()
            stop = True

    pickle.dump(coords,f)
    f.close()

    print 'Recording successful'

    sys.exit()


def shutdown():
    
    if arm is not None:
        arm.shutdown()
    print 'End of program'

    
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


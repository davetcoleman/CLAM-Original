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
    print 'Player\n'

    arm = a.Arm()

    # load the file

    f = open('/home/robot/ros/clam/src/output/dump','r')
    read_coords = pickle.load(f)

    for coor in read_coords:
        print 'Playing...'
        arm.position(coor)
        #print arm.read_position()
        time.sleep(5)

    shutdown();

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


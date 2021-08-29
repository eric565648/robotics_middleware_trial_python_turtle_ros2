import termios, fcntl, sys, os
#TODO: import RR/ROS libraries and ROS message/service type 


#keyboard reading settings
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

#TODO: Initialize ROS/RR node
#ROS: create publisher to publish Twist message to corresponding topic name

#RR: connect to service with url

print("Running")
print("Press Arrow Key to Control Turtle")
print("Press q to quit")
try:
    #TODO: hold the script running with ROS/RR way
    while True:
        try:
            #read input and print "command"
            c = sys.stdin.read()
            #TODO: ROS create message type variable, publish command
            #TODO: RR call drive function
            if "\x1b[A" in c:
                print("drive forward")          ####Drive forward
            if "\x1b[B" in c:
                print("drive backward")         ####Drive backward               
            if "\x1b[C" in c:
                print("drive right")            ####Drive right
            if "\x1b[D" in c:
                print("drive left")             ####Drive left
            if "q" in c:
                break

        except IOError: pass
        except TypeError: pass
#finish reading keyboard input
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
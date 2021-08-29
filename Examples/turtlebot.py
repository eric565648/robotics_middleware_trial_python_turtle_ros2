import turtle

screen = turtle.Screen()
t1 = turtle.Turtle()
t1.shape("turtle")
screen.bgcolor("lightblue")



def drive(turtlebot,move_speed,turn_speed):  #Drive function, update new position, this is the one referred in definition

	turtlebot.forward(move_speed)
	turtlebot.left(turn_speed)

def setpose(turtlebot,x,y,angle):            #set a new pose for turtlebot

	turtlebot.setpos(x,y)
	turtlebot.seth(angle)
		



for i in range(50):
	drive(t1,10,0)
t1.pencolor("red")
for i in range(50):
	drive(t1,10,i)

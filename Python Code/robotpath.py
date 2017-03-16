from robocontrol import *

#Function for moving the robot
def movetherobot(currentdestination,previousdestination):
    global currentorientation
    global count
    if currentorientation == currentdestination and currentdestination == previousdestination:
        forward_real()
        currentorientation = currentdestination
        count = count + 1
        print"dint change orientation"
        print"going forward from",previousdestination,"to",currentdestination
        print"0"
        print" "
    else:
        if currentorientation == "north":
            print"changed orientation from",currentorientation,
            if currentdestination == "west":
                left_real()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"1"
                print" "
            if currentdestination == "east":
                right_real()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"2"
                print" "
            if currentdestination == "south":
                about_turn()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"3"
                print" "
                
        if currentorientation == "south":
            print"changed orientation from",currentorientation,
            if currentdestination == "west":
                right_real()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print "to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"4"
                print" "
            if currentdestination == "east":
                left_real()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"5"
                print" "
            if currentdestination == "north":
                about_turn()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"6"
                print" "
            
        if currentorientation == "east":
            print"changed orientation from",currentorientation,
            if currentdestination == "north":
                left_real()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"7"
                print" "
            if currentdestination == "south":
                right_real()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"8"
                print" "
            if currentdestination == "west":
                about_turn()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"9"
                print" "
             
        if currentorientation == "west":
            print"changed orientation from",currentorientation,
            if currentdestination == "north":
                right_real()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"10"
                print" "
            if currentdestination == "south":
                left_real()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"11"
                print" "
            if currentdestination == "east":
                about_turn()
                forward_real()
                currentorientation = currentdestination
                count = count + 1
                print"to",currentorientation
                print"going forward from",previousdestination,"to",currentdestination
                print"12"
                print" "
            
    return

#function to update orientation of the robot
def updateorientation(currentdestination,previousdestination):
    global currentorientation
    global count
    count = count+1
    if currentorientation == currentdestination and currentdestination == previousdestination:
        currentorientation = currentdestination
        print"dint change orientation"
    else:
        if currentorientation == "north":
            print"changed orientation from",currentorientation,
            if currentdestination == "west":
                left_real()
                currentorientation = currentdestination
                print"to",currentorientation
            if currentdestination == "east":
                right_real()
                currentorientation = currentdestination
                print"to",currentorientation
            if currentdestination == "south":
                about_turn()
                currentorientation = currentdestination
                print"to",currentorientation
                
        if currentorientation == "south":
            print"changed orientation from",currentorientation,
            if currentdestination == "west":
                right_real()
                currentorientation = currentdestination
                print "to",currentorientation
            if currentdestination == "east":
                left_real()
                currentorientation = currentdestination
                print"to",currentorientation
            if currentdestination == "north":
                about_turn()
                currentorientation = currentdestination
                print"to",currentorientation
            
        if currentorientation == "east":
            print"changed orientation from",currentorientation,
            if currentdestination == "north":
                left_real()
                currentorientation = currentdestination
                print"to",currentorientation
            if currentdestination == "south":
                right_real()
                currentorientation = currentdestination
                print"to",currentorientation
            if currentdestination == "west":
                about_turn()
                currentorientation = currentdestination
                print"to",currentorientation
             
        if currentorientation == "west":
            print"changed orientation from",currentorientation,
            if currentdestination == "north":
                right_real()
                currentorientation = currentdestination
                print"to",currentorientation
            if currentdestination == "south":
                left_real()
                currentorientation = currentdestination
                print"to",currentorientation
            if currentdestination == "east":
                about_turn()
                currentorientation = currentdestination
                print"to",currentorientation  
    return

#function for going to location and  dropping the object
def performmovement_drop(path):
    global move
    global count
    move = []
    count = 0
    for index in range(len(path)-1):
        if path[index+1][0] == path[index][0]+1 : move.append("east")
        if path[index+1][0] == path[index][0]-1 : move.append("west")
        if path[index+1][1] == path[index][1]+1 : move.append("south")
        if path[index+1][1] == path[index][1]-1 : move.append("north")

    for index in range(len(move)):
        print(move[index])
    print(" ")
    print "Total Moves: ",len(move)
    print"Starting Orientation:",currentorientation
    print" "

    for index in range(len(move)):
        if index == 0:
            if index == len(move)-1:
                previousdestintaion2 = move[index-1]
                currentdestination2 = move[index]
                updateorientation(currentdestination2,previousdestintaion2) #only updates orientation
                drop()
                print "Object placed"
            else:
                previousdestination = move[0]
                currentdestination = move[0]
                movetherobot(currentdestination,previousdestination) #updates orientation as well as moves robot
        elif index == len(move)-1:
            previousdestintaion2 = move[index-1]
            currentdestination2 = move[index]
            updateorientation(currentdestination2,previousdestintaion2) #only updates orientation
            drop()
            print "Object placed"
        else:
            previousdestination = move[index-1]
            currentdestination = move[index]
            movetherobot(currentdestination,previousdestination) #updates orientation as well as moves robot
            
        

    print" "
    print"Total moves performed: ",count
    return

#function for perfroming path moment and picking up the object
def performmovement_pick(path):
    global move
    global count
    move = []
    count = 0
    
    for index in range(len(path)-1):
        if path[index+1][0] == path[index][0]+1 : move.append("east")
        if path[index+1][0] == path[index][0]-1 : move.append("west")
        if path[index+1][1] == path[index][1]+1 : move.append("south")
        if path[index+1][1] == path[index][1]-1 : move.append("north")

    for index in range(len(move)):
        print(move[index])
    print(" ")
    print "Total Moves: ",len(move)
    print"Starting Orientation:",currentorientation
    print" "

    for index in range(len(move)):
        if index == 0:
            if index == len(move)-1:
                previousdestintaion2 = move[index-1]
                currentdestination2 = move[index]
                updateorientation(currentdestination2,previousdestintaion2) #only updates orientation
                pickup()
                forward_real()
                print "Object picked and went forward"
            else:
                previousdestination = move[0]
                currentdestination = move[0]
                movetherobot(currentdestination,previousdestination) #updates orientation as well as moves robot
        elif index == len(move)-1:
            previousdestintaion2 = move[index-1]
            currentdestination2 = move[index]
            updateorientation(currentdestination2,previousdestintaion2) #only updates orientation
            pickup()
            forward_real()
            print "Object picked and went forward"
        else:
            previousdestination = move[index-1]
            currentdestination = move[index]
            movetherobot(currentdestination,previousdestination) #updates orientation as well as moves robot
            
        

    print" "
    print"Total moves performed: ",count
    return

currentorientation = "west"

move = []
count = 0



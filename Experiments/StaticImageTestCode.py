import cv2
import numpy as np
import copy

matched = []   # Stores matched pairs.
temp={}        # Temporary dictionary to store elements.
walls =[]      # Stores walls from the image.

# Functions and Classes for performing A-Star Algorithm.
'''
* Function Name: reconstruct_path
* Input: came_from, start, goal
* Output: path
* Logic: Uses reverse technique of a_star_search, starts from goal and computes
  till start is found.
* Example Call: path_tup = reconstruct_path(came_from,start=(1,9), goal =(8,5))
'''
def reconstruct_path(came_from, start, goal):
    current = goal
    current2 = current
    path = [current2]
    while current != start:
        current = came_from[current]
        current2 = correctTuple(current)
        path.append(current2)
    path.reverse()
    return path

# Class to contruct a square grid.
class SquareGrid:
    '''
    * Function Name: __init__
    * Input: width,heigth
    * Output: stores width and height
    * Logic: Assigns received values to self.
    * Example Call: SquareGrid.__init__(self, width, height)
    '''
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []

    '''
     * Function Name: in_bounds
     * Input: id
     * Output: Boolean Output
     * Logic: 0 <= x < self.width and 0 <= y < self.height
     * Example Call: results = filter(self.in_bounds, results)
     '''
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    '''
     * Function Name: passable
     * Input: id
     * Output: Boolean Output
     * Logic: return id not in self.walls
     * Example Call: results = filter(self.passable, results)
     '''
    def passable(self, id):
        return id not in self.walls

    '''
     * Function Name: neighbors
     * Input: id
     * Output: results
     * Logic: Enables the movement by +1 or -1 in (x,y) co-ordinate system.
     * Example Call: graph.neighbors(current)
     '''
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse()
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

# Class to assign weights in the grid.
class GridWithWeights(SquareGrid):
    '''
     * Function Name: __init__
     * Input: width,height
     * Output: Stores weights in self and  width,height in SquareGrid
     * Logic: Assigns recieved heights and widths as required.
     * Example Call: GridWithWeights.__init__(self, width, height)
     '''
    def __init__(self, width, height):
        SquareGrid.__init__(self, width, height)
        self.weights = {}

    '''
     * Function Name: cost
     * Input: from_node, to_node
     * Output: actual cost of moving in the grid
     * Logic: self.weights.get(to_node, 1) , based on weight cost is computed.
     * Example Call: graph.cost(current, next)
     '''
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)

import heapq

# Class to use queues.
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

'''
* Function Name: heuristic
* Input: a,b
* Output: sum
* Logic: sum(((x of a)-(x of b)),((y of a)-(y of b)))
* Example Call: heuristic(goal, next)
'''
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

'''
* Function Name: a_star_search
* Input: graph, start, goal
* Output: came_from, cost_so_far
* Logic: Computes path based on starting point and goal point and weights in the
  grid, takes path of least cost where cost is computed based on weights
  associated in the grid.
* Example Call: came_from, cost_so_far = a_star_search(diagram4,(0,9),(1,8))
'''
def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    # This loop computes the best path from start to goal based on weights and
    # obstacles.
    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far

# Function to store sorting key.
'''
* Function Name: getKey
* Input: item
* Output: item[0]
* Logic: Returns the first element of recieved array.
* Example Call: getKey(blueArray)
'''
def getKey(item):
    return item[0]

# This function identifies all objects in the image and return it as arrays of black objects and colours objects.
'''
* Function Name: get_objects
* Input: img
* Output: black_objects , coloured_objects arrays.
* Logic: Based on OpenCV techniques of masking, thresholding, countouring etc,
  details (objects) required from the image is extracted.
* Example Call: black_objects , coloured_objects = get_objects(image)
'''
def get_objects(img):
    # Arrays to store black objects (boxes/walls) and other colour objects.
    black_objects = []
    coloured_objects = []

    # Threshold values for respective colours.
    red_low = np.array([-10, 100, 100])
    red_high = np.array([10, 255, 255])
    green_low = np.array([50, 100, 100])
    green_high = np.array([70, 255, 255])
    blue_low = np.array([110, 100, 100])
    blue_high = np.array([130, 255, 255])
    y_low = np.array([29, 100, 100])
    y_high = np.array([57, 255, 255])
    backround_low = np.array([0, 0, 0])
    backround_high = np.array([179, 255, 15])

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask_backround = cv2.inRange(hsv,backround_low, backround_high)
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(mask_backround, cv2.MORPH_OPEN, kernel)
    mc_mask_background_int = cv2.bitwise_xor(mask_backround,opening)
    ret3 ,mc_mask_background = cv2.threshold(mc_mask_background_int,127,255,cv2.THRESH_BINARY_INV)

    # Eroded black for box detection.
    mask_black_eroded = cv2.erode(mask_backround,kernel,iterations = 1)

    mask_green = cv2.inRange(hsv, green_low, green_high)
    ret2, mc_mask_green = cv2.threshold(mask_green, 127, 255, 0)
    mask_blue = cv2.inRange(hsv, blue_low, blue_high)
    ret2, mc_mask_blue = cv2.threshold(mask_blue, 127, 255, 0)
    mask_yellow = cv2.inRange(hsv, y_low, y_high)
    ret2, mc_mask_yellow = cv2.threshold(mask_yellow, 127, 255, 0)
    mask_red = cv2.inRange(hsv, red_low, red_high)
    ret2, mc_mask_red = cv2.threshold(mask_red, 127, 255, 0)

    # For grid contours.
    contours_background, hierarchy = cv2.findContours(mc_mask_background.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # For black boxs (without grid) contours.
    contours_black_eroded ,hierarchy_black = cv2.findContours(mask_black_eroded.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # For coloured object contours.
    contours_green, hierarchy_green = cv2.findContours(mc_mask_green.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, hierarchy_blue = cv2.findContours(mc_mask_blue.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_yellow, hierarchy_yellow = cv2.findContours(mc_mask_yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, hierarchy_red = cv2.findContours(mc_mask_red.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    iter = 0

    # This loop computes position,colour,shapeType,area of an object by scanning each cell in the grid one by one.
    for cntr_background_pos, unk in enumerate(contours_background):
        cnt = contours_background[cntr_background_pos]
        no_of_boxes = len(contours_background)
        M = cv2.moments(cnt)
        centroid_x = int(M['m10'] / M['m00'])
        centroid_y = int(M['m01'] / M['m00'])
        for cntr_black_pos, unk2 in enumerate(contours_black_eroded):
            dist = cv2.pointPolygonTest(contours_black_eroded[cntr_black_pos], (centroid_x, centroid_y), True)
            if (dist >= 0):
                length = find_shape(contours_black_eroded[cntr_black_pos])
                area_black = cv2.contourArea(contours_black_eroded[cntr_black_pos])
                area_black = round(area_black, -2)
                tup_black = (num_to_coordinate(no_of_boxes - cntr_background_pos), "BLACK", length, area_black)
                black_objects.append(tup_black)

        for cntr_green_pos, unk2 in enumerate(contours_green):
            dist = cv2.pointPolygonTest(contours_green[cntr_green_pos], (centroid_x, centroid_y), True)
            if (dist >= 0):
                length = find_shape(contours_green[cntr_green_pos])
                area_green = cv2.contourArea(contours_green[cntr_green_pos])
                area_green = round(area_green, -2)
                tup_green = (num_to_coordinate(no_of_boxes - cntr_background_pos), "green", length, area_green)
                coloured_objects.append(tup_green)

        for cntr_blue_pos, unk3 in enumerate(contours_blue):
            dist = cv2.pointPolygonTest(contours_blue[cntr_blue_pos], (centroid_x, centroid_y), True)
            if (dist >= 0):
                length = find_shape(contours_blue[cntr_blue_pos])
                area_blue = cv2.contourArea(contours_blue[cntr_blue_pos])
                area_blue = round(area_blue, -2)
                tup_blue = (num_to_coordinate(no_of_boxes - cntr_background_pos), "blue", length, area_blue)
                coloured_objects.append(tup_blue)

        for cntr_red_pos, unk4 in enumerate(contours_red):
            dist = cv2.pointPolygonTest(contours_red[cntr_red_pos], (centroid_x, centroid_y), True)
            if (dist >= 0):
                length = find_shape(contours_red[cntr_red_pos])
                area_red = cv2.contourArea(contours_red[cntr_red_pos])
                area_red = round(area_red, -2)
                tup_red = (num_to_coordinate(no_of_boxes - cntr_background_pos), "red", length, area_red)
                coloured_objects.append(tup_red)

    black_objects = sorted(black_objects , key = getKey)
    coloured_objects = sorted(coloured_objects , key = getKey)

    return black_objects , coloured_objects

# Function to get the shape of the object.
'''
* Function Name: find_shape
* Input: contour_sub
* Output: shapeType
* Logic: Using approxPolyDP function from OpenCV library the vertices of the
  shape is computed to determine the shape.
* Example Call: find_shape(contours_red[cntr_red_pos])
'''
def find_shape(contour_sub):
    approx_contour = cv2.approxPolyDP(contour_sub, 0.02 * cv2.arcLength(contour_sub, True), True)
    vertices = len(approx_contour)
    if (vertices == 3):
        shapeType = "Triangle"
    if (vertices == 4):
        shapeType = "4-sided"
    if (vertices>5):
        shapeType = "Circle"
    return shapeType

# Function to convert grid index to co-ordinate scheme.
'''
* Function Name: num_to_coordinate
* Input: num
* Output: coordinate
* Logic: To get (x,y), we subtract 1 from recieved number and then find
  remainder and quotient for (x,y)
* Example Call: (x,y) = num_to_coordinate(8)
'''
def num_to_coordinate(num):
    x = int((num-1)%10)
    y = (int(num-1)/10)
    coordinate = (x,y)
    return coordinate

# Function to correct tuple co-ordinate
'''
* Function Name: correctTuple
* Input: tup
* Output: tempTuple
* Logic: converts tuple to list so that data can be manipulated, after adding 1
  to each co-ordinate the list is packed back into a tuple.
* Example Call: correctTuple((8,9))
'''
def correctTuple(tup):
        tempList = list(tup)
        tempList[0] = tempList[0]+1
        tempList[1] = tempList[1]+1
        tempTuple = tuple(tempList)
        return tempTuple

def main(image_filename):
        '''
This function is the main program which takes image of test_images as argument.

***DO NOT EDIT THE FUNCTION NAME. Leave it as main****
Function name: main()

******DO NOT EDIT name of these argument*******
Input argument: image_filename

        '''

        occupied_grids = []             # List to store coordinates of occupied grid -- DO NOT CHANGE VARIABLE NAME
        planned_path = {}               # Dictionary to store information regarding path planning       -- DO NOT CHANGE VARIABLE NAME




        ##### CODE STARTS HERE

        # cv2.imshow("board_filepath - press Esc to close",cv2.imread(board_filepath))                  - For check - remove
        # cv2.imshow("container_filepath - press Esc to close",cv2.imread(container_filepath))

        image = cv2.imread(image_filename,1)
        diagram4 = GridWithWeights(10, 10)
        black_objects , coloured_objects = get_objects(image)

        # The following loops count the number of occupied grids.
        for objects in black_objects:
                tempTuple = correctTuple(objects[0])
                occupied_grids.append(tempTuple)

        for objects in coloured_objects:
                tempTuple = correctTuple(objects[0])
                occupied_grids.append(tempTuple)

        temp_objects = copy.deepcopy(coloured_objects)
        unmatched = copy.deepcopy(coloured_objects)

        # This loop finds matching pairs and adds them to matched array of tuples.
        flag = 0
        for objects in temp_objects:
                for objects2 in temp_objects:
                    if (objects[1:] == objects2[1:] and objects[0] != objects2[0]):
                        matched.append([objects[0],objects2[0]])
                        matched.append([objects2[0],objects[0]])
                        flag = 1
                if(flag ==1):
                    unmatched.remove(objects)
                    flag = 0

        # This loop finds the shortest path for each pair and also computes if no path.
        for pair in matched:
                walls = []
                for objects in black_objects:
                    walls.append(objects[0])
                for objects in coloured_objects:
                    walls.append(objects[0])

                walls.remove(pair[0])
                walls.remove(pair[1])
                diagram4.walls = walls
                diagram4.weights = {}
                came_from, cost_so_far = a_star_search(diagram4,pair[0],pair[1])
                if (came_from.get(pair[1]) ==  None and planned_path.get(pair[0]) == None):
                    planned_path.update({pair[0] : ["NO PATH",[], 0]})
                elif(came_from.get(pair[1]) !=  None):
                    path_tup = reconstruct_path(came_from,start=pair[0], goal =pair[1])
                    moves = len(path_tup)-1
                    temp_value = planned_path.get(pair[0])
                    if(planned_path.get(pair[0]) == None or temp_value[-1] ==0):
                        planned_path.update({pair[0] : [pair[1], path_tup[1:moves], moves]})
                    else:
                        value_prev = planned_path.get(pair[0])
                        moves_prev = value_prev[-1]
                        if(moves_prev > moves and moves != 0):
                            planned_path.update({pair[0] : [pair[1], path_tup[1:moves], moves]})

        # This loops adds unmatched objects to dictionary (planned_path).
        for objects in unmatched:
                planned_path.update({objects[0] : ["NO MATCH",[], 0]})

        # This loops completes the final required dictionary for planned_path by correcting the co-ordinates.
        for key, value in planned_path.iteritems():
            key = correctTuple(key)
            if(isinstance(value[0],str) is not True):
                value[0] = correctTuple(value[0])
            temp.update({key:[value]})
            print key ,":", value
        planned_path = temp

        print(" ")
        print("===final form dict ===")
        print(planned_path)
        print(" ")
        print("===final occupied grid ===")
        print(occupied_grids)
        print("length: "),len(occupied_grids)
        # #### NO EDIT AFTER THIS

'''
Below part of program will run when ever this file (task1_main.py) is run directly from terminal/Idle prompt.

'''
if __name__ == '__main__':

    # change filename to check for other images
    image_filename = "test_images/test_image4.jpg"

    main(image_filename)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

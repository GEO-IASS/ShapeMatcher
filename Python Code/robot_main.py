# import the necessary packages
import numpy as np
import cv2
import collections
from robotpath import *
from robocontrol import *

class Queue:
    def __init__(self):
        self.elements = collections.deque()

    def empty(self):
        return len(self.elements) == 0

    def put(self, x):
        self.elements.append(x)

    def get(self):
        return self.elements.popleft()

# utility functions for dealing with square grids
def from_id_width(id, width):
    return (id % width, id // width)

def draw_tile(graph, id, style, width):
    r = "."
    if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = u'\u2192'
        if x2 == x1 - 1: r = u'\u2190'
        if y2 == y1 + 1: r = u'\u2193'
        if y2 == y1 - 1: r = u'\u2191'
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "R"
    if id in graph.walls: r = "X" * width
    return r

def draw_grid(graph, width=2, **style):
    for y in range(graph.height):
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width)),
        print(" ")

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
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Class to contruct a square grid.
class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return id not in self.walls

    '''
    * Function Name: neighbors
    * Input: id
    * Output: results
    * Logic: gives neighbours for a given grid position
    * by moving  +1,-1 unit in both x and y axis
    * Example Call: graph.neighbors(current)
    '''
    def neighbors(self, id):
        (x, y) = id
        results = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)]
        if (x + y) % 2 == 0: results.reverse()
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results


# Class to assign weights in the grid.
class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        SquareGrid.__init__(self, width, height)
        self.weights = {}

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
* Input: a,b where a & b are co-ordinates of two points on the grid
* Output: estimated distance between the two points
* Logic: Manhattan distance is calculated using formula: abs(x1 - x2) + abs(y1 - y2)
* Example Call: estimate = heuristic(goal, neighbour)
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


'''
*Function name :auto_canny
*Example call : auto_canny('test_image4.jpg')
*Output : gives out the edges extracted from the image using canny edge detection
*Logic : apply automatic Canny edge detection using the computed median
'''
def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

import time

# Information about the web_camera
camera_port = 0
#ramp_frames decides how many frames to ignore before capturing the image.
ramp_frames = 45
camera = cv2.VideoCapture(1)

#Capture the image
def get_image():
 retval, im = camera.read()
 return im

'''
ignore the first few frames
'''
for i in xrange(ramp_frames):
 temp = get_image()
print("Taking image...")

'''
capture the image after eliminating a few frames and write it to a file
'''
camera_capture = get_image()
file = r"C:test_image2.png"
cv2.imwrite(file, camera_capture)
del(camera)

'''
read the file that was written in the above code.
'''
img = cv2.imread(file,1)


# load the image-->Threshold it --> blur it slightly to eliminate noise
lower_hsv = np.array([0, 0, 0])
upper_hsv = np.array([179, 255, 15])
image=img
image = image
gray = cv2.inRange(image, lower_hsv, upper_hsv)
blurred = cv2.GaussianBlur(image, (5, 5), 0)

#Do auto_canny edge_detection on the blurred image to get the edges.
auto = auto_canny(blurred)

'''
*This part of the code is used to find the boundary of the arena and crop out of the inside of the boundary
*Logic : find the contours on edges of the image , find the biggest contour , approximate the contour ,
*extract co-ordinates of the approcimated (biggest) contour and store that in a new image called cropped.
'''
contours, hierarchy = cv2.findContours(auto.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
sec_biggest = 0
max_area = 0
# min_size = thresh1.size/4
index1 = 0
for i in contours:
    area = cv2.contourArea(i)
    if area > 10000:
        peri = cv2.arcLength(i, True)
    if area > max_area:
        biggest = index1
        max_area = area
    index1 = index1 + 1
# approximate the biggest contour
approx = cv2.approxPolyDP(contours[biggest], 0.05 * peri, True)

x1 = approx[0][0][0]
y1 = approx[0][0][1]
x2 = approx[1][0][0]
y2 = approx[1][0][1]
x3 = approx[3][0][0]
y3 = approx[3][0][1]
x4 = approx[2][0][0]
y4 = approx[2][0][1]

# print the co-orsinates of the biggest contour.
print (x1, y1)
print (x2, y2)
print (x3, y3)
print (x4, y4)

#Map these points to a new image of pre-defined size called cropped.
pts1 = np.float32([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
pts2 = np.float32([[0, 0], [0, 301], [453, 0], [453, 301]])  # remarking each four side of the cropped image
persM = cv2.getPerspectiveTransform(pts1, pts2)
cropped = cv2.warpPerspective(image, persM, (450, 300))
print("cropping done")
cropped = cv2.resize(cropped, (450, 300))

'''
*This part of the code is used for acessing door area objects and storing them as templates
*NOTE: Templates are produced/stored at door execution.
'''
template_info=[]
y = 0
for x in range(0, 6):
    #Divide the cropped image into grids.
    dst = cropped[50 * x:(50 * x + 50), 50 * y:(50 * y + 50)]
    blurred = cv2.GaussianBlur(dst, (5, 5), 0)
    #find the edges in the grid.
    auto = auto_canny(blurred)
    '''
    *This part of the code is used to extract to the biggest contour in the cropped grid,
    *and find a bounding box for the grid and use that bounding_box to extract the template from the door area.
    *The COLOR of the marker in the door area is estimated by finding maximum of the averaged histogram.
    '''
    contours, hierarchy = cv2.findContours(auto.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        #Bounding rectangle is used to extract the ROI of marker and create a template
        x1, y1, w1, h1 = cv2.boundingRect(contours[0])
        print (y1, h1, x1, w1)
        fin_crop = dst[(y1 - 1):(y1 + h1 + 1), (x1 - 1):(x1 + w1 + 1)]
        hist = cv2.calcHist([fin_crop], [0], None, [256], [0, 256])
        avg = np.average(fin_crop, axis=0)
        avgc = np.average(avg, axis=0)
        print(max(avgc))
        print(avgc.tolist().index(max(avgc)))
        name = "t" + str(x) + str(y) + ".jpg"
        #STORING THE TEMPLATES onto HardDisk
        cv2.imwrite(name, fin_crop)
        tup=(y,x)

        #This part is used to calculate the colour of the bounding rectangle(i.e color of marker)
        hist = cv2.calcHist([fin_crop], [0], None, [256], [0, 256])
        avg = np.average(fin_crop, axis=0)
        avgc = np.average(avg, axis=0)
        col = ['blue', 'green', 'red']
        #color of the marker in the door area
        col_obs = col[avgc.tolist().index(max(avgc))];
        template_info.append(tup)
print("---door info---")
print template_info

'''
This part of the code is used to find both objects and obstacles from the working area and store them in obstacles_list
'''
obstacles = []
lower=np.array([0,24,0])
upper=np.array([255,255,255])
color_dst=cv2.cvtColor(cropped,cv2.COLOR_BGR2HSV)
color_mask=cv2.inRange(color_dst,lower,upper)
color_cropped=cv2.bitwise_and(cropped,cropped,mask=color_mask)
#Traverse through each cell
for y in range(1, 9):
    for x in range(0, 6):
        #crop each cell -->dst
        dst = cropped[50 * x:(50 * x + 50), 50 * y:(50 * y + 50)]
        # using the edge info and the biggest contour the ROI(Region og interest) for the obstacle/object is found from the cell
        blurred = cv2.GaussianBlur(dst, (5, 5), 0)
        auto = auto_canny(blurred)
        contours_mini_obs, hierarchy = cv2.findContours(auto.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #FIND the biggest contour
        if (len(contours_mini_obs) > 0 ):
            tup = (y, x)

            biggest = 0
            sec_biggest = 0
            max_area = 0
            index1 = 0
            for i in contours_mini_obs:
                area = cv2.contourArea(i)
                if area > max_area:
                    biggest = index1
                    max_area = area
                index1 = index1 + 1

            #append the co-ordinates & color of the obstacle/object.
            x1, y1, w1, h1 = cv2.boundingRect(contours_mini_obs[biggest])
            fin_crop = dst[(y1):(y1 + h1 ), (x1):(x1 + w1)]

            hist = cv2.calcHist([fin_crop], [0], None, [256], [0, 256])
            avg = np.average(fin_crop, axis=0)
            avgc = np.average(avg, axis=0)
            col=['blue','green','red']
            #colour of the object/obstacle.
            col_obs=col[avgc.tolist().index(max(avgc))]
            obstacles.append(tup)

print("-----obs----")
print(obstacles)
pairs=[]
pairs_dict={}
matched=[]

'''
*This part of the code is used to do template matching and Color matching between markers in door area and objects in color area.
the matched co-ordinates are stored in pairs list.

'''
yt = 0
for xt in range(0, 6):
    name = "t" + str(xt) + str(yt) + ".jpg"
    #Read the template --Refer line 323 of this code on how they were created.
    template = cv2.imread(name, 1)
    #Find color of the marker
    hist = cv2.calcHist([template], [0], None, [256], [0, 256])
    avg = np.average(template, axis=0)
    avgc = np.average(avg, axis=0)
    col = ['blue', 'green', 'red']
    col_temp = col[avgc.tolist().index(max(avgc))]

    template = cv2.imread(name, 0)
    col_cell=''
    #Compare the template and color with each cell in the working area.
    for y in range(1,9):
        for x in range(0,6):
            cell = cropped[50 * x:(50 * x + 50), 50 * y:(50 * y + 50)]

            blurred = cv2.GaussianBlur(cell, (5, 5), 0)
            auto = auto_canny(blurred)
            contours, hierarchy = cv2.findContours(auto.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                tup = (x, y)

                biggest = 0
                sec_biggest = 0
                max_area = 0
                index1 = 0
                for i in contours:
                    area = cv2.contourArea(i)
                    if area > max_area:
                        biggest = index1
                        max_area = area
                    index1 = index1 + 1

                x1, y1, w1, h1 = cv2.boundingRect(contours[biggest])

                cell_crop = cell[(y1):(y1 + h1), (x1):(x1 + w1)]
                #cv2.imshow('lol',cell_crop)
                hist_cell = cv2.calcHist([cell_crop], [0], None, [256], [0, 256])
                avg_cell = np.average(cell_crop, axis=0)
                avgc_cell = np.average(avg_cell, axis=0)
                col_cell = ['blue', 'green', 'red']
                col_cell = col_cell[avgc_cell.tolist().index(max(avgc_cell))]

            img_gray = cv2.cvtColor(cell, cv2.COLOR_BGR2GRAY)
            w, h = template.shape[::-1]
            res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
            threshold = 0.83
            loc = np.where(res >= threshold)

            #template matching
            for pt in zip(*loc[::-1]):
               # cv2.rectangle(cell, pt, (pt[0] + w, pt[1] + h), (0, 255, 255), 1)
                matched_img=cropped[pt[0]:(pt[0]+w),pt[1]:(pt[1]+h)]
            if(np.amax(res)>=threshold and col_cell==col_temp):
                pairs_dict[(y,x)]=(yt,xt)
                pairs.append([(yt,xt),(y,x)])
                matched.append((y,x))
print ("---pairs---")
print pairs
print("---dict-----")
print pairs_dict
print("---matched---")
print matched
pairs.remove([(0,1),(3,0)])
pairs.remove([(0,3),(7,0)])
result=[]
print ("---pairs---")
print pairs
print("---dict-----")
print pairs_dict
print("---matched---")
print matched
for door_obj in template_info:
    obstacles.append(door_obj)
print "obssssssssssssss",obstacles

obj_path_array=[]
result_path_array=[]
#----------------------------Astar---------------------------------
'''
*This part of the code finds the shortest path from  the bot to the next object to be picked
'''
bot_pos=(4,2)
obstacles.remove(bot_pos)
for i in range(0,6):
    min_path_len = 300
    obj_next = (-1, -1)
    obj_path = []
    result_path=[]

    for test_ele in pairs:
        temp_wall=[]
        for ele in obstacles:
            temp_wall.append(ele)
        temp_wall.remove(test_ele[1])

        diagram4 = GridWithWeights(9, 6)
        diagram4.walls = temp_wall
        diagram4.weights = {}

        came_from, cost_so_far = a_star_search(diagram4, bot_pos,test_ele[1])

        if (came_from.get(test_ele[1]) == None):
            print('nope')
        elif (came_from.get(test_ele[1]) != None):
            path_tup = reconstruct_path(came_from, start=bot_pos, goal=test_ele[1])

            if (len(path_tup) < min_path_len):
                min_path_len=len(path_tup)
                obj_path=path_tup
                obj_next=test_ele[1]
                result=test_ele[0]


    #go to the selected_item
    #pick it up
    #print "obj_next :", obj_next
    obstacles.remove(obj_next)
    #print "obs",obstacles
    #matched.remove(obj_next)
    #print "matched" ,matched
    pairs.remove([result,obj_next])
    result_item=result
    #print "resut_item",result_item
    print "bot --> object :",obj_path
    obj_path_array.append(obj_path)


    diagram5 = GridWithWeights(9, 6)
    diagram5.walls = obstacles
    diagram5.weights = {}
    obstacles.remove(result_item)
    #print "door open at : ",obstacles
    '''
    *This part of the code finds the shortest path from  the objected picked to the color marker in the door area
    '''

    came_from2, cost_so_far2 = a_star_search(diagram5, obj_next, result_item)
    if (came_from2.get(result_item) == None):
        print('nope')
        result_path=[]
    elif (came_from2.get(result_item) != None):
        path_tup = reconstruct_path(came_from2, start=obj_next, goal=result_item)
        result_path=path_tup

    print "object--> door :",result_path
    result_path_array.append(result_path)
    print "drop the obj at -> ", result_item

    print "\n\n"
    obstacles.append(result_item)
    bot_pos=result_path[-2]

'''
    This part of the code is used to move the robot along the computed paths.perormmovement_pick and performmovement_drop are functions from robopath.py
'''
for i in range(0,len(obj_path_array)):
    performmovement_pick(obj_path_array[i])
    performmovement_drop(result_path_array[i])

#show for debugging
cv2.imshow("cropped", cropped)
blurred = cv2.GaussianBlur(image, (5, 5), 0)
auto = auto_canny(blurred)
cv2.imshow("Edges", auto)
cv2.waitKey(0)

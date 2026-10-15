# CLASE DE AYUDA PARA SEGMENTAR MEDIANTE VISION CLASICA COLORES
# -*- coding: utf-8 -*-
# from __future__ import print_function
import cv2 as cv # openCV
import numpy as np
import os
import matplotlib.pyplot as plt
from enum import Enum

MIN_MATCHES = 20 # minimum number of matches to the query to consider it good
SIFT_MATCH_THRESHOLD = 0.75 # threshold to consider matching between keypoints
DATA_DIR = os.path.join(os.getcwd(), "data") # data directory with the query image
QUERY_IMG = 'query_bvb.jpg' # query image of the target to recon by the robot
QUERY_PATH = os.path.join(DATA_DIR, QUERY_IMG)
# auxiliar variable to make use of sift descrip. and query, make True if so for waffle sim
SIFT = True
# number of pixels which match a mask of a clear way (determined experimentally)
#MIN_PIXELS = 65962 in gazebo
MIN_PIXELS = 63000 # inside the robotics lab
# class to recognize and segment the target, also detects obstacles
class Recon:
    def __init__(self, color, myWebcam=False, tb2=True):
        self.connected = False # webcam USB connection
        self.webcam = myWebcam
        self.color = color # color to segment
        if myWebcam: # for testing purposes based on camera without turtlebot
            self.cam = cv.VideoCapture(0) # camera input
            self.open = self.cam.isOpened()
            if self.open:
                self.connected = True
        self.frame = None # current frame
        self.tb2 = tb2 # checks if we use the real turtlebot2
        self.singleImg = False # to process an image instead of frames
        self.query_threshold = MIN_MATCHES # number of keypoints required to assert a match
        self.SIFT = SIFT # to use the descriptor
        self.floor_obstacles = False # checks any given obstacles in an area

    def resize(self, new_size): # resize the frame using bicubic interpolation
        return cv.resize(self.frame, new_size, interpolation= cv.INTER_CUBIC)
        
    def getFrameShape(self): # returns the current frame shape
        return self.frame.shape
    
    def setFrame(self, frame): # set the frame while resizing by interpolation
        self.frame = np.array(frame)

    # color-based segmentation
    def detect_color(self, frame):
        # gaussian blur to reduce noise
        blur = cv.GaussianBlur(frame, (3,3), 0)
        # convert to HSV to segment object
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
        # define the range to detect and create a mask
        binary_mask, res = None, None
        if self.color == 'red':
            lower_red_1 = np.array([0,100,20], np.uint8)
            upper_red_1 = np.array([5,255,255], np.uint8)
            lower_red_2 = np.array([175,100,20], np.uint8)
            upper_red_2 = np.array([179,255,255], np.uint8)
            # 2 masks to detect red (light and dark)
            mask1 = cv.inRange(hsv, lower_red_1, upper_red_1)
            mask2 = cv.inRange(hsv, lower_red_2, upper_red_2)
            # binary_mask using threshold to detect red
            binary_mask = cv.add(mask1, mask2)  
        
        elif self.color == 'yellow':
            lower_yellow = np.array([15, 100, 20], np.uint8)
            upper_yellow = np.array([45, 255, 255], np.uint8)
            binary_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
        
        elif self.color == 'green':
            lower_g = np.array([40, 100, 20], np.uint8)
            upper_g = np.array([90, 255, 255], np.uint8)
            binary_mask = cv.inRange(hsv, lower_g, upper_g)
        else:
            print("Error in color attribute. Try 'red' | 'yellow' | 'green' ")
            return None, None, None
        # apply erosion to reduce false positives
        erosion = None
        erosion_type = cv.MORPH_ELLIPSE
        size = (9,9)
        # structuring element
        element = cv.getStructuringElement(erosion_type, size)
        # mask with erosion applied
        erosion = cv.erode(binary_mask, element)
        # mask applied to real RGB frame
        if binary_mask is not None:
            # bitwise-AND mask 
            res = cv.bitwise_and(frame, frame, mask= binary_mask)
        
        return binary_mask, erosion, res

    # returns the largest bounding box according to the image segmented
    def getBoundingBox(self, img, contours):
        max_area = 0 # heuristic largest mask segmented
        most_kp = 0 # heuristic most keypoints from SIFT 
        pos = (0,0) # position of the bb (x,y)
        size = (0,0) # size of the bb (x+w, y+h)
        # find the largest bounding box -> detect the person wearing yellow clothing
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            # heuristics
            if not self.SIFT:
                # get the largest bbox due to mask segmented
                if w * h > max_area: 
                    pos  = (x,y)
                    size = (x + w, y + h)
                    max_area = w * h
            else: # we are using the descriptor
                # check if most of the keypoints detected are in the bbox
                xmin, ymin = x, y
                xmax, ymax = x + w, y + h
                counter = 0 # counter of the keypoints inside the bbox 
                for kp in self.keyp_list: # keyp_list is a list of tuples (x, y)
                    kp_x, kp_y = kp
                    if kp_x > xmin and kp_x < xmax and kp_y > ymin and kp_y < ymax:
                        counter += 1
                if counter > most_kp:
                    pos  = (x,y)
                    size = (x + w, y + h)
                    most_kp = counter # added
        # draw the bounding box on the img
        cv.rectangle(img, pos, size, (0,0,255), 2)
        # bbox info
        bbox_info = np.array([pos, size])
        return img, bbox_info

    # draws the bounding box of the greatest contour detected
    def draw_bounding_box(self, frame, mask, mode=cv.RETR_EXTERNAL):
        # get the contours
        if self.webcam or self.singleImg or self.tb2:
            contours, _ = cv.findContours(mask, mode, cv.CHAIN_APPROX_SIMPLE)
        else:
            _ , contours, _ = cv.findContours(mask, mode, cv.CHAIN_APPROX_SIMPLE)
        # draw the contours in a copy of the frame
        img_contoured = frame.copy()
        img_array = np.asarray(img_contoured)
        try:
            # get the bounding box 
            bbox, bbox_info = self.getBoundingBox(img_array, contours)
        except Exception as e:
            return None,None
        return img_array, bbox_info
    
    # getter (boolean value) according to obstacles in the way
    def floor_plane_obstacles(self):
        return self.floor_obstacles

    # draws the trapezoid area for obstacle detection
    def draw_trapezoid(self):
        if self.frame is None:
            rospy.loginfo("Could not draw the trapezoid used for floor plane extraction")
            return None
	# draws into the given frame the trapezoid used for floor plane extraction
        res = self.frame.copy()
	# define the points for the trapezoid
        height, width = res.shape[:2]
	# each point is given as (x,y)
        top_width = int(width/4)  # Top side of the trapezoid is third the image width
        bottom_width = int(width * 0.9)  # Bottom side of the trapezoid is 90% of the image width
        top_y = int((height*8)/10)  # Y-coordinate for the top of the trapezoid
        bottom_y = int(height)  # Y-coordinate for the bottom of the trapezoid
        # define the points of the trapezoid area to scan
        top_left = ((width - top_width) // 2, top_y)
        top_right = ((width + top_width) // 2, top_y)
        bottom_left = ((width - bottom_width) // 2, bottom_y)
        bottom_right = ((width + bottom_width) // 2, bottom_y)
		
        # create a mask and trapezoid by points
        mask = np.zeros_like(res, dtype=np.uint8)
        trapezoid_points = np.array([top_left, top_right, bottom_right, bottom_left])
        # fill the poligon in yellow (R=0, G=255, B=255)
        cv.fillPoly(mask, [trapezoid_points], (0,255,255))
	# combine both trapezoid mask with original frame 
        trap = cv.bitwise_and(res, mask)
        # interpolate the trapezoid to a squared view 
        dst_shape = (int(width), int(height))
        src = np.full(dst_shape, 0)
        src = np.zeros_like(src, dtype=np.uint8) # copy mask 
        # points in src image defining the points to proyect (inputs) and outputs in the dst image
        input_pts = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32) 
        output_pts = np.array([[0,0], [dst_shape[1] -1, 0], [dst_shape[1] -1, dst_shape[0] -1], [0, dst_shape[0]]], dtype=np.float32)
        # transformation matrix
        mt = cv.getPerspectiveTransform(input_pts, output_pts)
        # perform the perspective matrix wrap
        dst = cv.warpPerspective(trap, mt, dst_shape, flags=cv.INTER_LINEAR)
        # binarize the gray img -> pursue object detection by edges
        dst_gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
        # perform gaussian adaptive threshold to outline edges in the trapezoid area
        det = cv.adaptiveThreshold(dst_gray, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)
        # count the non zero pixels
        pixels = cv.countNonZero(det)
        #print(pixels)
        # if the mask loses positive pixels it means there is an object in the trapezoid area
        if pixels < MIN_PIXELS:
            self.floor_obstacles = True
            
        else:
            self.floor_obstacles = False
            
        # return the trapezoid on the frame drawn, the proyection to a rectangle from it, the mask of the object detected
        return trap, dst, det
		
    # reads from the WebCam and outputs the live feed + detection feed
    def read_from_webcam(self, sift=True, query_path=None): 
        if not self.connected:
            print("Error: La camara no esta conectada")
            return
        print("Reading camera...")
        while(self.open):
            ret, frame = self.cam.read()
            if not ret:
                quit()
            # show height and width of the img: mi webcam {480, 640, 3}
            self.setFrame(frame)
            # segmentation based on color
            mask, erosion, segmented = self.detect_color(frame)
            # try matching query and current frame
            match = None # image of the keypoints detected in current frame and query (target)
            if self.SIFT:
                match = self.matchingSIFT(query_path) # returns image with the query and current frame matching keypoints

            # draw bounding box of the largest contour detected around the keypoints given by SHIFT
            bbox, _ = self.draw_bounding_box(frame, erosion)

            try:
                # show the current video frames
                #cv.imshow('Erosion applied to mask', erosion)
                #cv.imshow('Mask image', mask)            
                cv.imshow('Bounding box detectado', bbox)
                #if match is not None:
                #    cv.imshow('Query matching', match)
            
            except Exception as e:
                self.open = False
                print("Error showing the bounding box in the current frame", e)  
            # exit using key
            if cv.waitKey(25) & 0xFF == ord('q'):
                self.open = False
       
        self.cam.release()
        cv.destroyAllWindows()
        print("All windows closed")

    """
    WAFFLE camera read method
    """
    # read the onboard waffle camera
    def read_from_wafflecam(self, query_path=QUERY_PATH): 
        if self.frame is None:
            print("Error: La camara no esta conectada")
            return
        # segmentation based on color
        mask, erosion, segmented = self.detect_color(self.frame)
        # try matching query and current frame
        match = None # image of the keypoints detected in current frame and query (target)
        if self.SIFT:
            match = self.matchingSIFT(query_path) # returns image with the query and current frame matching keypoints
        # draw bounding box of the largest contour detected (if there is no detection returns none, none)
        bbox, bbox_info = self.draw_bounding_box(self.frame, erosion)
        # draw the trapezoid for the floor plane extraction
        trap, proy, det = self.draw_trapezoid()
        try:
            # show the current video frames
            #cv.imshow('Floor plane extraction', trap)
            #cv.imshow('Proyection of the trapezoid', proy)
            #cv.imshow('Adaptive Gaussian Thresholding', det)

            if bbox is not None:              
                cv.imshow('Bounding box detected', bbox)
            #if match is not None:
            #    cv.imshow('Query matching', match)

        except Exception as e:
            print("Error showing video frames in read_from_wafflecam: ", e)
            pass

        # exit using key
        if cv.waitKey(25) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            print("All windows closed")

        # bbox_info is a numpy array of [pos=(x,y), size=(x+w,y+h) ]
        return bbox_info

    def process_single_img(self, img_path, singleImg=True):
        img = cv.imread(img_path)
        self.setFrame(img)
        self.singleImg = singleImg
        # segmentation based on color
        mask, erosion, segmented = self.detect_color(self.frame)
        # draw bounding box of the largest contour detected (if there is no detection returns none, >
        bbox, bbox_info = self.draw_bounding_box(self.frame, erosion)

        try:
            # show the current video frames
            cv.imshow('Erosion applied to mask', erosion)
            cv.imshow('Mask image', mask)
            cv.imshow('Bounding box detected', bbox)
            cv.waitKey(0)
            cv.destroyAllWindows()

        except Exception as e:
            print("Error showing mask | bbox: ", e)
            pass


    def matchingSIFT(self, query_path, img_path=None):
        # frame copy
        frame = None
        # if there is img path check the image given
        if img_path is not None:
            self.setFrame(cv.imread(img_path, cv.IMREAD_GRAYSCALE)) # current frame in grayscale
            frame = cv.imread(img_path) # copy of the current frame in RGB
            frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            # resize both frames to  (820, 620)
            new_size =  (820, 620)
            self.frame = self.resize(new_size)
            frame = self.resize(new_size)

        else: # reading from a camera
            frame = self.frame
            self.frame = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY) # frame to grayscale for SIFT
            
        try:
            # load query img
            query = cv.imread(query_path) # query original
            query_gray = cv.imread(query_path, cv.IMREAD_GRAYSCALE) # query grayscale for SIFT
        except Exception as e:
            print("Error: Query image path wrong: ", e)
            return None

        if query is None or self.frame is None:
            print("Error loading image of the current frame | Error loading image of the query")
            return None

        # create a SIFT descriptor
        sift = cv.SIFT_create()
        # detect keypoints and calc descriptors
        train_keyp, train_desc = sift.detectAndCompute(query_gray, None) # train is actually -> query
        test_keyp, test_desc = sift.detectAndCompute(self.frame, None) # test is actually -> current frame
        # do matching using FLANN and 2 neighbours
        neighbours = 2
        #FLANN parameters
        index_params = dict(algorithm = 1, tress = 5)
        search_params = dict(checks = 50)
        matcher = cv.FlannBasedMatcher(index_params, search_params)
        matches = matcher.knnMatch(train_desc, test_desc, k=neighbours)
        # save the matches whose distance is considered good enough
        good = list()
        umbral = SIFT_MATCH_THRESHOLD # threshold distance to consider a good match
        self.keyp_list = [] # list of keypoints coordinates in the current frame
        for m,n in matches:
            if m.distance < umbral* n.distance:
                good.append(m)

        if len(good) > self.query_threshold:
            # save keyp_list with the tuple coords of the keypoints to draw the bbox
            for match in good:
                try:
                    x, y = test_keyp[match.trainIdx].pt # coordinates of the keypoint in the current frame
                    self.keyp_list.append( (x, y) )
                except Exception as e:
                    print("Error getting keypoint coordinates: ", e)
                    continue
        # Draw the result
        draw_params = dict(matchColor = (0,255,0), singlePointColor = (255,0,0))
        # Draw the matches in the current frame
        imageMatches = cv.drawMatches(query, train_keyp, frame, test_keyp, good, None, **draw_params)

        return imageMatches

"""
import os
import argparse
# ZONA DE PRUEBAS de SEGMENTACION
if __name__ == "__main__":
    # Configurar el analizador de argumentos
    parser = argparse.ArgumentParser(description="Seguimiento de color con la clase ReconPerson")
    parser.add_argument(
        "--track-color",
        type=str,
        default="yellow",  # Color predeterminado
        help="Color a rastrear (opciones: 'red', 'yellow', 'green')",
    )
    args = parser.parse_args()

    # Leer el color desde los argumentos de línea de comandos
    track_color = args.track_color.lower()
    if track_color not in ["red", "yellow"]:
        print("Color no soportado. Usa 'red' o 'yellow'.")
    else:
        print("Realizando seguimiento al color: ", track_color)

    recon = Recon(track_color)
    DATA_DIR = os.path.join(os.getcwd(), "data")
    # imagenes del supuesto frame actual y el objetivo
    img = 'pasillo3.jpg'
    query = 'query_bvb.jpg'
    img_path = os.path.join(DATA_DIR, img)
    query_path = os.path.join(DATA_DIR, query)
    #TODO: Reconocer matches mediante SITF (SURF esta pantentado pero es mas eficiente computacionalmente)
    #recon.read_from_webcam(True, query_path)
    
    #TODO: Procesar imagen de un directorio
    #recon.process_single_img(os.path.join(DATA_DIR, img_name))
"""
    
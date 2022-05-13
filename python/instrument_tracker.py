from collections import deque
from pandas._libs.tslibs import timestamps
import cv2
import argparse
import time
import imutils
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import time
from scipy import interpolate

# run in terminal:
# python3 python/instrument_tracker.py -v videos/MVI_9976.mov 

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the video file")
args = vars(ap.parse_args())
filename = args["video"]
vs = cv2.VideoCapture(filename)
print(filename.split(", "))

# allow video file to warm up
time.sleep(1.0)

# frames per second
fps = 1/25

# store tracked positions and time stamp
pts_pos = deque([[0, 0], [0, 0]])
pts_ang = deque([[0], [0]])
pts_time = deque([[0], [0]])
pts_pos.clear()
pts_ang.clear()
pts_time.clear()

pos_tip = None
ang_tip = None

# crop frame to relevant region of interest
# crop = [[61, 934], [490, 1451]]  # 9978
crop = [[54, 1029], [483, 1445]] # 9976

# define the lower and upper boundaries of
# colors in the HSV color space
# green
h_lim_g = [30, 65]
s_lim_g = [30, 100]
v_lim_g = [100, 200]

# blue
h_lim_b = [100, 120]
s_lim_b = [50, 130]
v_lim_b = [100, 220]

# yellow
h_lim_y = [10, 30]
s_lim_y = [160, 180]
v_lim_y = [100, 200]
# pink
h_lim_p = [160, 165]
s_lim_p = [115, 192]
v_lim_p = [100, 200]
# orange
h_lim_o = [1, 6]
s_lim_o = [170, 190]
v_lim_o = [100, 200]

# # green 9978
# h_lim_g = [30, 65]
# s_lim_g = [30, 100]
# v_lim_g = [100, 250]
# # blue 9978
# h_lim_b = [100, 125]
# s_lim_b = [20, 60]
# v_lim_b = [100, 220]



def contour_from_hsv(hsv, h_lim, s_lim, v_lim):

        # perform a series of dilations and erosions to remove small
        # blobs left in the mask
        hsv_lower = (h_lim[0], s_lim[0], v_lim[0])
        hsv_upper = (h_lim[1], s_lim[1], v_lim[1])
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        mask = cv2.dilate(mask, None, iterations=2)
        # mask = cv2.erode(mask, None, iterations=3)
        # mask = cv2.dilate(mask, None, iterations=2)
        
        # find contours
        cnts = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        return cnts, mask

def plot_and_show_img(img):
    plt.figure()
    plt.imshow(img)
    plt.show()

def merge_contours(cnts):
    cnts_merged = cnts[0]
    # merge all contours
    for cnt in cnts:
        # only use large contours
        if cv2.contourArea(cnt) > 10:
            cnts_merged = np.concatenate((cnts_merged, cnt), axis=0)
    return cnts_merged

def center_from_cnt(cnt):
    mnt = cv2.moments(cnt)
    center = (
        int(mnt["m10"] / mnt["m00"]),
        int(mnt["m01"] / mnt["m00"]))
    return center

def center_pose_from_points(point_0, point_1):

    # midpoint between the red and green markers
    center = [
        int((point_0[0]+point_1[0])/2),
        int((point_0[1]+point_1[1])/2)]

    # find the angle of the magnetic tip
    # vector pointing in direction of red to green marker
    a = np.array([
        point_0[0]-point_1[0],
        point_0[1]-point_1[1], 0.])
    # normal vector
    b = np.array([0., 1., 0.])
    # angle between pointing vector and normal vector
    rad = np.arctan2(np.cross(b, a), np.dot(a, b))[2]
    deg = np.rad2deg(rad)
    return center, deg

def pose_from_two_cnts(img, cnts_0, cnts_1, dist_lim=[0,1000]):
        """
        Calculates center pose between two contours given 
        a max and min distance theay are allowed to be apart.
        Returns position of the midpoint and orientation (angle
        in deg).
        """
        center = None
        angle = None
        dist_ok = None
        frame_r_g = img.copy()
        center_0 = None
        center_1 = None
        # loop through all green and red contours found in image,
        # measure distance of center of mass and decide if contour
        # are the tip or just an artefact

        # loop through contours 1
        for cnt_1 in cnts_1:

            # draw red box
            rect = cv2.minAreaRect(cnt_1)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            frame_r_g = cv2.drawContours(
                frame_r_g, [box], 0, (0, 0, 255), 2)

            # loop through contours 0
            for cnt_0 in cnts_0:

                if cv2.contourArea(cnt_0) > 150 and cv2.contourArea(cnt_1) < 40:
                    return center, angle, dist_ok

                # draw green box
                rect = cv2.minAreaRect(cnt_0)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                frame_r_g = cv2.drawContours(
                    frame_r_g, [box], 0, (0, 255, 0), 2)
                cv2.imshow('frame', frame_r_g)
                try:
                    center_0 = center_from_cnt(cnt_0)
                    center_1 = center_from_cnt(cnt_1)
                except:
                    pass
                # distance between contours 0 and 1
                distance = np.sqrt(
                    (center_0[0]-center_1[0])**2+(center_0[1]-center_1[1])**2)

                # check if the contours are the right distance apart,
                # otherwise skip and move to next pair of blobs
                if distance < dist_lim[1] and distance > dist_lim[0]:
                    dist_ok = distance
                    # draw box centered on midpoint between markers 0 and 1
                    center, angle = center_pose_from_points(center_0, center_1) # in deg
                    break
        return center, angle, dist_ok

def pose_from_two_colors(img, img_hsv, hsv_lim_0, hsv_lim_1, dist_lim, box_width, label):   
        position = None
        angle = None
        distance = None

        # segment color blobs
        cnts_0, mask_0 = contour_from_hsv(
            img_hsv, hsv_lim_0[0], hsv_lim_0[1], hsv_lim_0[2])
        cnts_1, mask_1 = contour_from_hsv(
            img_hsv, hsv_lim_1[0], hsv_lim_1[1], hsv_lim_1[2])
        
        # only proceed if at least one contour given
        if len(cnts_0) > 0 and len(cnts_1) > 0:

            # find position of center and direction between two contours
            # given a dristance range they are allowed to be apart
            position, angle, distance = pose_from_two_cnts(
                frame, cnts_0, cnts_1, dist_lim)

            # draw box around tip if tip found
            if position != None:
                rect = ((position[0], position[1]), (box_width, distance*1.1), angle)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                img = cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)

                # draw label as text
                img = cv2.putText(
                    img, label,
                    (int(position[0]+10), int(position[1]-distance*0.7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
        
        return img, position, angle, distance

def contour_poly_fit(cnt, img, order=5):
    """Fits a polynom on contour and draw a dotted line on image"""
    x = cnt[:, :, 0].flatten()
    y = cnt[:, :, 1].flatten()

    poly = np.poly1d(np.polyfit(x, y, order))
    # draw fit as dots
    for _x in range(min(x), max(x), 2):
        img_fit = cv2.circle(img, (_x, int(poly(_x))), 2, [255, 255, 0])
    return img_fit

def skeleton_contours(cnts, blurr=1):
    if len(cnts) > 0:
        black = np.zeros(frame.shape, np.uint8)
        # fill contours
        img = cv2.fillPoly(black.copy(), pts=cnts, color=(255,255,255))
        # turn into bw image
        thin = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        # =1
        thin = cv2.blur(thin, (blurr, blurr))
        _, thin = cv2.threshold(thin, 220, 255, 0)
        # thin image to find clear contours
        thin = cv2.ximgproc.thinning(thin, thinningType=cv2.ximgproc.THINNING_GUOHALL)
        # find contours
        cnts_skel = cv2.findContours(thin, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cnts_skel = cnts_skel[0] if len(cnts_skel) == 2 else cnts_skel[1]

        return cnts_skel
    else:
        return None

def track_body(img):
    cnts_y, mask_y = contour_from_hsv(hsv, h_lim_y, s_lim_y, v_lim_y)
    if len(cnts_y):
        cv2.imshow('mask', mask_y)
        # merge seperated contrours into one big contour
        cnts_y_merged = merge_contours(cnts_y)

    # draw yellow contours
        tracked = cv2.drawContours(img.copy(), cnts_y, -1, (0, 255, 0), 1)

        # Fit polynom on contour and draw on image
        # poly_fit = contour_poly_fit(cnts_y_merged, img.copy())
        # cv2.imshow('poly_fit', poly_fit)
        
        # Skeletonize contours
        cnts = skeleton_contours(cnts_y, blurr=7)
        skel_mask = cv2.drawContours(black.copy(), cnts, -1, (255, 255, 255), 1)
        skel = cv2.drawContours(img.copy(), cnts, -1, (0, 0, 255), 3)
        cv2.imshow('skeleton', skel)
        cv2.imshow('skeleton masked', skel_mask)
        cv2.imshow('instrument body tacked', tracked) 

frame_counter = 0
while True:
    # grab the current frame
    frame = vs.read()
    # handle the frame from VideoCapture or VideoStream
    frame = frame[1] if args["video"] else frame
    # stop when reached the end
    if frame is None:
        break
    
    frame = frame[crop[0][0]:crop[0][1], crop[1][0]:crop[1][1]]

    # generate time stamp
    time_stamp = fps*frame_counter

    # convert frame into hsv color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # new empty image
    black = np.zeros(frame.shape, np.uint8)
    
    pts_pos.appendleft(pos_tip)
    pts_ang.appendleft(ang_tip)
    pts_time.appendleft(time_stamp)

    # loop over the set of tracked points
    for i in range(1, len(pts_pos)):
        # if either of the tracked points are None, ignore
        # them
        if pts_pos[i - 1] is None or pts_pos[i] is None:
            continue

        # draw the connecting lines
        thickness = 2
        buffer = 100
        thickness = int(np.sqrt(buffer / float(i + 1)) * 2.)
        if thickness < 1:
            thickness = 1
        cv2.line(
            frame, pts_pos[i - 1], pts_pos[i], (255, 0, 0), thickness)

    # track tip
    img_tip, pos_tip, ang_tip, dist_colrs = pose_from_two_colors(
        frame.copy(), hsv,
        [h_lim_g, s_lim_g, v_lim_g],
        [h_lim_b, s_lim_b, v_lim_b],
        dist_lim =[55, 65],
        box_width=7,
        label='tip')
    if ang_tip != None:
        pos_tip[0] = pos_tip[0]+int(dist_colrs/2*np.sin(np.deg2rad(ang_tip)))
        pos_tip[1] = pos_tip[1]-int(dist_colrs/2*np.cos(np.deg2rad(ang_tip)))

    # # track insertion point
    # img_tip, pos_insertion, ang_insertion, dist_insertion_colrs = pose_from_two_colors(
    #     frame.copy(), hsv,
    #     [h_lim_o, s_lim_o, v_lim_o],
    #     [h_lim_p, s_lim_p, v_lim_p],
    #     dist_lim =[40, 50],
    #     box_width=20,
    #     label='insertion')

    # # track instrument body    
    # track_body(img_tip)

    # plot hsv frame to readout hsv values for parameter
    # tuning
    # plot_and_show_img(hsv)

    cv2.imshow('tip tracked', img_tip)

    frame_counter = frame_counter + 1

    # if center != None:
    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

now = time.strftime("%Y%m%d")
filename_csv = now+args["video"]
# export data
# remove the None values
pts_pos_export = []
pts_ang_export = []
pts_time_export = []
for i in range(len(pts_pos)):
    if pts_pos[i] != None:
        pts_pos_export.append(pts_pos[i])
        pts_ang_export.append(pts_ang[i])
        pts_time_export.append(pts_time[i])
pts_pos_export = np.transpose(pts_pos_export).astype(float)
pts_ang_export = np.transpose(pts_ang_export).astype(float)
pts_time_export = np.transpose(pts_time_export).astype(float)
pts_pos_export = np.flip(pts_pos_export, axis=1)
pts_ang_export = np.flip(pts_ang_export)
pts_time_export = np.flip(pts_time_export)

# convert units from px to m and recentered
length_m = 0.2
length_px = 981.
offset_xy = [470., 470.]
pts_pos_export[0] = (
    pts_pos_export[0]-offset_xy[0])*length_m/length_px
pts_pos_export[1] = -(
    pts_pos_export[1]-offset_xy[1])*length_m/length_px

# convert angle from deg to rad
pts_ang_export = np.deg2rad(pts_ang_export)

df = pd.DataFrame({
    't': pts_time_export,
    'x': pts_pos_export[0],
    'y': pts_pos_export[1],
    'ang': pts_ang_export})

now = time.strftime("%Y%m%d")
experiment_label = args["video"].split('/')[-1].split('.')[0]
filename_csv = now+'_'+experiment_label
print('Done! Exporting trackt points as '+filename_csv+'.csv')

df.to_csv(filename_csv+'.csv', index=False)

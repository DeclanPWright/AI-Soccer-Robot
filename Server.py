import socket
import numpy as np
import cv2 as cv
from cv2 import aruco
from numpy import asarray
import PIL
from PIL import Image
from matplotlib import image
from matplotlib import pyplot
import matplotlib.pyplot as plt
import time
import math
import keyboard


marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

param_markers = aruco.DetectorParameters_create()


cap = cv.VideoCapture(0)
serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

serv.bind(('0.0.0.0', 14232))
serv.listen(5)
print('Server Up')
#connect to bot
conn, addr = serv.accept()
print('Bot Connected')
#main loop

out_of_bounds = (450,1500,250,780)
save = np.zeros((2,4,2))
take = 0
target = (0,0)
have_ball = 0
error = 0
ball_pos = (0,0)
while True:

    
    servo_ang = '90'
    j_turn = 0
    input_target = (0,0)
    rot_id = np.zeros((2))
    stop_distance = 200
    

    id_XY = np.zeros((2,4,2))
    pos_id = np.zeros((2,2))
#Aruco Traker
    ret, frame = cap.read()
    take = 0
    while id_XY[0,0,0] == 0 :
        if take == 1:
            ret, frame = cap.read()
        take = 1
        if not ret:
            print('break')
            break
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)
        if marker_corners:
            for ids, corners in zip(marker_IDs, marker_corners):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()
                cv.putText(
                    frame,
                    f"id: {ids[0]}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200, 100, 0),
                    2,
                    cv.LINE_AA,
                )
                if ids == 0:
                    id_XY[ids] = (corners)
                elif ids == 3:
                    id_XY[1] = (corners)
                #print(ids, "  ", corners)
        #cv.imshow("frame", frame)

    for b in range (2):
        if id_XY[b,0,0] == 0:
            id_XY = save
        else:
            for a in range (4):
                pos_id[b,0] += (id_XY[b,a,0])
                pos_id[b,1] += (id_XY[b,a,1])
            pos_id[b,0] = pos_id[b,0]/4
            pos_id[b,1] = pos_id[b,1]/4
        save = id_XY
    for i in range (2): 
        math_op_pre = (pos_id[i,1]- id_XY[i,0,1])/(pos_id[i,0]- id_XY[i,0,0])

        rot_id[i] = ((math.atan(math_op_pre))*(180/3.141592653589793))-45
        if (pos_id[i,0]- id_XY[i,0,0]) < 0: 
            rot_id[i] += 180
        if rot_id[i] < 0:
            rot_id[i] += 360
    
    
    key = cv.waitKey(1)

#Calculating posisitions of corners

    out_of_bounds = (450,1500,250,830)

    if rot_id[0] < 180:
        if rot_id[0] < 30:
            out_of_bounds = (350,1620,250,1080)
        elif 30 <= rot_id[0] < 60:
            out_of_bounds = (400,1670,200,1030)
        elif 60 <= rot_id[0] < 90:
            out_of_bounds = (450,1720,150,980)
        elif 90 <= rot_id[0] < 120:
            out_of_bounds = (450,1720,100,930)
        elif 120 <= rot_id[0] < 150:
            out_of_bounds = (400,1670,50,880)
        elif 150 <= rot_id[0] < 180:
            out_of_bounds = (350,1620,0,830)
    else:
        if 180 <= rot_id[0] < 210:
            out_of_bounds = (300,1570,0,830)
        elif 210 <= rot_id[0] < 240:
            out_of_bounds = (250,1520,50,880)
        elif 240 <= rot_id[0] < 270:
            out_of_bounds = (200,1470,100,930)
        elif 270 <= rot_id[0] < 300:
            out_of_bounds = (200,1470,150,980)
        elif 300 <= rot_id[0] < 330:
            out_of_bounds = (250,1520,200,1030)
        elif 330 <= rot_id[0]:
            out_of_bounds = (300,1570,250,1080)

    

#Detecting Ball location

    #screen = np.zeros((216,384))
    data = asarray(frame)
    #print(data[0,0])
    data = cv.cvtColor(data, cv.COLOR_BGR2HSV)
    #print(data[538,673])
    output = 0
    count = (0,0)
    
    s = 0
    for y in range (0,1080,5):
        for x in range (0,1920,5):
            if 110 < data[y][x][0] < 130 and data[y][x][1] > 150 and data[y][x][2] > 70:
                output = output + 1
                count = (count[0] + y, count[1] + x)
                #screen[y//5,x//5] = 1
    if output > 10:
        ball_pos_pre = (count[1]/output, count[0]/output)
        if ball_pos_pre[0] !=0:
            ball_pos = ball_pos_pre
    
    #DisplayCheck
    #print(len(data))
    #print(len(data[1]))
    #print(data[0,0])
    #plt.imshow(screen, interpolation='nearest', cmap='gray')
    #plt.savefig('decrypted1.png')
    #plt.show()
    

#Decisionmaking
    print('\n')
    a1 = abs(pos_id[0,0] - ball_pos[0])
    b1 = abs(pos_id[0,1] - ball_pos[1])
    bot_to_ball = math.sqrt((a1*a1) + (b1*b1))
    a2 = abs(pos_id[1,0] - ball_pos[0])
    b2 = abs(pos_id[1,1] - ball_pos[1])
    en_to_ball = math.sqrt((a2*a2) + (b2*b2))
    if bot_to_ball < 150:
        have_ball = 1
    else:
        have_ball = 0
    print('have_ball: ',have_ball)
    if (bot_to_ball < en_to_ball) or (have_ball == 1) or (en_to_ball > 300) or ball_pos[0] > 1200:
        #offense mode
        print('offense')
        if have_ball == 1:
            
            #not past half
            if pos_id[0,0] < 1000:
                print('not past half')
                if pos_id[0,1] > 500:
                    input_target = (1200,580)
                    stop_distance = 150
                else:
                    input_target = (1200,300)
                    stop_distance = 150
            else:
                j_turn = 1
                if -10 > error or error > 10:
                    print('turn_to_kick')
                    if pos_id[1,1] < 540:
                        input_target = (1700,500)
                    else:
                        input_target = (1700,580)
                else:
                    print('kick')
                    servo_ang = '50'

        else:
            print('go to ball')
            input_target = ball_pos
            stop_distance = 100
    else:
        #defense mode
        
        if pos_id[1,0] < 1000:
            print('traking goal defense')
            input_target = (550,pos_id[1,1])
            stop_distance = 50
        else:
            print('basic goal defense')
            print(pos_id[1,0])
            input_target = (500,550)
            stop_distance = 50

    
#PID
    
    #Tunes
    start_straight = 15
    
    
    #Turn PID
    kPT = 0.5
    
    #Straight PID
    kPD = 0.3
    #input_target = pos_id[1]
    #input_target = ball_pos
    my_data = [0,0]
    
    if input_target[0] != 0 and input_target[1] != 0:
        target = input_target
    pos = pos_id[0]
    rot = rot_id[0]
    l_speed = 0
    r_speed = 0
    pre_pid_math = (pos[0] - target[0])/(pos[1] - target[1])
    targ_ang = ((math.atan(pre_pid_math))*(180/3.141592653589793))+90
    if (pos[1] - target[1]) < 0: 
        targ_ang += 180
    targ_ang = 360 - targ_ang
    targ_ang -= 270
    if targ_ang < 0:
        targ_ang += 360

    a = (pos[0] - target[0])
    b = (pos[1] - target[1])

    c = math.sqrt((a*a) + (b*b))
    if c < 100:
        distance = c
    else:
        distance = 100

    if pos_id[0,0] < out_of_bounds[0] or pos_id[0,0] > out_of_bounds[1] or pos_id[0,1] < out_of_bounds[2] or pos_id[0,1] > out_of_bounds[3]:
        if target[0] < out_of_bounds[0] or target[0] > out_of_bounds[1] or target[1] < out_of_bounds[2] or target[1] > out_of_bounds[3]:
            print('out of bounds')
            out_stop = 1
        else:
            out_stop = 0
    else:
        out_stop = 0
    if c > stop_distance:
        if abs(targ_ang - rot) < abs(targ_ang - (rot +  360)):
            print('left_0-360')
            error = targ_ang - rot
        else:
            print('right_0-360')
            error = targ_ang - (rot + 360)
        
        if abs((error) > start_straight or abs(error) > start_straight) or out_stop == 1 or j_turn == 1:
            power = error*kPT
            my_data = (power,power* -1)
            print('turn')
        else:
            print('going forward')
            power = error*kPD
            my_data = (distance + power,distance - power - 5)

        if my_data[0] > 100:
            my_data = (100,my_data[1])
        if my_data[1] > 100:
            my_data = (my_data[0],100)
        if my_data[0] < -100:
            my_data = (-100,my_data[1])
        if my_data[1] < -100:
            my_data = (my_data[0],-100)
    else:
        my_data = (0,0)
    
    #print('target: ',target, 'targ_ang: ',targ_ang)
    print('ball_pos: ',ball_pos)
    #print('bot_to_ball: ',bot_to_ball)
    #print('en_to_ball: ',en_to_ball)
    #print('\n')
    print('error: ',error)
    #print('MD: ', my_data)
    #print('rot: ',rot)
    print('pos: ', pos_id)
    #print('power:',power,',my_data ', my_data)
#Data Send
    
    
    mid = " ".join(map(str,my_data))
    mid = mid + ' ' + servo_ang
    data = mid.encode('utf-8')
    conn.send(data)



#Break
    if key == ord("q"):
        break
    time.sleep(0.1)
cap.release()
cv.destroyAllWindows()
conn.close()
print ('client disconnected')




from __future__ import division 
from math import ceil, floor, sqrt
import copy
import numpy
import sys
from math import cos, sin, tan, atan2, asin, acos

from math import pi as PI
from numpy import sign
from numpy import nan
def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False

def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001

def deg2rad(inp):
    return inp*np.pi/180
    
def RVO_update_single(X_agent, X_neighbor, V_agent, V_neighbor, V_des, Parameter,obs_angle,obs_distance,agent_heading):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""
    ROB_RAD = Parameter.ws_model['robot_radius']+Parameter.ws_model['robot_clearence'] # radius robot ditambah 0.02 clearence
    
    #V_opt = list(V_current)    
    #for i in range(len(X)): # loop untuk seluruh agent
    vA = [V_agent[0], V_agent[1]] # kecepatan dari agent ke i
    pA = [X_agent[0], X_agent[1]] # posisi dari agen ke-i
    RVO_BA_all = []

    clearence = 2.0

    for i in range(len(obs_angle)):
        r = obs_distance[i][2]
        if(abs(r)>0.1):
            clearence_angle_left = numpy.arccos((2*r*r-clearence*clearence)/(2*r*r))
        else:
            clearence_angle_left = deg2rad(10.0)
        r = obs_distance[i][0]
        if(abs(r)>0.1):
            clearence_angle_right = numpy.arccos((2*r*r-clearence*clearence)/(2*r*r))
        else:
            clearence_angle_right = deg2rad(10.0)
        transl_vB_vA = [pA[0], pA[1]]
        #dist_BA = distance(pA, pB) # pA posisi robot , pB posisi obstacle
        theta_BA = agent_heading
        theta_ort_left = theta_BA+obs_angle[i][1]+clearence_angle_left
        bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
        theta_ort_right = theta_BA+obs_angle[i][0]-clearence_angle_right
        bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
        RVO_BA = [transl_vB_vA, bound_left, bound_right, obs_distance[i][1]-clearence, ROB_RAD+1.5, "obstacle"]
        #print "unknown"
        #print theta_BA*180/numpy.pi
        #print obs_angle[i][0]*180/numpy.pi , obs_angle[i][1]*180/numpy.pi
        #print theta_ort_left*180/numpy.pi , theta_ort_right*180/numpy.pi
        #print RVO_BA
        RVO_BA_all.append(RVO_BA)
    
    hole_idx = 0
    for hole in Parameter.ws_model['circular_obstacles']:
        # hole = [x, y, rad]
        vB = [0, 0]
        pB = hole[0:2] # posisi obstacle
        transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
        #transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
        dist_BA = distance(pA, pB)
        theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
        # over-approximation of square to circular
        OVER_APPROX_C2S = 1.2
        rad = 1.5 #hole[2]*OVER_APPROX_C2S
        if (rad+ROB_RAD) > dist_BA:
            #print 'agent-',i,' collide with obstacle'
            dist_BA = rad+ROB_RAD
        theta_BAort = asin((rad+ROB_RAD)/dist_BA)
        theta_ort_left = theta_BA+theta_BAort
        bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
        theta_ort_right = theta_BA-theta_BAort
        bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
        #dist_dif = distance([0.5*(vB[0]-vA[0]),0.5*(vB[1]-vA[1])],[0,0])
        #transl_vB_vA = [pA[0]+vB[0]+cos(theta_ort_left)*dist_dif, pA[1]+vB[1]+sin(theta_ort_left)*dist_dif]
        RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad+ROB_RAD, "obstacle"]
        #print "known"
        #print theta_ort_left*180/numpy.pi , theta_ort_right*180/numpy.pi
        #print RVO_BA
        #RVO_BA_all.append(RVO_BA)
        hole_idx+=1    
    
    vA_post = intersect(pA, V_des, RVO_BA_all)
    V_opt = vA_post[:]
    return V_opt


def RVO_update_single_static(X_agent, X_neighbor, V_agent, V_neighbor, V_des, Parameter):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""

    ROB_RAD = Parameter.ws_model['robot_radius']+Parameter.ws_model['robot_clearence']# radius robot ditambah 0.02 clearence

    vA = [V_agent[0], V_agent[1]] # kecepatan dari agent ke i
    pA = [X_agent[0], X_agent[1]] # posisi dari agen ke-i
    RVO_BA_all = []
    for j in range(len(X_neighbor)): # loop untuk seluruh agent kecuali i
            vB = [V_neighbor[j][0], V_neighbor[j][1]] # kecepatan agent j
            pB = [X_neighbor[j][0], X_neighbor[j][1]] # posisi agen j
            # use RVO
            if(Parameter.ws_model['method'] == 'RVO'):
                transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])] # titik dimana segitiga RVO dimulai
            elif(Parameter.ws_model['method'] == 'HRVO'):
                transl_vB_vA2 = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])] # titik dimana segitiga RVO dimulai
            # use VO
            if(Parameter.ws_model['method'] == 'VO'):
                transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
            elif(Parameter.ws_model['method'] == 'HRVO'):
                transl_vB_vA1 = [pA[0]+vB[0], pA[1]+vB[1]]
            dist_BA = distance(pA, pB) # jarak antara agent i dan agent j
            theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0]) # sudut antara agen i dan agent j
            if 2*ROB_RAD > dist_BA: # jika jarak antara agent i dan agent j kurang dari dua kali ROB_RAD maka jaraknya dibuat 2 kali ROB_RAD
                #print 'robot collide'
                dist_BA = 2*ROB_RAD # untuk membuat 2*ROB_RAD/dist_BA selalu <= 1
            if (2* Parameter.ws_model['robot_radius']) > dist_BA:
                print 'agent collide with agent-',j
            theta_BAort = asin(2*ROB_RAD/dist_BA) # sudut tambahan untuk RVO
            theta_ort_left = theta_BA+theta_BAort # sudut RVO sebelah kiri
            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            theta_ort_right = theta_BA-theta_BAort # sudut RVO sebelah kanan
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
            # use HRVO
            if(Parameter.ws_model['method'] == 'HRVO'):
                #dist_dif = distance([0.5*(vB[0]-vA[0]),0.5*(vB[1]-vA[1])],[0,0])
                #transl_vB_vA = [pA[0]+vB[0]+cos(theta_ort_left)*dist_dif, pA[1]+vB[1]+sin(theta_ort_left)*dist_dif]
                Ax = transl_vB_vA2[0]
                Ay = transl_vB_vA2[1]
                Bx = transl_vB_vA2[0]+cos(theta_BA)
                By = transl_vB_vA2[1]+sin(theta_BA)
                XX = vA[0]+pA[0]
                YY = vA[1]+pA[1]
                position = sign((Bx - Ax) * (YY - Ay) - (By - Ay) * (XX - Ax)) # kanan: negative
                if(position < 0):
                    #print "vA > CL"
                    #RVO kanan
                    x1 = transl_vB_vA2[0]
                    y1 = transl_vB_vA2[1]
                    x2 = x1 + bound_right[0]
                    y2 = y1 + bound_right[1]
                    #VO kiri
                    p1 = transl_vB_vA1[0]
                    q1 = transl_vB_vA1[1]
                    p2 = p1 + bound_left[0]
                    q2 = q1 + bound_left[1]
                    L1 = line([x1,y1], [x2,y2])
                    L2 = line([p1,q1], [p2,q2])
                
                    R = intersection(L1, L2)
                    if R:
                        #print "Intersection detected:", R
                        #print x,y
                        transl_vB_vA = [R[0],R[1]]
                    else:
                        #print "No single intersection point detected"
                        transl_vB_vA = transl_vB_vA2
                else:
                    #print "vA < CL"
                    #RVO kiri
                    x1 = transl_vB_vA2[0]
                    y1 = transl_vB_vA2[1]
                    x2 = x1 + bound_left[0]
                    y2 = y1 + bound_left[1]
                    #VO kanan
                    p1 = transl_vB_vA1[0]
                    q1 = transl_vB_vA1[1]
                    p2 = p1 + bound_right[0]
                    q2 = q1 + bound_right[1]
                    L1 = line([x1,y1], [x2,y2])
                    L2 = line([p1,q1], [p2,q2])
                
                    R = intersection(L1, L2)
                    if R:
                        #print "Intersection detected:", R
                        #print x,y
                        transl_vB_vA = [R[0],R[1]]
                    else:
                        #print "No single intersection point detected"
                        transl_vB_vA = transl_vB_vA2
            
            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD,'agent'] # daerah RVO_BA didapatkan
            RVO_BA_all.append(RVO_BA)                
    hole_idx = 0
    for hole in Parameter.ws_model['circular_obstacles']:
        # hole = [x, y, rad]
        vB = [0, 0]
        pB = [hole[0]*Parameter.RealScale,hole[1]*Parameter.RealScale] # posisi obstacle
        #print pA , pB
        transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
        #transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
        dist_BA = distance(pA, pB)
        theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
        # over-approximation of square to circular

        OVER_APPROX_C2S = 1.0
        rad = hole[2]*OVER_APPROX_C2S*Parameter.RealScale
        if (rad+ROB_RAD) > dist_BA:
            print 'agent almost collide with obstacle'
            dist_BA = rad+ROB_RAD
        theta_BAort = asin((rad+ROB_RAD)/dist_BA)
        theta_ort_left = theta_BA+theta_BAort
        bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
        theta_ort_right = theta_BA-theta_BAort
        bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
        #dist_dif = distance([0.5*(vB[0]-vA[0]),0.5*(vB[1]-vA[1])],[0,0])
        #transl_vB_vA = [pA[0]+vB[0]+cos(theta_ort_left)*dist_dif, pA[1]+vB[1]+sin(theta_ort_left)*dist_dif]
        RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad+ROB_RAD,'obstacle']
        RVO_BA_all.append(RVO_BA)
        hole_idx+=1

    vA_post = intersect(pA, V_des, RVO_BA_all,Parameter.V_MAX)
    V_opt = vA_post[:]
    return V_opt,RVO_BA_all

def RVO_update(X, V_des, V_current, ws_model, Parameter):
    """ compute best velocity given the desired velocity, current velocity and workspace model"""
    ROB_RAD = ws_model['robot_radius']+ws_model['robot_clearence'] # radius robot ditambah 0.02 clearence
    V_opt = list(V_current)    
    for i in range(len(X)): # loop untuk seluruh agent
        vA = [V_current[i][0], V_current[i][1]] # kecepatan dari agent ke i
        pA = [X[i][0], X[i][1]] # posisi dari agen ke-i
        RVO_BA_all = []
        for j in range(len(X)): # loop untuk seluruh agent kecuali i
            if i!=j:
                vB = [V_current[j][0], V_current[j][1]] # kecepatan agent j
                pB = [X[j][0], X[j][1]] # posisi agen j
                # use RVO
                if(ws_model['method'] == 'RVO'):
                    transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])] # titik dimana segitiga RVO dimulai
                elif(ws_model['method'] == 'HRVO'):
                    transl_vB_vA2 = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])] # titik dimana segitiga RVO dimulai
                # use VO
                if(ws_model['method'] == 'VO'):
                    transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
                elif(ws_model['method'] == 'HRVO'):
                    transl_vB_vA1 = [pA[0]+vB[0], pA[1]+vB[1]]
                dist_BA = distance(pA, pB) # jarak antara agent i dan agent j
                theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0]) # sudut antara agen i dan agent j
                if 2*ROB_RAD > dist_BA: # jika jarak antara agent i dan agent j kurang dari dua kali ROB_RAD maka jaraknya dibuat 2 kali ROB_RAD
                    #print 'robot collide'
                    dist_BA = 2*ROB_RAD # untuk membuat 2*ROB_RAD/dist_BA selalu <= 1
                if (2* ws_model['robot_radius']) > dist_BA:
                    print 'agent collide with agent-',j
                theta_BAort = asin(2*ROB_RAD/dist_BA) # sudut tambahan untuk RVO
                theta_ort_left = theta_BA+theta_BAort # sudut RVO sebelah kiri
                bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                theta_ort_right = theta_BA-theta_BAort # sudut RVO sebelah kanan
                bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                # use HRVO
                if(ws_model['method'] == 'HRVO'):
                    #dist_dif = distance([0.5*(vB[0]-vA[0]),0.5*(vB[1]-vA[1])],[0,0])
                    #transl_vB_vA = [pA[0]+vB[0]+cos(theta_ort_left)*dist_dif, pA[1]+vB[1]+sin(theta_ort_left)*dist_dif]
                    Ax = transl_vB_vA2[0]
                    Ay = transl_vB_vA2[1]
                    Bx = transl_vB_vA2[0]+cos(theta_BA)
                    By = transl_vB_vA2[1]+sin(theta_BA)
                    XX = vA[0]+pA[0]
                    YY = vA[1]+pA[1]
                    position = sign((Bx - Ax) * (YY - Ay) - (By - Ay) * (XX - Ax)) # kanan: negative
                    if(position < 0):
                        #print "vA > CL"
                        #RVO kanan
                        x1 = transl_vB_vA2[0]
                        y1 = transl_vB_vA2[1]
                        x2 = x1 + bound_right[0]
                        y2 = y1 + bound_right[1]
                        #VO kiri
                        p1 = transl_vB_vA1[0]
                        q1 = transl_vB_vA1[1]
                        p2 = p1 + bound_left[0]
                        q2 = q1 + bound_left[1]
                        L1 = line([x1,y1], [x2,y2])
                        L2 = line([p1,q1], [p2,q2])
                    
                        R = intersection(L1, L2)
                        if R:
                            #print "Intersection detected:", R
                            #print x,y
                            transl_vB_vA = [R[0],R[1]]
                        else:
                            #print "No single intersection point detected"
                            transl_vB_vA = transl_vB_vA2
                    else:
                        #print "vA < CL"
                        #RVO kiri
                        x1 = transl_vB_vA2[0]
                        y1 = transl_vB_vA2[1]
                        x2 = x1 + bound_left[0]
                        y2 = y1 + bound_left[1]
                        #VO kanan
                        p1 = transl_vB_vA1[0]
                        q1 = transl_vB_vA1[1]
                        p2 = p1 + bound_right[0]
                        q2 = q1 + bound_right[1]
                        L1 = line([x1,y1], [x2,y2])
                        L2 = line([p1,q1], [p2,q2])
                    
                        R = intersection(L1, L2)
                        if R:
                            #print "Intersection detected:", R
                            #print x,y
                            transl_vB_vA = [R[0],R[1]]
                        else:
                            #print "No single intersection point detected"
                            transl_vB_vA = transl_vB_vA2
                
                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD] # daerah RVO_BA didapatkan
                RVO_BA_all.append(RVO_BA)                
        hole_idx = 0
        for hole in ws_model['circular_obstacles']:
            # hole = [x, y, rad]
            vB = [0, 0]
            pB = hole[0:2] # posisi obstacle
            transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
            #transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
            dist_BA = distance(pA, pB)
            theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
            # over-approximation of square to circular
            if(ws_model['type_obstacle'][hole_idx] == 's'):
                OVER_APPROX_C2S = 1.2
            else:
                OVER_APPROX_C2S = 1.2
            rad = hole[2]*OVER_APPROX_C2S
            if (rad+ROB_RAD) > dist_BA:
                print 'agent collide with obstacle'
                dist_BA = rad+ROB_RAD
            theta_BAort = asin((rad+ROB_RAD)/dist_BA)
            theta_ort_left = theta_BA+theta_BAort
            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            theta_ort_right = theta_BA-theta_BAort
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
            #dist_dif = distance([0.5*(vB[0]-vA[0]),0.5*(vB[1]-vA[1])],[0,0])
            #transl_vB_vA = [pA[0]+vB[0]+cos(theta_ort_left)*dist_dif, pA[1]+vB[1]+sin(theta_ort_left)*dist_dif]
            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad+ROB_RAD]
            RVO_BA_all.append(RVO_BA)
            hole_idx+=1

        vA_post = intersect(pA, V_des[i], RVO_BA_all)
        V_opt[i] = vA_post[:]
    return V_opt

def intersect(pA, vA, RVO_BA_all,V_MAX): # vA adalah V menuju goal
    # print '----------------------------------------'
    # print 'Start intersection test'
    norm_v = V_MAX #distance(vA, [0, 0]) # normalisasi kecepatan agent menuju goal
    suitable_V = []
    unsuitable_V = []
    for theta in numpy.arange(0, 2*PI, 0.1): # untuk sudut theta dari 0 hingga 2pi dengan sampling 0.1
        try:
            for rad in numpy.arange(0.0, norm_v+0.02, norm_v/20.0): # untuk radius dari 0.02 hingga normalisasi kecepatan agent+0.02 dengan sampling norm/5
                new_v = [rad*cos(theta), rad*sin(theta)] # diusulkan suatu kecepatan baru
                suit = True # awalnya kecepatan tsb suitable
                for RVO_BA in RVO_BA_all: # untuk RVO dalam list RVO
                    p_0 = RVO_BA[0] # posisi RVO
                    left = RVO_BA[1] # batas sudut kiri RVO
                    right = RVO_BA[2] # batas sudut kanan RVO
                    dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]] # 
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    #if((pA[0] < 0.05) and (vA[0] < 0)):
                    #    suit = False
                    #    break
                    if in_between(theta_right, theta_dif, theta_left):
                        suit = False
                        break
                if suit:
                    suitable_V.append(new_v)
                else:
                    if (True):#not ((pA[0] < 0.05) and (vA[0] < 0))):
                        unsuitable_V.append(new_v)
        except:
            print 'error in arange'
            print vA
            print norm_v
            print norm_v/5.0                
    new_v = vA[:]
    suit = True
    for RVO_BA in RVO_BA_all:                
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
        theta_dif = atan2(dif[1], dif[0])
        theta_right = atan2(right[1], right[0])
        theta_left = atan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False # jika sudut dif ada diantara sudut batas kanan dan kiri RVO, maka V yang diusulkan tidak suitable
            break
    if suit:
        suitable_V.append(new_v)
    else:
        if (True):#(not ((pA[0] < 0.05) and (vA[0] < 0))):
            unsuitable_V.append(new_v)
    
    #----------------------        
    if suitable_V: # jika ada yang suitable
        # print 'Suitable found'
        vA_post = min(suitable_V, key = lambda v: distance(v, vA)) # dipilih suitable V yang paling dekat dengan vA (V menuju goal)
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
    else: # jika tidak ada yg suitable
        # print 'Suitable not found'
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                if(RVO_BA[5] == "agent"):
                    p_0 = RVO_BA[0] # titik tengah
                    left = RVO_BA[1] # vektor batas kiri
                    right = RVO_BA[2] # vektor batas kanan
                    dist = RVO_BA[3] # jarak agent dengan obstacle
                    rad = RVO_BA[4] # radius obstacle
                    dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if in_between(theta_right, theta_dif, theta_left):
                        dist_tg = dist-rad
                        if dist_tg < 0: # collide
                            dist_tg = 0                    
                        tc_v = dist_tg/distance(dif, [0,0]) # jarak / kecepatan
                        tc.append(tc_v)
                else:
                    p_0 = RVO_BA[0] # titik tengah
                    left = RVO_BA[1] # vektor batas kiri
                    right = RVO_BA[2] # vektor batas kanan
                    dist = RVO_BA[3] # jarak agent dengan obstacle
                    rad = RVO_BA[4] # radius obstacle
                    dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if in_between(theta_right, theta_dif, theta_left):  
                        dist_tg = dist-rad
                        if dist_tg < 0: # collide
                            dist_tg = 0      
                        tc_v = dist/distance(dif, [0,0]) # jarak / kecepatan
                        tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 5.0 #5.0 #0.2 #5.0 # 0.5 #0.5 #0.2
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+distance(v, vA))) # dipilih V pada unsuitable V yang memiliki nilai penalti paling kecil
    return vA_post 

def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False

def compute_V_des_single(X,goal,V_max):
    kprop = 0.3
    dif_x = [kprop*(goal[k]-X[k]) for k in xrange(2)] # jarak antara posisi agen sekarang dengan posisi goal dalam koordinat [x,y]
    
    V_des=dif_x
    normv = numpy.linalg.norm(V_des) 
    if(normv > V_max):
        G = V_des[0]/V_des[1] 
        if(V_des[1] < 0):
            V_des[1] = -V_max/(numpy.sqrt(G*G+1))
        else:
            V_des[1] = V_max/(numpy.sqrt(G*G+1))
        V_des[0] = G*V_des[1]

    return V_des

def compute_V_des_single_PD(X,goal,X_dot,V_max):
    kprop = 0.6
    kderiv = 2
    dif_x = [kprop*(goal[k]-X[k])-kderiv*X_dot[k] for k in xrange(2)] # jarak antara posisi agen sekarang dengan posisi goal dalam koordinat [x,y]
    
    V_des=dif_x
    normv = numpy.linalg.norm(V_des) 
    if(normv > V_max):
        G = V_des[0]/V_des[1] 
        if(V_des[1] < 0):
            V_des[1] = -V_max/(numpy.sqrt(G*G+1))
        else:
            V_des[1] = V_max/(numpy.sqrt(G*G+1))
        V_des[0] = G*V_des[1]

    return V_des

def compute_V_des(X, goal, V_max):

    V_des = []
    '''
    for i in xrange(len(X)): # loop untuk jumlah agent
        dif_x = [goal[i][k]-X[i][k] for k in xrange(2)] # jarak antara posisi agen sekarang dengan posisi goal dalam koordinat [x,y]
        norm = distance(dif_x, [0, 0]) # jarak antara posisi agen sekarang dengan posisi goal dalam satuan grid
        norm_dif_x = [dif_x[k]*V_max[k]/norm for k in xrange(2)] # semakin jauh jarak, V semakin besar
        V_des.append(norm_dif_x[:]) # kumpulkan seluruh V dalam array
        if reach(X[i], goal[i], 0.02): # jika jarak antara agen dan goal kurang dari 0.5, maka kecepatan dibuat nol (diam)
            V_des[i][0] = 0
            V_des[i][1] = 0
    return V_des
    '''
    '''
    V_des = []
    for i in xrange(len(X)): # loop untuk jumlah agent
        dif_x = [goal[i][k]-X[i][k] for k in xrange(2)] # jarak antara posisi agen sekarang dengan posisi goal dalam koordinat [x,y]
        norm = distance(dif_x, [0, 0]) # jarak antara posisi agen sekarang dengan posisi goal dalam satuan grid
        if(norm >=1):
            norm_dif_x = [dif_x[k]*V_max[k]/norm for k in xrange(2)] # semakin jauh jarak, V semakin besar
        else:
            norm_dif_x = [dif_x[k]*V_max[k] for k in xrange(2)]
        norm_dif_x = [dif_x[k]*V_max[k] for k in xrange(2)]
        V_des.append(norm_dif_x[:]) # kumpulkan seluruh V dalam array
        #if reach(X[i], goal[i], 0.02): # jika jarak antara agen dan goal kurang dari 0.5, maka kecepatan dibuat nol (diam)
        #    V_des[i][0] = 0
        #    V_des[i][1] = 0
    '''
    '''
    listVx = []
    listVy = []
    for i in xrange(len(X)): # loop untuk jumlah agent
        dif_x = [goal[i][k]-X[i][k] for k in xrange(2)] # jarak antara posisi agen sekarang dengan posisi goal dalam koordinat [x,y]
        norm = distance(dif_x, [0, 0]) # jarak antara posisi agen sekarang dengan posisi goal dalam satuan grid
        norm_dif_x = [norm for k in xrange(2)]
        V_des.append(norm_dif_x[:]) # kumpulkan seluruh V dalam array
        kprop = 1#V_max[i]
        if(norm >=1):
            V_des[i][0] = kprop*dif_x[0]/norm
            V_des[i][1] = kprop*dif_x[1]/norm
        else:
            V_des[i][0] = kprop*dif_x[0]
            V_des[i][1] = kprop*dif_x[1]
        listVx.append(V_des[i][0])
        listVy.append(V_des[i][1])
    '''
    '''
        vmax = V_max[i]
        if(V_des[i][0] > vmax):
            V_des[i][0] = vmax
        elif(V_des[i][0] < -vmax):
            V_des[i][0] = -vmax
        
        if(V_des[i][1] > vmax):
            V_des[i][1] = vmax
        elif(V_des[i][1] < -vmax):
            V_des[i][1] = -vmax
    '''
        #if reach(X[i], goal[i], 0.02): # jika jarak antara agen dan goal kurang dari 0.5, maka kecepatan dibuat nol (diam)
        #    V_des[i][0] = 0
        #    V_des[i][1] = 0
    
    #listVx = []
    #listVy = []
    for i in xrange(len(X)): # loop untuk jumlah agent
        kprop = 3.0
        dif_x = [kprop*(goal[i][k]-X[i][k]) for k in xrange(2)] # jarak antara posisi agen sekarang dengan posisi goal dalam koordinat [x,y]
        
        V_des.append(dif_x[:])
        normv = numpy.linalg.norm(V_des[i]) 
        if(normv > V_max[i]):
            G = V_des[i][0]/V_des[i][1] 
            if(V_des[i][1] < 0):
                V_des[i][1] = -V_max[i]/(numpy.sqrt(G*G+1))
            else:
                V_des[i][1] = V_max[i]/(numpy.sqrt(G*G+1))
            V_des[i][0] = G*V_des[i][1]

        #listVx.append(V_des[i][0])
        #listVy.append(V_des[i][1])

    
    '''
        vmax = V_max[i]
        if(V_des[i][0] > vmax):
            V_des[i][0] = vmax
        elif(V_des[i][0] < -vmax):
            V_des[i][0] = -vmax
        
        if(V_des[i][1] > vmax):
            V_des[i][1] = vmax
        elif(V_des[i][1] < -vmax):
            V_des[i][1] = -vmax 
    '''
    #print 'v:',numpy.max(listVx),numpy.max(listVy)
    return V_des
            
def reach(p1, p2, bound=0.5):
    if distance(p1,p2)< bound:
        return True
    else:
        return False
    
    

# Author of the Code: Francesco Rose
# Master Thesis Project, Universitat Politecnica de la Catalunya
# Dynamic Interface AirSim/XML Fight Plan

import airsim

import pprint
import time
from math import *

import xlsxwriter

import xml.etree.ElementTree as ET

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import openpyxl

from geographiclib import geodesic


# --------------------------- Creation of the Functions ---------------------------- #

# Get_Data_to_Plot_drone's_track Function

def get_data(state):
    global column
    column += 1
    n_c = state.kinematics_estimated.position.x_val
    e_c = state.kinematics_estimated.position.y_val
    h = state.kinematics_estimated.position.z_val
    worksheet.write(0, column, n_c)
    worksheet.write(1, column, e_c)
    worksheet.write(2, column, h)


# Check_Position Function

def check_position(list1):
    state = client.getMultirotorState()
    p_attuale = [state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val,
                 state.kinematics_estimated.position.z_val]
    distance = sqrt((list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (list1[2] - p_attuale[2]) ** 2)
    time_required = (distance / list1[3])
    Time_For_Each_Leg.append(time_required)
    if LOG == 'ON':
        a = True
        while a:
            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[1] < (list1[1] + 2) and \
                    (list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                a = False
            else:
                get_data(state)
                time.sleep(0.1)
                state = client.getMultirotorState()
                p_attuale = [state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val,
                             state.kinematics_estimated.position.z_val]
    else:
        time.sleep(time_required*0.9)
        a = True
        while a:
            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[1] < (list1[1] + 2) and \
                    (list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                a = False
            else:
                time.sleep(0.1)
                state = client.getMultirotorState()
                p_attuale = [state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val,
                             state.kinematics_estimated.position.z_val]


# To_Fix_Leg Function

def tf_leg(n_coord, e_coord, alt, speed):
    return client.moveToPositionAsync(n_coord, e_coord, alt, speed)


# Scan_Leg Function

def scan_leg(ts, f_p_n, f_p_e, v, h, i):
    client.moveToPositionAsync(f_p_n, f_p_e, h, v)
    list = [f_p_n, f_p_e, h, v]
    North_Coordinates2.append(list[0])
    East_Coordinates2.append(list[1])
    Altitudes2.append(list[2])
    Speeds.append(list[3])
    check_position(list)
    if (f_p_e < 0 < f_p_n) or (f_p_n > 0 and f_p_e == 0):
        if i % 2 == 0:
            p = []
            s = []
            t = []
            q = []
            for num in range(1, i, 2):
                sott1 = num * ts
                a = f_p_n - sott1
                P = (a, f_p_e, h, v)
                p.append(P)
                S = (a, f_p_e + ll, h, v)
                s.append(S)
            for mol in range(2, i+1, 2):
                sott2 = mol * ts
                b = f_p_n - sott2
                T = (b, f_p_e + ll, h, v)
                t.append(T)
                Q = (b, f_p_e, h, v)
                q.append(Q)
            Total = []
            for n in range(len(q)):
                total = [p[n], s[n], t[n], q[n]]
                Total.append(total)
            Totale = []
            for n in range(len(Total)):
                for a in range(4):
                    totale = Total[n][a]
                    Totale.append(totale)
            print(Totale)
            for n in range(len(Totale)):
                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3])
                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                North_Coordinates2.append(list1[0])
                East_Coordinates2.append(list1[1])
                Altitudes2.append(list1[2])
                Speeds.append(list1[3])
                check_position(list1)
        else:
            p = []
            s = []
            t = []
            q = []
            for num in range(1, i+1, 2):
                sott1 = num * ts
                a = f_p_n - sott1
                P = (a, f_p_e, h, v)
                p.append(P)
                S = (a, f_p_e + ll, h, v)
                s.append(S)
            for mol in range(2, i+2, 2):
                sott2 = mol * ts
                b = f_p_n - sott2
                T = (b, f_p_e + ll, h, v)
                t.append(T)
                Q = (b, f_p_e, h, v)
                q.append(Q)
            Total = []
            for n in range(len(s)):
                total = [p[n], s[n], t[n], q[n]]
                Total.append(total)
            Totale = []
            for n in range(len(Total)):
                for a in range(4):
                    totale = Total[n][a]
                    Totale.append(totale)
            print(Totale)
            limit = len(Totale)-2
            for n in range(limit):
                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3])
                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                North_Coordinates2.append(list1[0])
                East_Coordinates2.append(list1[1])
                Altitudes2.append(list1[2])
                Speeds.append(list1[3])
                check_position(list1)
    elif (f_p_n > 0 and f_p_e > 0) or (f_p_n == 0 and f_p_e > 0):
        if i % 2 == 0:
            p = []
            s = []
            t = []
            q = []
            for num in range(1, i, 2):
                sott1 = num * ts
                a = f_p_n - sott1
                P = (a, f_p_e, h, v)
                p.append(P)
                S = (a, f_p_e - ll, h, v)
                s.append(S)
            for mol in range(2, i+1, 2):
                sott2 = mol * ts
                b = f_p_n - sott2
                T = (b, f_p_e - ll, h, v)
                t.append(T)
                Q = (b, f_p_e, h, v)
                q.append(Q)
            Total = []
            for n in range(len(q)):
                total = [p[n], s[n], t[n], q[n]]
                Total.append(total)
            Totale = []
            for n in range(len(Total)):
                for a in range(4):
                    totale = Total[n][a]
                    Totale.append(totale)
            print(Totale)
            for n in range(len(Totale)):
                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3])
                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                North_Coordinates2.append(list1[0])
                East_Coordinates2.append(list1[1])
                Altitudes2.append(list1[2])
                Speeds.append(list1[3])
                check_position(list1)
        else:
            p = []
            s = []
            t = []
            q = []
            for num in range(1, i+1, 2):
                sott1 = num * ts
                a = f_p_n - sott1
                P = (a, f_p_e, h, v)
                p.append(P)
                S = (a, f_p_e - ll, h, v)
                s.append(S)
            for mol in range(2, i+2, 2):
                sott2 = mol * ts
                b = f_p_n - sott2
                T = (b, f_p_e - ll, h, v)
                t.append(T)
                Q = (b, f_p_e, h, v)
                q.append(Q)
            Total = []
            for n in range(len(s)):
                total = [p[n], s[n], t[n], q[n]]
                Total.append(total)
            Totale = []
            for n in range(len(Total)):
                for a in range(4):
                    totale = Total[n][a]
                    Totale.append(totale)
            print(Totale)
            limit = len(Totale) - 2
            for n in range(limit):
                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3])
                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                North_Coordinates2.append(list1[0])
                East_Coordinates2.append(list1[1])
                Altitudes2.append(list1[2])
                Speeds.append(list1[3])
                check_position(list1)
    elif (f_p_n < 0 < f_p_e) or (f_p_n < 0 and f_p_e == 0):
        if i % 2 == 0:
            p = []
            s = []
            t = []
            q = []
            for num in range(1, i, 2):
                sott1 = num * ts
                a = f_p_n + sott1
                P = (a, f_p_e, h, v)
                p.append(P)
                S = (a, f_p_e - ll, h, v)
                s.append(S)
            for mol in range(2, i+1, 2):
                sott2 = mol * ts
                b = f_p_n + sott2
                T = (b, f_p_e - ll, h, v)
                t.append(T)
                Q = (b, f_p_e, h, v)
                q.append(Q)
            Total = []
            for n in range(len(q)):
                total = [p[n], s[n], t[n], q[n]]
                Total.append(total)
            Totale = []
            for n in range(len(Total)):
                for a in range(4):
                    totale = Total[n][a]
                    Totale.append(totale)
            print(Totale)
            for n in range(len(Totale)):
                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3])
                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                North_Coordinates2.append(list1[0])
                East_Coordinates2.append(list1[1])
                Altitudes2.append(list1[2])
                Speeds.append(list1[3])
                check_position(list1)
        else:
            p = []
            s = []
            t = []
            q = []
            for num in range(1, i+1, 2):
                sott1 = num * ts
                a = f_p_n + sott1
                P = (a, f_p_e, h, v)
                p.append(P)
                S = (a, f_p_e - ll, h, v)
                s.append(S)
            for mol in range(2, i+2, 2):
                sott2 = mol * ts
                b = f_p_n + sott2
                T = (b, f_p_e - ll, h, v)
                t.append(T)
                Q = (b, f_p_e, h, v)
                q.append(Q)
            Total = []
            for n in range(len(s)):
                total = [p[n], s[n], t[n], q[n]]
                Total.append(total)
            Totale = []
            for n in range(len(Total)):
                for a in range(4):
                    totale = Total[n][a]
                    Totale.append(totale)
            limit = len(Totale) - 2
            for n in range(limit):
                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3])
                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                North_Coordinates2.append(list1[0])
                East_Coordinates2.append(list1[1])
                Altitudes2.append(list1[2])
                Speeds.append(list1[3])
                check_position(list1)
    elif (f_p_n < 0 and f_p_e < 0) or (f_p_n == 0 and f_p_e < 0):
        if i % 2 == 0:
            p = []
            s = []
            t = []
            q = []
            for num in range(1, i, 2):
                sott1 = num * ts
                a = f_p_n + sott1
                P = (a, f_p_e, h, v)
                p.append(P)
                S = (a, f_p_e + ll, h, v)
                s.append(S)
            for mol in range(2, i+1, 2):
                sott2 = mol * ts
                b = f_p_n + sott2
                T = (b, f_p_e + ll, h, v)
                t.append(T)
                Q = (b, f_p_e, h, v)
                q.append(Q)
            Total = []
            for n in range(len(q)):
                total = [p[n], s[n], t[n], q[n]]
                Total.append(total)
            Totale = []
            for n in range(len(Total)):
                for a in range(4):
                    totale = Total[n][a]
                    Totale.append(totale)
            print(Totale)
            for n in range(len(Totale)):
                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3])
                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                North_Coordinates2.append(list1[0])
                East_Coordinates2.append(list1[1])
                Altitudes2.append(list1[2])
                Speeds.append(list1[3])
                check_position(list1)
        else:
            p = []
            s = []
            t = []
            q = []
            for num in range(1, i+1, 2):
                sott1 = num * ts
                a = f_p_n + sott1
                P = (a, f_p_e, h, v)
                p.append(P)
                S = (a, f_p_e + ll, h, v)
                s.append(S)
            for mol in range(2, i+2, 2):
                sott2 = mol * ts
                b = f_p_n + sott2
                T = (b, f_p_e + ll, h, v)
                t.append(T)
                Q = (b, f_p_e, h, v)
                q.append(Q)
            Total = []
            for n in range(len(s)):
                total = [p[n], s[n], t[n], q[n]]
                Total.append(total)
            Totale = []
            for n in range(len(Total)):
                for a in range(4):
                    totale = Total[n][a]
                    Totale.append(totale)
            print(Totale)
            limit = len(Totale) - 2
            for n in range(limit):
                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3])
                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                North_Coordinates2.append(list1[0])
                East_Coordinates2.append(list1[1])
                Altitudes2.append(list1[2])
                Speeds.append(list1[3])
                check_position(list1)


# Iterative_Leg Function

def iterative_leg(name_point, num_of_iter, name_next_point):
    l = waypoints_data[name_point]["len_iter_path"]
    iter_path = []
    for n in range(l):
        iter_path.append(waypoints_data[name_point]["name_point_iter_path" + str(n)])
    rip = 0
    while rip < num_of_iter:
        for n in range(len(iter_path)):
            name_point = iter_path[n]
            if waypoints_data[name_point]["leg_type"] == "fp_TFLeg":
                n_coord = waypoints_data[name_point]["n_coord"]
                e_coord = waypoints_data[name_point]["e_coord"]
                alt = waypoints_data[name_point]["altitude"]
                speed = waypoints_data[name_point]["speed"]
                tf_leg(n_coord, e_coord, alt, speed)
                North_Coordinates2.append(n_coord)
                East_Coordinates2.append(e_coord)
                Altitudes2.append(alt)
                Speeds.append(speed)
                list = [n_coord, e_coord, alt, speed]
                check_position(list)
            if waypoints_data[name_point]["leg_type"] == "fp_Scan":
                ts = waypoints_data[name_point]["trackseparation"]
                f_p_n = waypoints_data[name_point]["point1_n"]
                f_p_e = waypoints_data[name_point]["point1_e"]
                v = waypoints_data[name_point]["speed"]
                h = waypoints_data[name_point]["altitude"]
                i = waypoints_data[name_point]["iteration"]
                scan_leg(ts, f_p_n, f_p_e, v, h, i)
            if waypoints_data[name_point]["leg_type"] == "fp_IterativeLeg":
                name_next_point = waypoints_data[name_point]["name_next_point"]
                num_of_iter = waypoints_data[name_point]["num_of_iter"]
                iterative_leg(name_point, num_of_iter, name_next_point)
            if waypoints_data[name_point]["leg_type"] == "fp_IntersectionLeg":
                limit = waypoints_data[name_point]["limit"]
                option = input("Intersection Leg: to select one of the possibilities, insert a "
                               "number included between 1 and " + str(limit) + ": ")
                option = int(option)
                intersection_leg(name_point, option)
        rip += 1
    n_coord = waypoints_data[name_next_point]["n_coord"]
    e_coord = waypoints_data[name_next_point]["e_coord"]
    h = waypoints_data[name_next_point]["altitude"]
    v = waypoints_data[name_next_point]["speed"]
    tf_leg(n_coord, e_coord, h, v)
    list1 = [n_coord, e_coord, h, v]
    North_Coordinates2.append(list1[0])
    East_Coordinates2.append(list1[1])
    Altitudes2.append(list1[2])
    Speeds.append(list1[3])
    check_position(list1)


# Intersection_Leg Function

def intersection_leg(name_points, option):
    name_point = waypoints_data[name_points]["name_point_possibility" + str(option-1)]
    if waypoints_data[name_point]["leg_type"] == "fp_TFLeg":
        n_coord = waypoints_data[name_point]["n_coord"]
        e_coord = waypoints_data[name_point]["e_coord"]
        alt = waypoints_data[name_point]["altitude"]
        speed = waypoints_data[name_point]["speed"]
        tf_leg(n_coord, e_coord, alt, speed)
        North_Coordinates2.append(n_coord)
        East_Coordinates2.append(e_coord)
        Altitudes2.append(alt)
        Speeds.append(speed)
        list = [n_coord, e_coord, alt, speed]
        check_position(list)
    if waypoints_data[name_point]["leg_type"] == "fp_Scan":
        ts = waypoints_data[name_point]["trackseparation"]
        f_p_n = waypoints_data[name_point]["point1_n"]
        f_p_e = waypoints_data[name_point]["point1_e"]
        v = waypoints_data[name_point]["speed"]
        h = waypoints_data[name_point]["altitude"]
        i = waypoints_data[name_point]["iteration"]
        scan_leg(ts, f_p_n, f_p_e, v, h, i)
    if waypoints_data[name_point]["leg_type"] == "fp_IterativeLeg":
        name_next_point = waypoints_data[name_point]["name_next_point"]
        num_of_iter = waypoints_data[name_point]["num_of_iter"]
        iterative_leg(name_point, num_of_iter, name_next_point)
    if waypoints_data[name_point]["leg_type"] == "fp_IntersectionLeg":
        limit = waypoints_data[name_point]["limit"]
        option = input("Intersection Leg: to select one of the possibilities, insert a "
                       "number included between 1 and " + str(limit) + ": ")
        option = int(option)
        intersection_leg(name_point, option)


# Leg Function

def leg(name_point):
    if waypoints_data[name_point]["leg_type"] == "fp_TFLeg":
        n_coord = waypoints_data[name_point]["n_coord"]
        e_coord = waypoints_data[name_point]["e_coord"]
        alt = waypoints_data[name_point]["altitude"]
        speed = waypoints_data[name_point]["speed"]
        tf_leg(n_coord, e_coord, alt, speed)
        North_Coordinates2.append(n_coord)
        East_Coordinates2.append(e_coord)
        Altitudes2.append(alt)
        Speeds.append(speed)
        list = [n_coord, e_coord, alt, speed]
        check_position(list)
    if waypoints_data[name_point]["leg_type"] == "fp_Scan":
        ts = waypoints_data[name_point]["trackseparation"]
        f_p_n = waypoints_data[name_point]["point1_n"]
        f_p_e = waypoints_data[name_point]["point1_e"]
        v = waypoints_data[name_point]["speed"]
        h = waypoints_data[name_point]["altitude"]
        i = waypoints_data[name_point]["iteration"]
        scan_leg(ts, f_p_n, f_p_e, v, h, i)
    if waypoints_data[name_point]["leg_type"] == "fp_IterativeLeg":
        name_next_point = waypoints_data[name_point]["name_next_point"]
        num_of_iter = waypoints_data[name_point]["num_of_iter"]
        iterative_leg(name_point, num_of_iter, name_next_point)
    if waypoints_data[name_point]["leg_type"] == "fp_IntersectionLeg":
        limit = waypoints_data[name_point]["limit"]
        option = input("Intersection Leg: to select one of the possibilities, insert a "
                       "number included between 1 and " + str(limit) + ": ")
        option = int(option)
        intersection_leg(name_point, option)


# GetFirstChild_of_a_Stage Function

def get_first_child_stage(name_stage):
    legs = []
    for stages in tree.iter(tag='stage'):
        if stages.attrib['id'] == name_stage:
            for components in stages.iter(tag='leg'):
                name_point = components.attrib['id']
                legs.append(name_point)
    possibilities = []
    for elem in tree.iter(tag='leg'):
        if elem.attrib['xsi_type'] == "fp_IntersectionLeg":
            for choices in elem.iter():
                possibilitie = choices.text
                possibilities = possibilitie.split()
    next = []
    iter_path = []
    for elem in tree.iter(tag='leg'):
        if elem.attrib['xsi_type'] == "fp_IterativeLeg":
            for components in elem.iter():
                if components.tag == "body":
                    iterative_path = components.text
                    iter_path = iterative_path.split()
                elif components.tag == "next":
                    name_next_point = components.text
                    next.append(name_next_point)
    matches1 = []
    for leg1 in legs:
        for possibility in possibilities:
            if leg1 == possibility:
                matches1.append(leg1)
    for n in range(len(matches1)):
        legs.remove(matches1[n])
    matches2 = []
    for leg2 in legs:
        for iter in iter_path:
            if leg2 == iter:
                matches2.append(leg2)
    for n in range(len(matches2)):
        legs.remove(matches2[n])
    matches3 = []
    for leg3 in legs:
        for nex in next:
            if leg3 == nex:
                matches3.append(leg3)
    for n in range(len(matches3)):
        legs.remove(matches3[n])
    first_child = legs
    return first_child


# Stage Function

def stage(name_stage):
    name_points = get_first_child_stage(name_stage)
    for n in range(len(name_points)):
        leg(name_points[n])
    t_stage = time.time() - t0
    Time.append(t_stage)
    sum = 0
    for n in range(len(Time_For_Each_Leg)):
        sum = sum + Time_For_Each_Leg[n]
    Theoretical_Time_to_Plot.append(sum)
    del Time_For_Each_Leg[:]


# ----------------------------- Reading and parsing XML File ----------------------------- #

FP = input("Which Flight Plan do you want to use? Please, select 0 for Lat/Long FP or 1 for NED FP: ")

if FP == '0':
    tree = ET.parse('DroneFlightPlan_Neighborhood_Final_Version_LongLat.xml')
elif FP == '1':
    tree = ET.parse('DroneFlightPlan_Neighborhood_Final_Version_meters.xml')

root = tree.getroot()
ET.tostring(root, encoding='utf8').decode('utf8')

LOG = input("To run the code with plots enter ON; to run the code without plots enter OFF: ")

# -------------- Creation of an empty excel file to store points of real path --------------#

data_collection = xlsxwriter.Workbook('Data.xlsx')
worksheet = data_collection.add_worksheet()
worksheet.write(0, 0, "North Coordinates")
worksheet.write(1, 0, "East Coordinates")
worksheet.write(2, 0, "Altitudes")

# --------------------------------- Connection to AirSim  -------------------------------- #

# Connection to AirSim simulator

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

x_home = state.kinematics_estimated.position.x_val
y_home = state.kinematics_estimated.position.y_val
z_home = state.kinematics_estimated.position.z_val

lat0 = state.gps_location.latitude
long0 = state.gps_location.longitude
h_geo0 = state.gps_location.altitude

arcWE = geodesic.Geodesic.WGS84.Inverse(lat0, long0-0.5, lat0, long0+0.5)
arcNS = geodesic.Geodesic.WGS84.Inverse(lat0-0.5, long0, lat0+0.5, long0)

lat_conv = 1 / arcNS['s12']
long_conv = 1 / arcWE['s12']

# ---------------- Creation of a dictionary with all waypoints informations ---------------#

waypoints_data = {}

if FP == '0':
    for legs in tree.iter(tag='leg'):
        if legs.attrib['xsi_type'] == "fp_TFLeg":
            name_point = legs.attrib['id']
            waypoints_data[name_point] = {}
            leg_type = legs.attrib['xsi_type']
            waypoints_data[name_point]["leg_type"] = leg_type
            for components in legs.iter():
                if components.tag == "latitude":
                    lat = components.text
                    lat = float(lat)
                    if lat == lat0:
                        n_coord = 0
                    else:
                        n_coord = (lat-lat0)/lat_conv
                    waypoints_data[name_point]["n_coord"] = n_coord
                elif components.tag == "longitude":
                    long = components.text
                    long = float(long)
                    if long == long0:
                        e_coord = 0
                    else:
                        e_coord = (long-long0)/long_conv
                    waypoints_data[name_point]["e_coord"] = e_coord
                elif components.tag == "altitude":
                    altitude = components.text
                    altitude = float(altitude)
                    alt = z_home - (altitude - h_geo0)
                    waypoints_data[name_point]["altitude"] = alt
                elif components.tag == "speed":
                    speed = components.text
                    speed = float(speed)
                    waypoints_data[name_point]["speed"] = speed
        if legs.attrib['xsi_type'] == "fp_Scan":
            name_point = legs.attrib['id']
            waypoints_data[name_point] = {}
            leg_type = legs.attrib['xsi_type']
            waypoints_data[name_point]["leg_type"] = leg_type
            for components in legs.iter():
                if components.tag == "trackseparation":
                    ts = components.text
                    ts = float(ts)
                    waypoints_data[name_point]["trackseparation"] = ts
                elif components.tag == "point1":
                    f_p = components.text
                    f_p = f_p.split()
                    lat = float(f_p[0])
                    if lat == lat0:
                        f_p_n = 0
                    else:
                        f_p_n = (lat - lat0) / lat_conv
                    long = float(f_p[1])
                    if long == long0:
                        f_p_e = 0
                    else:
                        f_p_e = (long - long0) / long_conv
                    waypoints_data[name_point]["point1_n"] = f_p_n
                    waypoints_data[name_point]["point1_e"] = f_p_e
                elif components.tag == "altitude":
                    h = components.text
                    h = float(h)
                    alt = z_home - (h - h_geo0)
                    waypoints_data[name_point]["altitude"] = alt
                elif components.tag == "speed":
                    v = components.text
                    v = float(v)
                    waypoints_data[name_point]["speed"] = v
            l_l = abs(f_p_n) + abs(f_p_e)
            ll = round(l_l)
            waypoints_data[name_point]["iteration"] = int(ll / ts)
        if legs.attrib['xsi_type'] == "fp_IterativeLeg":
            name_point = legs.attrib['id']
            waypoints_data[name_point] = {}
            leg_type = legs.attrib['xsi_type']
            waypoints_data[name_point]["leg_type"] = leg_type
            for components in legs.iter():
                if components.tag == "body":
                    iterative_path = components.text
                    iter_path = iterative_path.split()
                    len_iter_path = len(iter_path)
                    waypoints_data[name_point]["len_iter_path"] = len_iter_path
                    for n in range(len(iter_path)):
                        waypoints_data[name_point]["name_point_iter_path" + str(n)] = iter_path[n]
                elif components.tag == "next":
                    name_next_point = components.text
                    waypoints_data[name_point]["name_next_point"] = name_next_point
                elif components.tag == "upperBound":
                    num_of_iteration = components.text
                    num_of_iter = int(num_of_iteration)
                    waypoints_data[name_point]["num_of_iter"] = num_of_iter
        if legs.attrib['xsi_type'] == "fp_IntersectionLeg":
            name_point = legs.attrib['id']
            waypoints_data[name_point] = {}
            leg_type = legs.attrib['xsi_type']
            waypoints_data[name_point]["leg_type"] = leg_type
            for choices in legs.iter():
                possibilities = choices.text
                possibilities = possibilities.split()
                for n in range(len(possibilities)):
                    waypoints_data[name_point]["name_point_possibility" + str(n)] = possibilities[n]
                limit = len(possibilities)
                waypoints_data[name_point]["limit"] = limit
elif FP == '1':
    for legs in tree.iter(tag='leg'):
        if legs.attrib['xsi_type'] == "fp_TFLeg":
            name_point = legs.attrib['id']
            waypoints_data[name_point] = {}
            leg_type = legs.attrib['xsi_type']
            waypoints_data[name_point]["leg_type"] = leg_type
            for components in legs.iter():
                if components.tag == "north_coordinate":
                    n_c = components.text
                    n_coord = float(n_c)
                    waypoints_data[name_point]["n_coord"] = n_coord
                elif components.tag == "east_coordinate":
                    e_c = components.text
                    e_coord = float(e_c)
                    waypoints_data[name_point]["e_coord"] = e_coord
                elif components.tag == "altitude":
                    alt = components.text
                    alt = float(alt)
                    waypoints_data[name_point]["altitude"] = alt
                elif components.tag == "speed":
                    speed = components.text
                    speed = float(speed)
                    waypoints_data[name_point]["speed"] = speed
        if legs.attrib['xsi_type'] == "fp_Scan":
            name_point = legs.attrib['id']
            waypoints_data[name_point] = {}
            leg_type = legs.attrib['xsi_type']
            waypoints_data[name_point]["leg_type"] = leg_type
            for components in legs.iter():
                if components.tag == "trackseparation":
                    ts = components.text
                    ts = float(ts)
                    waypoints_data[name_point]["trackseparation"] = ts
                elif components.tag == "point1":
                    f_p = components.text
                    f_p = f_p.split()
                    f_p_n = float(f_p[0])
                    f_p_e = float(f_p[1])
                    waypoints_data[name_point]["point1_n"] = f_p_n
                    waypoints_data[name_point]["point1_e"] = f_p_e
                elif components.tag == "altitude":
                    h = components.text
                    h = float(h)
                    waypoints_data[name_point]["altitude"] = h
                elif components.tag == "speed":
                    v = components.text
                    v = float(v)
                    waypoints_data[name_point]["speed"] = v
            l_l = abs(f_p_n) + abs(f_p_e)
            ll = round(l_l)
            waypoints_data[name_point]["iteration"] = int(ll / ts)
        if legs.attrib['xsi_type'] == "fp_IterativeLeg":
            name_point = legs.attrib['id']
            waypoints_data[name_point] = {}
            leg_type = legs.attrib['xsi_type']
            waypoints_data[name_point]["leg_type"] = leg_type
            for components in legs.iter():
                if components.tag == "body":
                    iterative_path = components.text
                    iter_path = iterative_path.split()
                    len_iter_path = len(iter_path)
                    waypoints_data[name_point]["len_iter_path"] = len_iter_path
                    for n in range(len(iter_path)):
                        waypoints_data[name_point]["name_point_iter_path" + str(n)] = iter_path[n]
                elif components.tag == "next":
                    name_next_point = components.text
                    waypoints_data[name_point]["name_next_point"] = name_next_point
                elif components.tag == "upperBound":
                    num_of_iteration = components.text
                    num_of_iter = int(num_of_iteration)
                    waypoints_data[name_point]["num_of_iter"] = num_of_iter
        if legs.attrib['xsi_type'] == "fp_IntersectionLeg":
            name_point = legs.attrib['id']
            waypoints_data[name_point] = {}
            leg_type = legs.attrib['xsi_type']
            waypoints_data[name_point]["leg_type"] = leg_type
            for choices in legs.iter():
                possibilities = choices.text
                possibilities = possibilities.split()
                for n in range(len(possibilities)):
                    waypoints_data[name_point]["name_point_possibility" + str(n)] = possibilities[n]
                limit = len(possibilities)
                waypoints_data[name_point]["limit"] = limit

print(waypoints_data)

# ------------------------------------ Take Off Part -------------------------------------- #

client.takeoffAsync()
time.sleep(4)

t0 = time.time()

# ------------------------------ Execution of the Flight Plan ----------------------------- #

Time = []
North_Coordinates2 = [x_home]
East_Coordinates2 = [y_home]
Altitudes2 = [z_home]
Speeds = []
Time_For_Each_Leg = []
Theoretical_Time_to_Plot = []

column = 0

Stages = []
for element in root.iter(tag='stage'):
    Stages.append(element.attrib['id'])

for n in range(len(Stages)):
    stage(Stages[n])

# -------------------------------- Landing + Carlo Disarm  -------------------------------- #

client.moveByVelocityZAsync(0, 0, z_home, 2, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 0)).join()

client.armDisarm(False)
client.enableApiControl(False)

data_collection.close()

# ------------------------------- Position Plots Part ------------------------------------- #

# Real Path vs Theoretical Path Plot

if LOG == 'ON':
    data = openpyxl.load_workbook('Data.xlsx')
    sheet = data['Sheet1']

    estremo_sx = []
    estremo_dx = []
    estremo_cn = []

    all_columns = sheet.columns

    for tupla in all_columns:
        lista = list(tupla)
        listino = [str(lista[0]), str(lista[1]), str(lista[2])]
        sx = listino[0]
        cn = listino[1]
        dx = listino[2]
        if len(dx) == 18 and len(sx) == 18 and len(cn) == 18:
            e_dx = dx[15:17]
            estremo_dx.append(e_dx)
            e_cn = cn[15:17]
            estremo_cn.append(e_cn)
            e_sx = sx[15:17]
            estremo_sx.append(e_sx)
        elif len(dx) == 19 and len(sx) == 19 and len(cn) == 19:
            e_dx = dx[15:18]
            estremo_dx.append(e_dx)
            e_cn = cn[15:18]
            estremo_cn.append(e_cn)
            e_sx = sx[15:18]
            estremo_sx.append(e_sx)
        elif len(dx) == 20 and len(sx) == 20 and len(cn) == 20:
            e_dx = dx[15:19]
            estremo_dx.append(e_dx)
            e_cn = cn[15:19]
            estremo_cn.append(e_cn)
            e_sx = sx[15:19]
            estremo_sx.append(e_sx)

    estremo_sx.remove('A1')
    estremo_cn.remove('A2')
    estremo_dx.remove('A3')

    Tutti_Punti = []

    for n in range(len(estremo_sx)):
        a = estremo_sx[n]
        c = estremo_cn[n]
        b = estremo_dx[n]
        multiple_cells = sheet[a:b]
        for row in multiple_cells:
            for cell in row:
                list = cell.value
                Tutti_Punti.append(list)

    North_Coordinates1 = []
    East_Coordinates1 = []
    Altitudes1 = []
    l = len(Tutti_Punti)

    for n in range(0, l, 3):
        North_Coordinates1.append(Tutti_Punti[n])
        East_Coordinates1.append(Tutti_Punti[n+1])
        Altitudes1.append(-Tutti_Punti[n+2])

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    plt.gca().invert_yaxis()

    Altitudes2_abs = []
    for n in range(len(Altitudes2)):
        alt = - (Altitudes2[n])
        Altitudes2_abs.append(alt)

    ax.plot3D(North_Coordinates1, East_Coordinates1, Altitudes1, color='blue')
    ax.plot3D(North_Coordinates2, East_Coordinates2, Altitudes2_abs, color='red')
    ax.set_xlabel('North Coordinate [m]')
    ax.set_ylabel('East Coordinate [m]')
    ax.set_zlabel('Absolute Value of Altitude [m]')


    # Real and Theoretical Coordinates vs Time Plot

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    Tot = Time[len(Time)-1]

    iteraz1 = len(North_Coordinates1)
    iteraz2 = len(North_Coordinates2)
    iteraz3 = len(East_Coordinates1)
    iteraz4 = len(East_Coordinates2)
    iteraz5 = len(Altitudes1)
    iteraz6 = len(Altitudes2_abs)

    x1 = np.linspace(0.0, Tot, iteraz1)
    x2 = np.linspace(0.0, Tot, iteraz2)
    x3 = np.linspace(0.0, Tot, iteraz3)
    x4 = np.linspace(0.0, Tot, iteraz4)
    x5 = np.linspace(0.0, Tot, iteraz5)
    x6 = np.linspace(0.0, Tot, iteraz6)

    y1 = North_Coordinates1
    y2 = North_Coordinates2
    y3 = East_Coordinates1
    y4 = East_Coordinates2
    y5 = Altitudes1
    y6 = Altitudes2_abs

    plt.subplot(3, 1, 1)
    plt.plot(x1, y1, 'blue', x2, y2, 'red')

    plt.subplot(3, 1, 2)
    plt.plot(x3, y3, 'blue', x4, y4, 'red')

    plt.subplot(3, 1, 3)
    plt.plot(x5, y5, 'blue', x6, y6, 'red')
    ax.grid()

    # --------------------------------- Time Plots Part --------------------------------------- #

    Real_Time_to_Plot = []
    Real_Time_to_Plot.append(Time[0])
    for n in range(1, len(Time)):
        Real_Time_to_Plot.append(Time[n]-Time[n-1])
    Real_Time_to_Plot.append(Time[len(Time)-1])
    print(Real_Time_to_Plot)

    Total_Theorical_Time = 0
    for n in range(len(Theoretical_Time_to_Plot)):
        Total_Theorical_Time = Total_Theorical_Time + Theoretical_Time_to_Plot[n]
    Theoretical_Time_to_Plot.append(Total_Theorical_Time)
    print(Theoretical_Time_to_Plot)

    # First, Second, Third Stage Time Plot

    for n in range(len(Real_Time_to_Plot)-1):
        fig = plt.figure()
        ax = plt.axes(projection="3d")

        num_bars = 2
        x_pos = [1, 1]
        y_pos = [1, 5]
        z_pos = [0] * num_bars
        x_size = np.ones(num_bars)
        y_size = np.ones(num_bars)
        z_size = [Theoretical_Time_to_Plot[n], Real_Time_to_Plot[n]]

        ax.bar3d(x_pos[0], y_pos[0], z_pos[0], x_size[0], y_size[0], z_size[0], color='blue')
        ax.bar3d(x_pos[1], y_pos[1], z_pos[1], x_size[1], y_size[1], z_size[1], color='red')

        a = str(n+1)

        ax.set_zlabel('Time spent [s]')
        ax.set_title(a + '° Stage Duration')

    # Total Mission Time Plot

    fig = plt.figure()
    ax = plt.axes(projection="3d")

    l1 = len(Theoretical_Time_to_Plot)-1
    l2 = len(Real_Time_to_Plot)-1

    num_bars = 2
    x_pos = [1, 1]
    y_pos = [1, 5]
    z_pos = [0] * num_bars
    x_size = np.ones(num_bars)
    y_size = np.ones(num_bars)
    z_size = [Theoretical_Time_to_Plot[l1], Real_Time_to_Plot[l2]]

    ax.bar3d(x_pos[0], y_pos[0], z_pos[0], x_size[0], y_size[0], z_size[0], color='blue')
    ax.bar3d(x_pos[1], y_pos[1], z_pos[1], x_size[1], y_size[1], z_size[1], color='red')

    ax.set_zlabel('Time spent [s]')
    ax.set_title(' Total Mission Duration')

    plt.show()


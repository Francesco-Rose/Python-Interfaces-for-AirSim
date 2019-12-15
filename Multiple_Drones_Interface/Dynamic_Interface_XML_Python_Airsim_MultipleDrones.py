# Author of the Code: Francesco Rose
# Master Thesis Project, Universitat Politecnica de la Catalunya
# Dynamic Interface AirSim/XML Fight Plan

import airsim

import threading
import time
import concurrent.futures

from math import *

import xml.etree.ElementTree as ET

import numpy as np

from geographiclib import geodesic


# ---------------------------------------------------------------------------------------

client = airsim.MultirotorClient()
client.confirmConnection()

client.reset()

populations = input("How many drone do you want to use?: ")
population = int(populations)

for count in range(population):
    (client.enableApiControl(True, vehicle_name='Drone' + str(count + 1)))
    (client.armDisarm(True, vehicle_name='Drone' + str(count + 1)))

i = 1
h = -1

for count in range(population):
    client.moveToZAsync(h-i, 3, vehicle_name='Drone' + str(count + 1))
    i += 3
    time.sleep(4)


class MainProgram:
    def __init__(self):
        self._lock = threading.Lock()

    def execution(self, drone_number):

        self._lock.acquire()

        FP = input("Which Flight Plan do you want to use? Please, select 0 for Geographical FP or 1 for NED FP: ")

        if FP == '0':
            tree = ET.parse(
                'Drone' + str(drone_number + 1) + '_FlightPlan_Neighborhood_Final_Version_Geographical.xml')
        elif FP == '1':
            tree = ET.parse('Drone' + str(drone_number + 1) + '_FlightPlan_Neighborhood_Final_Version_meters.xml')

        root = tree.getroot()
        ET.tostring(root, encoding='utf8').decode('utf8')

        vehicle_name = 'Drone' + str(drone_number + 1)

        state = client.getMultirotorState(vehicle_name)

        global x_home, y_home, z_home

        x_home = state.kinematics_estimated.position.x_val
        y_home = state.kinematics_estimated.position.y_val
        z_home = state.kinematics_estimated.position.z_val

        lat0 = state.gps_location.latitude
        long0 = state.gps_location.longitude
        h_geo0 = state.gps_location.altitude

        arcWE = geodesic.Geodesic.WGS84.Inverse(lat0, long0 - 0.5, lat0, long0 + 0.5)
        arcNS = geodesic.Geodesic.WGS84.Inverse(lat0 - 0.5, long0, lat0 + 0.5, long0)

        lat_conv = 1 / arcNS['s12']
        long_conv = 1 / arcWE['s12']

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
                                n_coord = (lat - lat0) / lat_conv
                            waypoints_data[name_point]["n_coord"] = n_coord
                        elif components.tag == "longitude":
                            long = components.text
                            long = float(long)
                            if long == long0:
                                e_coord = 0
                            else:
                                e_coord = (long - long0) / long_conv
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
                    waypoints_data[name_point]["lenght_side"] = ll
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
                    waypoints_data[name_point]["lenght_side"] = ll
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

        self._lock.release()

        stages = []
        for element in root.iter(tag='stage'):
            stages.append(element.attrib['id'])

        legs = []
        for n in range(len(stages)):
            for stage in tree.iter(tag='stage'):
                if stage.attrib['id'] == stages[n]:
                    for components in stage.iter(tag='leg'):
                        name_point = components.attrib['id']
                        legs.append(name_point)
        possibilities = []
        for elem in tree.iter(tag='leg'):
            if elem.attrib['xsi_type'] == "fp_IntersectionLeg":
                for choices in elem.iter():
                    pox = choices.text
                    possibilities = pox.split()
        iter_path = []
        going = []
        for elem in tree.iter(tag='leg'):
            if elem.attrib['xsi_type'] == "fp_IterativeLeg":
                for components in elem.iter():
                    if components.tag == "body":
                        iterative_path = components.text
                        iter_path = iterative_path.split()
                    elif components.tag == "next":
                        name_next_point = components.text
                        going.append(name_next_point)
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
            for nex in going:
                if leg3 == nex:
                    matches3.append(leg3)
        for n in range(len(matches3)):
            legs.remove(matches3[n])
        first_child = legs

        for n in range(len(first_child)):
            if waypoints_data[first_child[n]]["leg_type"] == "fp_TFLeg":
                n_coord = waypoints_data[first_child[n]]["n_coord"]
                e_coord = waypoints_data[first_child[n]]["e_coord"]
                alt = waypoints_data[first_child[n]]["altitude"]
                speed = waypoints_data[first_child[n]]["speed"]

                client.moveToPositionAsync(n_coord, e_coord, alt, speed, vehicle_name=vehicle_name)

                list1 = [n_coord, e_coord, alt, speed]
                state = client.getMultirotorState(vehicle_name)
                p_attuale = [state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val,
                             state.kinematics_estimated.position.z_val]

                distance = sqrt(
                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                list1[2] - p_attuale[2]) ** 2)
                time_required = (distance / list1[3])

                time.sleep(time_required * 0.9)
                a = True
                while a:
                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[1] < (
                            list1[1] + 2) and (list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                        a = False
                    else:
                        time.sleep(0.1)

                        state = client.getMultirotorState(vehicle_name)
                        p_attuale = [state.kinematics_estimated.position.x_val,
                                     state.kinematics_estimated.position.y_val,
                                     state.kinematics_estimated.position.z_val]
            if waypoints_data[first_child[n]]["leg_type"] == "fp_Scan":
                ts = waypoints_data[first_child[n]]["trackseparation"]
                f_p_n = waypoints_data[first_child[n]]["point1_n"]
                f_p_e = waypoints_data[first_child[n]]["point1_e"]
                v = waypoints_data[first_child[n]]["speed"]
                h = waypoints_data[first_child[n]]["altitude"]
                i = waypoints_data[first_child[n]]["iteration"]
                ll = waypoints_data[first_child[n]]["lenght_side"]

                client.moveToPositionAsync(f_p_n, f_p_e, h, v, vehicle_name=vehicle_name)

                list1 = [f_p_n, f_p_e, h, v]
                state = client.getMultirotorState(vehicle_name)
                p_attuale = [state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val,
                             state.kinematics_estimated.position.z_val]

                distance = sqrt((list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                list1[2] - p_attuale[2]) ** 2)
                time_required = (distance / list1[3])

                time.sleep(time_required * 0.9)
                a = True
                while a:
                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[1] < (
                            list1[1] + 2) and (list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                        a = False
                    else:
                        time.sleep(0.1)

                        state = client.getMultirotorState(vehicle_name)
                        p_attuale = [state.kinematics_estimated.position.x_val,
                                     state.kinematics_estimated.position.y_val,
                                     state.kinematics_estimated.position.z_val]
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
                        for mol in range(2, i + 1, 2):
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
                            client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                       vehicle_name=vehicle_name)

                            list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
                    else:
                        p = []
                        s = []
                        t = []
                        q = []
                        for num in range(1, i + 1, 2):
                            sott1 = num * ts
                            a = f_p_n - sott1
                            P = (a, f_p_e, h, v)
                            p.append(P)
                            S = (a, f_p_e + ll, h, v)
                            s.append(S)
                        for mol in range(2, i + 2, 2):
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
                        limit = len(Totale) - 2
                        for n in range(limit):
                            client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                       vehicle_name=vehicle_name)

                            list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
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
                        for mol in range(2, i + 1, 2):
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
                            client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                       vehicle_name=vehicle_name)

                            list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
                    else:
                        p = []
                        s = []
                        t = []
                        q = []
                        for num in range(1, i + 1, 2):
                            sott1 = num * ts
                            a = f_p_n - sott1
                            P = (a, f_p_e, h, v)
                            p.append(P)
                            S = (a, f_p_e - ll, h, v)
                            s.append(S)
                        for mol in range(2, i + 2, 2):
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
                            client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                       vehicle_name=vehicle_name)

                            list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
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
                        for mol in range(2, i + 1, 2):
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
                            client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                       vehicle_name=vehicle_name)

                            list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
                    else:
                        p = []
                        s = []
                        t = []
                        q = []
                        for num in range(1, i + 1, 2):
                            sott1 = num * ts
                            a = f_p_n + sott1
                            P = (a, f_p_e, h, v)
                            p.append(P)
                            S = (a, f_p_e - ll, h, v)
                            s.append(S)
                        for mol in range(2, i + 2, 2):
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
                            client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                       vehicle_name=vehicle_name)

                            list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
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
                        for mol in range(2, i + 1, 2):
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
                            client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                       vehicle_name=vehicle_name)

                            list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
                    else:
                        p = []
                        s = []
                        t = []
                        q = []
                        for num in range(1, i + 1, 2):
                            sott1 = num * ts
                            a = f_p_n + sott1
                            P = (a, f_p_e, h, v)
                            p.append(P)
                            S = (a, f_p_e + ll, h, v)
                            s.append(S)
                        for mol in range(2, i + 2, 2):
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
                            client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                       vehicle_name=vehicle_name)

                            list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
            if waypoints_data[first_child[n]]["leg_type"] == "fp_IterativeLeg":
                name_next_point = waypoints_data[first_child[n]]["name_next_point"]
                num_of_iter = waypoints_data[first_child[n]]["num_of_iter"]
                name_point = first_child[n]
                lip = waypoints_data[name_point]["len_iter_path"]
                iter_path = []
                for n in range(lip):
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

                            client.moveToPositionAsync(n_coord, e_coord, alt, speed, vehicle_name=vehicle_name)

                            list1 = [n_coord, e_coord, alt, speed]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
                        if waypoints_data[name_point]["leg_type"] == "fp_Scan":
                            ts = waypoints_data[name_point]["trackseparation"]
                            f_p_n = waypoints_data[name_point]["point1_n"]
                            f_p_e = waypoints_data[name_point]["point1_e"]
                            v = waypoints_data[name_point]["speed"]
                            h = waypoints_data[name_point]["altitude"]
                            i = waypoints_data[name_point]["iteration"]
                            ll = waypoints_data[name_point]["lenght_side"]
                            client.moveToPositionAsync(f_p_n, f_p_e, h, v, vehicle_name=vehicle_name)

                            list1 = [f_p_n, f_p_e, h, v]
                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]

                            distance = sqrt(
                                (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                        list1[2] - p_attuale[2]) ** 2)
                            time_required = (distance / list1[3])

                            time.sleep(time_required * 0.9)
                            a = True
                            while a:
                                if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[
                                    1] < (
                                        list1[1] + 2) and (
                                        list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                    a = False
                                else:
                                    time.sleep(0.1)

                                    state = client.getMultirotorState(vehicle_name)
                                    p_attuale = [state.kinematics_estimated.position.x_val,
                                                 state.kinematics_estimated.position.y_val,
                                                 state.kinematics_estimated.position.z_val]
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
                                    for mol in range(2, i + 1, 2):
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
                                        client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2],
                                                                   Totale[n][3],
                                                                   vehicle_name=vehicle_name)

                                        list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]

                                        distance = sqrt(
                                            (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                                    list1[2] - p_attuale[2]) ** 2)
                                        time_required = (distance / list1[3])

                                        time.sleep(time_required * 0.9)
                                        a = True
                                        while a:
                                            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                                    p_attuale[
                                                        1] < (
                                                    list1[1] + 2) and (
                                                    list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                                a = False
                                            else:
                                                time.sleep(0.1)

                                                state = client.getMultirotorState(vehicle_name)
                                                p_attuale = [state.kinematics_estimated.position.x_val,
                                                             state.kinematics_estimated.position.y_val,
                                                             state.kinematics_estimated.position.z_val]
                                else:
                                    p = []
                                    s = []
                                    t = []
                                    q = []
                                    for num in range(1, i + 1, 2):
                                        sott1 = num * ts
                                        a = f_p_n - sott1
                                        P = (a, f_p_e, h, v)
                                        p.append(P)
                                        S = (a, f_p_e + ll, h, v)
                                        s.append(S)
                                    for mol in range(2, i + 2, 2):
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
                                    limit = len(Totale) - 2
                                    for n in range(limit):
                                        client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2],
                                                                   Totale[n][3],
                                                                   vehicle_name=vehicle_name)

                                        list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]

                                        distance = sqrt(
                                            (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                                    list1[2] - p_attuale[2]) ** 2)
                                        time_required = (distance / list1[3])

                                        time.sleep(time_required * 0.9)
                                        a = True
                                        while a:
                                            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                                    p_attuale[
                                                        1] < (
                                                    list1[1] + 2) and (
                                                    list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                                a = False
                                            else:
                                                time.sleep(0.1)

                                                state = client.getMultirotorState(vehicle_name)
                                                p_attuale = [state.kinematics_estimated.position.x_val,
                                                             state.kinematics_estimated.position.y_val,
                                                             state.kinematics_estimated.position.z_val]
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
                                    for mol in range(2, i + 1, 2):
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
                                        client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2],
                                                                   Totale[n][3],
                                                                   vehicle_name=vehicle_name)

                                        list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]

                                        distance = sqrt(
                                            (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                                    list1[2] - p_attuale[2]) ** 2)
                                        time_required = (distance / list1[3])

                                        time.sleep(time_required * 0.9)
                                        a = True
                                        while a:
                                            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                                    p_attuale[
                                                        1] < (
                                                    list1[1] + 2) and (
                                                    list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                                a = False
                                            else:
                                                time.sleep(0.1)

                                                state = client.getMultirotorState(vehicle_name)
                                                p_attuale = [state.kinematics_estimated.position.x_val,
                                                             state.kinematics_estimated.position.y_val,
                                                             state.kinematics_estimated.position.z_val]
                                else:
                                    p = []
                                    s = []
                                    t = []
                                    q = []
                                    for num in range(1, i + 1, 2):
                                        sott1 = num * ts
                                        a = f_p_n - sott1
                                        P = (a, f_p_e, h, v)
                                        p.append(P)
                                        S = (a, f_p_e - ll, h, v)
                                        s.append(S)
                                    for mol in range(2, i + 2, 2):
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
                                        client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2],
                                                                   Totale[n][3],
                                                                   vehicle_name=vehicle_name)

                                        list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]

                                        distance = sqrt(
                                            (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                                    list1[2] - p_attuale[2]) ** 2)
                                        time_required = (distance / list1[3])

                                        time.sleep(time_required * 0.9)
                                        a = True
                                        while a:
                                            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                                    p_attuale[
                                                        1] < (
                                                    list1[1] + 2) and (
                                                    list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                                a = False
                                            else:
                                                time.sleep(0.1)

                                                state = client.getMultirotorState(vehicle_name)
                                                p_attuale = [state.kinematics_estimated.position.x_val,
                                                             state.kinematics_estimated.position.y_val,
                                                             state.kinematics_estimated.position.z_val]
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
                                    for mol in range(2, i + 1, 2):
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
                                        client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2],
                                                                   Totale[n][3],
                                                                   vehicle_name=vehicle_name)

                                        list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]

                                        distance = sqrt(
                                            (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                                    list1[2] - p_attuale[2]) ** 2)
                                        time_required = (distance / list1[3])

                                        time.sleep(time_required * 0.9)
                                        a = True
                                        while a:
                                            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                                    p_attuale[
                                                        1] < (
                                                    list1[1] + 2) and (
                                                    list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                                a = False
                                            else:
                                                time.sleep(0.1)

                                                state = client.getMultirotorState(vehicle_name)
                                                p_attuale = [state.kinematics_estimated.position.x_val,
                                                             state.kinematics_estimated.position.y_val,
                                                             state.kinematics_estimated.position.z_val]
                                else:
                                    p = []
                                    s = []
                                    t = []
                                    q = []
                                    for num in range(1, i + 1, 2):
                                        sott1 = num * ts
                                        a = f_p_n + sott1
                                        P = (a, f_p_e, h, v)
                                        p.append(P)
                                        S = (a, f_p_e - ll, h, v)
                                        s.append(S)
                                    for mol in range(2, i + 2, 2):
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
                                        client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2],
                                                                   Totale[n][3],
                                                                   vehicle_name=vehicle_name)

                                        list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]

                                        distance = sqrt(
                                            (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                                    list1[2] - p_attuale[2]) ** 2)
                                        time_required = (distance / list1[3])

                                        time.sleep(time_required * 0.9)
                                        a = True
                                        while a:
                                            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                                    p_attuale[
                                                        1] < (
                                                    list1[1] + 2) and (
                                                    list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                                a = False
                                            else:
                                                time.sleep(0.1)

                                                state = client.getMultirotorState(vehicle_name)
                                                p_attuale = [state.kinematics_estimated.position.x_val,
                                                             state.kinematics_estimated.position.y_val,
                                                             state.kinematics_estimated.position.z_val]
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
                                    for mol in range(2, i + 1, 2):
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
                                        client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2],
                                                                   Totale[n][3],
                                                                   vehicle_name=vehicle_name)

                                        list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]

                                        distance = sqrt(
                                            (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                                    list1[2] - p_attuale[2]) ** 2)
                                        time_required = (distance / list1[3])

                                        time.sleep(time_required * 0.9)
                                        a = True
                                        while a:
                                            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                                    p_attuale[
                                                        1] < (
                                                    list1[1] + 2) and (
                                                    list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                                a = False
                                            else:
                                                time.sleep(0.1)

                                                state = client.getMultirotorState(vehicle_name)
                                                p_attuale = [state.kinematics_estimated.position.x_val,
                                                             state.kinematics_estimated.position.y_val,
                                                             state.kinematics_estimated.position.z_val]
                                else:
                                    p = []
                                    s = []
                                    t = []
                                    q = []
                                    for num in range(1, i + 1, 2):
                                        sott1 = num * ts
                                        a = f_p_n + sott1
                                        P = (a, f_p_e, h, v)
                                        p.append(P)
                                        S = (a, f_p_e + ll, h, v)
                                        s.append(S)
                                    for mol in range(2, i + 2, 2):
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
                                        client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2],
                                                                   Totale[n][3],
                                                                   vehicle_name=vehicle_name)

                                        list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]

                                        distance = sqrt(
                                            (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                                    list1[2] - p_attuale[2]) ** 2)
                                        time_required = (distance / list1[3])

                                        time.sleep(time_required * 0.9)
                                        a = True
                                        while a:
                                            if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                                    p_attuale[
                                                        1] < (
                                                    list1[1] + 2) and (
                                                    list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                                a = False
                                            else:
                                                time.sleep(0.1)

                                                state = client.getMultirotorState(vehicle_name)
                                                p_attuale = [state.kinematics_estimated.position.x_val,
                                                             state.kinematics_estimated.position.y_val,
                                                             state.kinematics_estimated.position.z_val]
                    rip += 1
                n_coord = waypoints_data[name_next_point]["n_coord"]
                e_coord = waypoints_data[name_next_point]["e_coord"]
                h = waypoints_data[name_next_point]["altitude"]
                v = waypoints_data[name_next_point]["speed"]

                client.moveToPositionAsync(n_coord, e_coord, h, v, vehicle_name=vehicle_name)

                list1 = [n_coord, e_coord, h, v]
                state = client.getMultirotorState(vehicle_name)
                p_attuale = [state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val,
                             state.kinematics_estimated.position.z_val]

                distance = sqrt(
                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                list1[2] - p_attuale[2]) ** 2)
                time_required = (distance / list1[3])

                time.sleep(time_required * 0.9)
                a = True
                while a:
                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[1] < (
                            list1[1] + 2) and (
                            list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                        a = False
                    else:
                        time.sleep(0.1)

                        state = client.getMultirotorState(vehicle_name)
                        p_attuale = [state.kinematics_estimated.position.x_val,
                                     state.kinematics_estimated.position.y_val,
                                     state.kinematics_estimated.position.z_val]
            if waypoints_data[first_child[n]]["leg_type"] == "fp_IntersectionLeg":
                limit = waypoints_data[first_child[n]]["limit"]
                option = input("Intersection Leg: to select one of the possibilities, insert a "
                               "number included between 1 and " + str(limit) + ": ")
                option = int(option)
                name_points = first_child[n]
                name_point = waypoints_data[name_points]["name_point_possibility" + str(option - 1)]
                if waypoints_data[name_point]["leg_type"] == "fp_TFLeg":
                    n_coord = waypoints_data[name_point]["n_coord"]
                    e_coord = waypoints_data[name_point]["e_coord"]
                    alt = waypoints_data[name_point]["altitude"]
                    speed = waypoints_data[name_point]["speed"]

                    client.moveToPositionAsync(n_coord, e_coord, alt, speed, vehicle_name=vehicle_name)

                    list1 = [n_coord, e_coord, alt, speed]
                    state = client.getMultirotorState(vehicle_name)
                    p_attuale = [state.kinematics_estimated.position.x_val,
                                 state.kinematics_estimated.position.y_val,
                                 state.kinematics_estimated.position.z_val]

                    distance = sqrt(
                        (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                    list1[2] - p_attuale[2]) ** 2)
                    time_required = (distance / list1[3])

                    time.sleep(time_required * 0.9)
                    a = True
                    while a:
                        if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[1] < (
                                list1[1] + 2) and (
                                list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                            a = False
                        else:
                            time.sleep(0.1)

                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]
                if waypoints_data[name_point]["leg_type"] == "fp_Scan":
                    ts = waypoints_data[name_point]["trackseparation"]
                    f_p_n = waypoints_data[name_point]["point1_n"]
                    f_p_e = waypoints_data[name_point]["point1_e"]
                    v = waypoints_data[name_point]["speed"]
                    h = waypoints_data[name_point]["altitude"]
                    i = waypoints_data[name_point]["iteration"]
                    ll = waypoints_data[name_point]["lenght_side"]
                    client.moveToPositionAsync(f_p_n, f_p_e, h, v, vehicle_name=vehicle_name)

                    list1 = [f_p_n, f_p_e, h, v]
                    state = client.getMultirotorState(vehicle_name)
                    p_attuale = [state.kinematics_estimated.position.x_val,
                                 state.kinematics_estimated.position.y_val,
                                 state.kinematics_estimated.position.z_val]

                    distance = sqrt(
                        (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                    list1[2] - p_attuale[2]) ** 2)
                    time_required = (distance / list1[3])

                    time.sleep(time_required * 0.9)
                    a = True
                    while a:
                        if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < p_attuale[1] < (
                                list1[1] + 2) and (
                                list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                            a = False
                        else:
                            time.sleep(0.1)

                            state = client.getMultirotorState(vehicle_name)
                            p_attuale = [state.kinematics_estimated.position.x_val,
                                         state.kinematics_estimated.position.y_val,
                                         state.kinematics_estimated.position.z_val]
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
                            for mol in range(2, i + 1, 2):
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
                                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                           vehicle_name=vehicle_name)

                                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                state = client.getMultirotorState(vehicle_name)
                                p_attuale = [state.kinematics_estimated.position.x_val,
                                             state.kinematics_estimated.position.y_val,
                                             state.kinematics_estimated.position.z_val]

                                distance = sqrt(
                                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                            list1[2] - p_attuale[2]) ** 2)
                                time_required = (distance / list1[3])

                                time.sleep(time_required * 0.9)
                                a = True
                                while a:
                                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                            p_attuale[
                                                1] < (
                                            list1[1] + 2) and (
                                            list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                        a = False
                                    else:
                                        time.sleep(0.1)

                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]
                        else:
                            p = []
                            s = []
                            t = []
                            q = []
                            for num in range(1, i + 1, 2):
                                sott1 = num * ts
                                a = f_p_n - sott1
                                P = (a, f_p_e, h, v)
                                p.append(P)
                                S = (a, f_p_e + ll, h, v)
                                s.append(S)
                            for mol in range(2, i + 2, 2):
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
                            limit = len(Totale) - 2
                            for n in range(limit):
                                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                           vehicle_name=vehicle_name)

                                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                state = client.getMultirotorState(vehicle_name)
                                p_attuale = [state.kinematics_estimated.position.x_val,
                                             state.kinematics_estimated.position.y_val,
                                             state.kinematics_estimated.position.z_val]

                                distance = sqrt(
                                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                            list1[2] - p_attuale[2]) ** 2)
                                time_required = (distance / list1[3])

                                time.sleep(time_required * 0.9)
                                a = True
                                while a:
                                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                            p_attuale[
                                                1] < (
                                            list1[1] + 2) and (
                                            list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                        a = False
                                    else:
                                        time.sleep(0.1)

                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]
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
                            for mol in range(2, i + 1, 2):
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
                                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                           vehicle_name=vehicle_name)

                                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                state = client.getMultirotorState(vehicle_name)
                                p_attuale = [state.kinematics_estimated.position.x_val,
                                             state.kinematics_estimated.position.y_val,
                                             state.kinematics_estimated.position.z_val]

                                distance = sqrt(
                                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                            list1[2] - p_attuale[2]) ** 2)
                                time_required = (distance / list1[3])

                                time.sleep(time_required * 0.9)
                                a = True
                                while a:
                                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                            p_attuale[
                                                1] < (
                                            list1[1] + 2) and (
                                            list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                        a = False
                                    else:
                                        time.sleep(0.1)

                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]
                        else:
                            p = []
                            s = []
                            t = []
                            q = []
                            for num in range(1, i + 1, 2):
                                sott1 = num * ts
                                a = f_p_n - sott1
                                P = (a, f_p_e, h, v)
                                p.append(P)
                                S = (a, f_p_e - ll, h, v)
                                s.append(S)
                            for mol in range(2, i + 2, 2):
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
                                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                           vehicle_name=vehicle_name)

                                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                state = client.getMultirotorState(vehicle_name)
                                p_attuale = [state.kinematics_estimated.position.x_val,
                                             state.kinematics_estimated.position.y_val,
                                             state.kinematics_estimated.position.z_val]

                                distance = sqrt(
                                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                            list1[2] - p_attuale[2]) ** 2)
                                time_required = (distance / list1[3])

                                time.sleep(time_required * 0.9)
                                a = True
                                while a:
                                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                            p_attuale[
                                                1] < (
                                            list1[1] + 2) and (
                                            list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                        a = False
                                    else:
                                        time.sleep(0.1)

                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]
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
                            for mol in range(2, i + 1, 2):
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
                                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                           vehicle_name=vehicle_name)

                                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                state = client.getMultirotorState(vehicle_name)
                                p_attuale = [state.kinematics_estimated.position.x_val,
                                             state.kinematics_estimated.position.y_val,
                                             state.kinematics_estimated.position.z_val]

                                distance = sqrt(
                                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                            list1[2] - p_attuale[2]) ** 2)
                                time_required = (distance / list1[3])

                                time.sleep(time_required * 0.9)
                                a = True
                                while a:
                                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                            p_attuale[
                                                1] < (
                                            list1[1] + 2) and (
                                            list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                        a = False
                                    else:
                                        time.sleep(0.1)

                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]
                        else:
                            p = []
                            s = []
                            t = []
                            q = []
                            for num in range(1, i + 1, 2):
                                sott1 = num * ts
                                a = f_p_n + sott1
                                P = (a, f_p_e, h, v)
                                p.append(P)
                                S = (a, f_p_e - ll, h, v)
                                s.append(S)
                            for mol in range(2, i + 2, 2):
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
                                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                           vehicle_name=vehicle_name)

                                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                state = client.getMultirotorState(vehicle_name)
                                p_attuale = [state.kinematics_estimated.position.x_val,
                                             state.kinematics_estimated.position.y_val,
                                             state.kinematics_estimated.position.z_val]

                                distance = sqrt(
                                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                            list1[2] - p_attuale[2]) ** 2)
                                time_required = (distance / list1[3])

                                time.sleep(time_required * 0.9)
                                a = True
                                while a:
                                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                            p_attuale[
                                                1] < (
                                            list1[1] + 2) and (
                                            list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                        a = False
                                    else:
                                        time.sleep(0.1)

                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]
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
                            for mol in range(2, i + 1, 2):
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
                                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                           vehicle_name=vehicle_name)

                                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                state = client.getMultirotorState(vehicle_name)
                                p_attuale = [state.kinematics_estimated.position.x_val,
                                             state.kinematics_estimated.position.y_val,
                                             state.kinematics_estimated.position.z_val]

                                distance = sqrt(
                                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                            list1[2] - p_attuale[2]) ** 2)
                                time_required = (distance / list1[3])

                                time.sleep(time_required * 0.9)
                                a = True
                                while a:
                                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                            p_attuale[
                                                1] < (
                                            list1[1] + 2) and (
                                            list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                        a = False
                                    else:
                                        time.sleep(0.1)

                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]
                        else:
                            p = []
                            s = []
                            t = []
                            q = []
                            for num in range(1, i + 1, 2):
                                sott1 = num * ts
                                a = f_p_n + sott1
                                P = (a, f_p_e, h, v)
                                p.append(P)
                                S = (a, f_p_e + ll, h, v)
                                s.append(S)
                            for mol in range(2, i + 2, 2):
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
                                client.moveToPositionAsync(Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3],
                                                           vehicle_name=vehicle_name)

                                list1 = [Totale[n][0], Totale[n][1], Totale[n][2], Totale[n][3]]
                                state = client.getMultirotorState(vehicle_name)
                                p_attuale = [state.kinematics_estimated.position.x_val,
                                             state.kinematics_estimated.position.y_val,
                                             state.kinematics_estimated.position.z_val]

                                distance = sqrt(
                                    (list1[0] - p_attuale[0]) ** 2 + (list1[1] - p_attuale[1]) ** 2 + (
                                            list1[2] - p_attuale[2]) ** 2)
                                time_required = (distance / list1[3])

                                time.sleep(time_required * 0.9)
                                a = True
                                while a:
                                    if (list1[0] - 2) < p_attuale[0] < (list1[0] + 2) and (list1[1] - 2) < \
                                            p_attuale[
                                                1] < (
                                            list1[1] + 2) and (
                                            list1[2] - 1) < p_attuale[2] < (list1[2] + 1):
                                        a = False
                                    else:
                                        time.sleep(0.1)

                                        state = client.getMultirotorState(vehicle_name)
                                        p_attuale = [state.kinematics_estimated.position.x_val,
                                                     state.kinematics_estimated.position.y_val,
                                                     state.kinematics_estimated.position.z_val]

    def landing(self, drone_number):

        with self._lock:

            vehicle_name = 'Drone' + str(drone_number + 1)

            client.moveToPositionAsync(x_home, y_home, -3, 3, vehicle_name=vehicle_name)

            client.moveByVelocityZAsync(x_home, y_home, z_home, 2, airsim.DrivetrainType.MaxDegreeOfFreedom,
                                        airsim.YawMode(False, 0),
                                        vehicle_name='Drone' + str(drone_number + 1)).join()

            client.armDisarm(False, vehicle_name='Drone' + str(drone_number + 1))

            client.enableApiControl(False, vehicle_name='Drone' + str(drone_number + 1))



if __name__ == "__main__":

    program = MainProgram()

    with concurrent.futures.ThreadPoolExecutor(max_workers=population) as executor:
        for index in range(population):
            executor.submit(program.execution, index)

    with concurrent.futures.ThreadPoolExecutor(max_workers=population) as executor:
        for index in range(population):
            executor.submit(program.landing, index)


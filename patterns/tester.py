#!/usr/bin/env

import numpy as np
from math import *
import time

import serial
import csv
serial_port = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_956353330313512012D0-if00', 9600)



def Send(Mat):
    max = np.zeros(1)
    max[0] = 0
    for i in range(len(Mat)):
        max[0] = max[0] + np.amax(Mat[i][:,1])*100
    # print("time", max[0])
    serial_port = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_956353330313512012D0-if00', 9600)
    t =matrix_send(Mat)

    serial_port.close()
    t2=(max[0] / 1000.0)+t
    return t2


def matrix_send(Matr):
    X = np.zeros((5, 1, 2))
    X = (
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0])

    matrix = np.copy(Matr)

    for i in range(len(matrix)):
        Z = np.copy(matrix[i])
        for k in range(len(Z)):
            item = '%s\r' % Z[k][0]
            serial_port.write(item.encode())

            # print("raw1", Z[k][0])
        for n in range(len(Z)):
            item = '%s\r' % Z[n][1]
            serial_port.write(item.encode())
            # print("raw1", Z[n][1])


    for i in range (5- len(matrix)):

        Z = np.copy(X)
        for k in range(len(Z)):
            item = '%s\r' % Z[k][0]
            serial_port.write(item.encode())

            # print("raw1", Z[k][0])
        for n in range(len(Z)):
            item = '%s\r' % Z[n][1]
            serial_port.write(item.encode())
            # print("raw1", Z[n][1])
    return 0.1*(5- len(matrix))


duration = 4
max_duration = 6
duration_empty = 2

low_lev = 6
mid_lev = 7
high_lev = 9

empty = np.zeros((5, 1, 2))
empty = (
[0, duration_empty],
[0, duration_empty],
[0, duration_empty],
[0, duration_empty],
[0, duration_empty])

D = np.zeros((5, 1, 2)) #5,7,9
D = (
[0, duration],
[0, duration],
[low_lev, duration],
[0, duration],
[0, duration])
D1 = np.zeros((5, 1, 2))
D1 = (
[0, duration],
[mid_lev, duration],
[0, duration],
[mid_lev, duration],
[0, duration])
D2 = np.zeros((5, 1, 2))
D2 = (
[high_lev, max_duration],
[0, duration],
[0, duration],
[0, duration],
[high_lev-1, max_duration])


P1=[]
P1.append(np.copy(D))
P1.append(np.copy(empty))
P1.append(np.copy(D1))
P1.append(np.copy(empty))
P1.append(np.copy(D2))
# P2.append(np.copy(B))
#increasing distance (contracted)
P2=[]
P2.append(np.copy(D2))
P2.append(np.copy(empty))
P2.append(np.copy(D1))
P2.append(np.copy(empty))
P2.append(np.copy(D))





S = np.zeros((5, 1, 2)) #5,7,9
S = (
[0, duration],
[0, duration],
[mid_lev, duration],
[0, duration],
[0, duration])

S1 = np.zeros((5, 1, 2)) #5,7,9
S1 = (
[0, duration],
[mid_lev, duration],
[0, duration],
[mid_lev, duration],
[0, duration])

S2 = np.zeros((5, 1, 2)) #5,7,9
S2 = (
[mid_lev, duration],
[0, duration],
[0, duration],
[0, duration],
[mid_lev, duration])
#constant distance
P3=[]
P3.append(np.copy(S))
P3.append(np.copy(empty))
P3.append(np.copy(S1))
P3.append(np.copy(empty))
P3.append(np.copy(S2))

#constant distance (contracted state)

S_ = np.zeros((5, 1, 2)) #5,7,9
S_ = (
[0, duration],
[0, duration],
[mid_lev+1, duration],
[0, duration],
[0, duration])

P4=[]
P4.append(np.copy(S2))
P4.append(np.copy(empty))
P4.append(np.copy(S1))
P4.append(np.copy(empty))
P4.append(np.copy(S_))


G = np.zeros((5, 1, 2)) #5,7,9
G = (
[0, duration],
[0, duration],
[high_lev, max_duration],
[0, duration],
[0, duration])
G1 = np.zeros((5, 1, 2))
G1 = (
[0, duration],
[mid_lev, duration],
[0, duration],
[mid_lev, duration],
[0, duration])
G2 = np.zeros((5, 1, 2))
G2 = (
[low_lev+1, duration],
[0, duration],
[0, duration],
[0, duration],
[low_lev-1, duration])

#Decreasing distance (extended state)
P5=[]
P5.append(np.copy(G))
P5.append(np.copy(empty))
P5.append(np.copy(G1))
P5.append(np.copy(empty))
P5.append(np.copy(G2))

#decreasing distance (contracted state)

P6=[]
P6.append(np.copy(G2))
P6.append(np.copy(empty))
P6.append(np.copy(G1))
P6.append(np.copy(empty))
P6.append(np.copy(G))





F = np.zeros((5, 1, 2)) #5,7,9
F = (
[high_lev, duration],
[0, duration],
[0, duration],
[0, duration],
[0, duration])
F1 = np.zeros((5, 1, 2))
F1 = (
[0, duration],
[0, duration],
[high_lev, duration],
[0, duration],
[0, duration])
F2 = np.zeros((5, 1, 2))
F2 = (
[0, duration],
[0, duration],
[0, duration],
[0, duration],
[high_lev, duration])

F_ = np.zeros((5, 1, 2))
F_ = (
[0, duration],
[high_lev, duration],
[0, duration],
[0, duration],
[0, duration])
F__ = np.zeros((5, 1, 2))
F__ = (
[0, duration],
[0, duration],
[0, duration],
[high_lev, duration],
[0, duration])


P7=[]
P7.append(np.copy(F))
P7.append(np.copy(F_))
P7.append(np.copy(F1))
P7.append(np.copy(F__))
P7.append(np.copy(F2))


P8=[]
P8.append(np.copy(F2))
P8.append(np.copy(F__))
P8.append(np.copy(F1))
P8.append(np.copy(F_))
P8.append(np.copy(F))



#___________________________________________________________________________________________________
L = np.zeros((5, 1, 2)) #5,7,9
L = (
[high_lev, max_duration],
[0, max_duration],
[0, max_duration],
[0, max_duration],
[0, max_duration])

R = np.zeros((5, 1, 2)) #5,7,9
R = (
[0, max_duration],
[0, max_duration],
[0, max_duration],
[0, max_duration],
[high_lev, max_duration])

ML1 = np.zeros((5, 1, 2))
ML1 = (
[0, duration],
[mid_lev, duration],
[0, duration],
[0, duration],
[0, duration])
ML2 = np.zeros((5, 1, 2))
ML2 = (
[0, duration],
[0, duration],
[mid_lev, duration],
[0, duration],
[0, duration])
ML3 = np.zeros((5, 1, 2))
ML3 = (
[0, duration],
[0, duration],
[0, duration],
[mid_lev, duration],
[0, duration])

MR1 = np.zeros((5, 1, 2))
MR1 = (
[0, duration],
[0, duration],
[0, duration],
[mid_lev, duration],
[0, duration])
MR2 = np.zeros((5, 1, 2))
MR2 = (
[0, duration],
[0, duration],
[mid_lev, duration],
[0, duration],
[0, duration])
MR3 = np.zeros((5, 1, 2))
MR3 = (
[0, duration],
[mid_lev, duration],
[0, duration],
[0, duration],
[0, duration])

M1 = np.zeros((5, 1, 2))
M1 = (
[0, duration],
[mid_lev, duration],
[mid_lev, duration],
[mid_lev, duration],
[0, duration])
M2 = np.zeros((5, 1, 2))
M2 = (
[0, duration],
[mid_lev, duration],
[mid_lev, duration],
[mid_lev, duration],
[0, duration])
M3 = np.zeros((5, 1, 2))
M3 = (
[0, duration],
[mid_lev, duration],
[mid_lev, duration],
[mid_lev, duration],
[0, duration])
#________________________________________________________________________________________________________

#Decreasing distance (extended state)
P9=[]
P9.append(np.copy(R))
P10=[]
P10.append(np.copy(L))

P11=[]
P11.append(np.copy(MR1))
P11.append(np.copy(MR2))
P11.append(np.copy(MR3))
P12=[]
P12.append(np.copy(ML1))
P12.append(np.copy(ML2))
P12.append(np.copy(ML3))
P13=[]
P13.append(np.copy(M1))
P13.append(np.copy(M2))
P13.append(np.copy(M3))





def play_pattern(m):
    if(m==1):
        t = Send(P1)
        print("P1")
        time.sleep(t)
    if (m == 2):
        t = Send(P2)
        print("P2")
        time.sleep(t)
    if (m == 3):
        t = Send(P3)
        print("P3")
        time.sleep(t)
    if (m == 4):
        t = Send(P4)
        print("P4")
        time.sleep(t)
    if (m == 5):
        t = Send(P5)
        print("P5")
        time.sleep(t)
    if (m == 6):
        t = Send(P6)
        print("P6")
        time.sleep(t)
    if (m == 7):
        t = Send(P7)
        print("P7")
        time.sleep(t)
    if (m == 8):
        t = Send(P8)
        print("P8")
        time.sleep(t)

    if (m == 9):
        t = Send(P9)
        print("P9")
        time.sleep(t)
    if (m == 10):
        t = Send(P10)
        print("P10")
        time.sleep(t)
    if (m == 11):
        t = Send(P11)
        print("P11")
        time.sleep(t)
    if (m == 12):
        t = Send(P12)
        print("P12")
        time.sleep(t)
    if (m == 13):
        t = Send(P13)
        print("P13")
        time.sleep(t)

patterns = [1,2,3,4,5,6]
arr = np.zeros(6)



while(1):
    person = int(input('Pls, enter the pattern number: '))
    play_pattern(person)
print("Thank you for your feedback")
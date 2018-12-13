# ==================== Imports ====================
import numpy as np
import serial
from matplotlib import pyplot as plt
from matplotlib import animation
from time import sleep
import sys, time
import filtering as flt
import scipy
import math 
import hr_ecg as hr
import hr_ecg
import math
import time
#import write_to_pyrebase
#from pyrebase import pyrebase
#import write_to_pyrebase



# ==================== Globals ====================
N = 400                                         # samples to plot
NS = 20                                         # samples to grab each iteration
pause_interval = .5
sample_count = 0                                # current sample count
times, raw, processed, hr, hrProcessed= np.zeros((5, N))                         # data vectors
serial_port = '/dev/cu.usbserial'
#serial_port = '/dev/cu.usbmodem14201'  # the serial port to use
serial_baud = 9600
max_hr=210
arr= np.zeros((4, 4))
arrHr= np.zeros((4, 4))
ic= np.zeros((2, 3))
icHr= np.zeros((2, 3))
loc = np.zeros(400)
sCount = 0
data = ""
end_message = ""
prevTime = 0
stepTime = 0
sPrev = 0
HrValid = 0
step_goal = 0
goal_reached = False
begin = True

# ==================== Grab 'count' samples from Serial ====================
def grab_samples( n_samples ):

    global sample_count # using global variable 'sample_count'

    t, rawSum, hrIn = np.zeros((3, n_samples))
    for i in range(n_samples) :
        try:
            try:
                data = ser.readline().strip().decode('utf-8')
            except KeyboardInterrupt:
                print("\nexiting grab samples...")
                ser.close()
                exit()
            
            t1, gx1, gy1, gz1, hr1 = data.split(' ')
            t[i] = float(t1)/1000
            rawSum[i] = sum_raw(int(gx1), int(gy1), int(gz1))
            hrIn[i] = hr1

        except ValueError:                                # report error if we failed
            print('Invalid data: "', data, '"')
            print('        ')
            continue

    sample_count += n_samples
    return t, rawSum, hrIn

def process_without_plots(): 
    global times, raw, processed, hrProcessed, hr, ic, icHr, sCount, prevTime, stepTime, sPrev, HrValid, goal_reached, end_message

    # shift samples left by 'NS'
    times[:N-NS] = times[NS:]
    raw[:N-NS] = raw[NS:]
    processed[:N-NS] = processed[NS:]
    hr[:N-NS] = hr[NS:]

    # grab new samples
    #times[N-NS:], values[N-NS:], values2[N-NS:]= grab_samples(NS)
    times[N-NS:], raw[N-NS:], hr[N-NS:]= grab_samples(NS)
    ic, processed[N-NS:]  = flt.process_ir(raw[N-NS:],arr,ic)

    #get Heart Rate
    icHr, hrProcessed[N-NS:] = flt.process_ir(hr[N-NS:], arrHr, icHr)
    beats = np.array([times, hrProcessed])
    Hr = 5
    Hr, loc1 = hr_ecg.calculate_hr_hr(beats)
    if(math.isnan(Hr)):
        nothing = 0
    else:
        #heart rate peak limit
        if(Hr < max_hr):
            HrValid = Hr

    #get steps
    steps = np.array([times, processed])
    HR_useless, loc = hr_ecg.calculate_hr(steps)
    step = np.zeros(400)

    for i in range(N - 1):
        x = loc[i+1] - loc[i]
        if(x > (0.5 * max(processed))):
            step[i] = 1
        else:
            step[i] = 0

    """
    for i in range(N-1):
        if (step[i] == 1):
            stepTime = times[i]     #get the time of current step
            print("stepped")
            if (stepTime - prevTime) > step_pause_interval:
                sCount = sCount + 1
                print("Step: " + str(sCount))
                prevTime = stepTime
    """
    if 1 in step[N-(NS+1):]:
        # stepTime = time.time()        
        #if (stepTime - prevTime) > .5:
        sCount = sCount + 1
        # print("Step: " + str(sCount))
        # ser.write(("Step: " + str(sCount) + ",").encode('utf-8'))
        # prevTime = stepTime

    #check if step count goal reached
    if(sCount >= step_goal):
        goal_reached = True
        end_message = "\nYou've reached your step goal!!!"
    else:
        end_message = "\nSteps remaining: " + str(step_goal-sCount)


    #send data to BLE
    currTime = time.time()
    if((currTime - prevTime) > pause_interval):
        print(("HR: " + format(HrValid, '.2f') + " Steps: " + str(sCount) + end_message + ","))
        try:
            ser.write(("HR: " + format(HrValid, '.2f') + " Steps: " + str(sCount) + end_message + ",").encode('utf-8'))
        except KeyboardInterrupt:
            print("\nexiting process plots...")
            ser.close()
            exit()
        prevTime = currTime
    #Send to databasepip install pyrebase
    #write_to_pyrebase("walker", HrValid, sCount)


# get the sum of raw
def sum_raw(a,b,c):
    return math.sqrt((a*a)+(b*b)+(c*c))

def read_BLE( ser ):
    msg = ""
    if( ser.in_waiting > 0 ):
        msg = ser.readline( ser.in_waiting ).decode('utf-8')
    return msg

# ==================== Main ====================
if (__name__ == "__main__") :
    #for step count
    b_high, a_high = scipy.signal.butter(3, .005, 'highpass', analog=False)
    b_low, a_low = scipy.signal.butter(3, .1, 'lowpass', analog=False)
    arr[0] = a_high
    arr[1] = b_high
    arr[2] = a_low
    arr[3] = b_low

    ic[0] = scipy.signal.lfilter_zi(b_low, a_low)    #initial filter
    ic[1] = scipy.signal.lfilter_zi(b_high, a_high)    #initial filter


    #for HR:
    b_highHR, a_highHR = scipy.signal.butter(3, .005, 'highpass', analog=False)
    b_lowHR, a_lowHR = scipy.signal.butter(3, .1, 'lowpass', analog=False)
    arrHr[0] = a_highHR
    arrHr[1] = b_highHR
    arrHr[2] = a_lowHR
    arrHr[3] = b_lowHR

    icHr[0] = scipy.signal.lfilter_zi(b_lowHR, a_lowHR)    #initial filter
    icHr[1] = scipy.signal.lfilter_zi(b_highHR, a_highHR)    #initial filter
    #print ("before bluetooth")
    
    step_goal = int(input('Please enter your step goal:'))
    '''
    while(type(step_goal) != int):
        print('Please enter your step goal as integer:')
        step_goal = input()
    '''
    #step_goal=10

    with serial.Serial(port=serial_port, baudrate=serial_baud, timeout=1) as ser:
        print("inside bluetooth")
        #try:
        sleep(3)
        try:
            ser.write(b"1")
        except KeyboardInterrupt:
            print("\nexiting serial write...")
            ser.close()
            exit()
        #ser.write(b"\n")
       
        # # initialize the figure 
        # fig, axes = plt.subplots(1,2)

        #times, raw = grab_samples(N)
        print("grab first samples")
        times[N-NS:], raw[N-NS:], hr[N-NS:]= grab_samples(NS)
        sleep(0.5);

        #filter
        ic, processed[N-NS:]  = flt.process_ir(raw[N-NS:],arr,ic)
        icHr, hrProcessed[N-NS:] = flt.process_ir(hr[N-NS:], arrHr, icHr)

        while(True):
            process_without_plots()
        
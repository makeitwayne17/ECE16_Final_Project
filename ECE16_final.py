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



# ==================== Globals ====================
N = 400                                         # samples to plot
NS = 20                                         # samples to grab each iteration
step_pause_interval = .25
sample_count = 0                                # current sample count
times, raw, processed, hr= np.zeros((4, N))                         # data vectors
serial_port = '/dev/cu.usbserial'
#serial_port = '/dev/cu.usbmodem14201'  # the serial port to use
serial_baud = 9600    
arr= np.zeros((4, 4))
arrHr= np.zeros((4, 4))
ic= np.zeros((2, 3))
icHr= np.zeros((2, 3))
loc = np.zeros(400)
sCount = 0
data = ""
prevTime = 0
stepTime = 0
sPrev = 0

# ==================== Grab 'count' samples from Serial ====================
def grab_samples( n_samples ):

    global sample_count # using global variable 'sample_count'

    t, rawSum, hrIn = np.zeros((3, n_samples))
    for i in range(n_samples) :
        try:
            data = ser.readline().strip().decode('utf-8')
            
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

# ==================== Grab new samples and plot ====================
def update_plots(i):
    global times, raw, processed, hr, ic, icHr, sCount, prevTime, stepTime, sPrev

    # shift samples left by 'NS'
    times[:N-NS] = times[NS:]
    raw[:N-NS] = raw[NS:]
    processed[:N-NS] = processed[NS:]
    hr[:N-NS] = hr[NS:]

    # grab new samples
    #times[N-NS:], values[N-NS:], values2[N-NS:]= grab_samples(NS)
    times[N-NS:], raw[N-NS:], hr[:N-NS]= grab_samples(NS)
    ic, processed[N-NS:]  = flt.process_ir(raw[N-NS:],arr,ic)

    #get Heart Rate
    icHr, hrProcessed[N-NS:] = flt.process_ir(hr[N-NS:], arrHr, icHR)
    beats = np.array([times, hrProcessed])
    HR, loc = hr.calculate_hr(beats)


    #stuff for welch filter !!IGNORE!!
    """
    freq, power = scipy.signal.welch(processed, 25)

    maxPower = 0

    #loop through freq and find max power
    for i in range((N-1)*.25):  #only checks the lower 25 % if frequnces
        if(freq[i] > 1):    #gets rid of resting freqency
            if(maxPower < power[i]):
                maxPower = power[i]
    """


    #get steps
    steps = np.array([times, processed])
    HR_useless, loc = hr.calculate_hr(steps)
    step = np.zeros(400)

    for i in range(N - 1):
        x = loc[i+1] - loc[i]
        if(x > (0.75 * max(processed))):
            step[i] = 1
        else:
            step[i] = 0

    for i in range(N-1):
        if (step[i] == 1):
            stepTime = times[i]     #get the time of current step
            #print(times[i])
            if (stepTime - prevTime) > step_pause_interval:
                sCount = sCount + 1
                print("Step: " + str(sCount))
                prevTime = stepTime

    currTime = time.time()
    if((currTime - prevTime) > step_pause_interval):
        ser.write(("HR: " + format(HR, '.2f') + " Steps: " + str(sCount) + ",").encode('utf-8'))
        prevTime = currTime



    #stuff for welch filter !!IGNORE!!
    """
    if 1 in step[N-(NS+1):]:
        stepTime = time.time()        
        if (stepTime - prevTime) > .5:
            sCount = sCount + 1
            print("Step: " + str(sCount))
            ser.write(("Step: " + str(sCount) + ",").encode('utf-8'))
            prevTime = stepTime
    plot
    axes[0].set_xlim(times[0],times[N-1])
    axes[1].set_xlim(times[0],times[N-1])
    """
    
    [ax.set_xlim(times[0],times[N-1]) for ax in axes]
    live_plots[0].set_data(times, hrProcessed)
    live_plots[1].set_data(times, processed)
    axes[1].set_ylim(-4000,4000)
    axes[0].set_ylim(min(hrProcessed)-abs(min(hrProcessed)*.5), max(hrProcessed)*1.5)

    #live_plots[1].set_data(freq, power)
    #axes[1].set_xlim(freq[0], freq[-1])
    #axes[1].set_ylim(min(power), max(power))
    return live_plots

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
    print ("before bluetooth")

    with serial.Serial(port=serial_port, baudrate=serial_baud, timeout=1) as ser:
        
        print("inside bluetooth")
        ser.write("AT+DISC?".encode())
        sleep(0.5)
        ser.write("AT+ROLE1".encode())
        sleep(0.5)
        ser.write("AT+IMME1".encode())
        sleep(0.5)
        ser.write("AT+CON3403DE1C5F63".encode())
        sleep(0.5)
        #try:
        sleep(3)
        ser.write(b"1")
        #ser.write(b"\n")
        # initialize the figure 
        fig, axes = plt.subplots(1,2)

        #times, raw = grab_samples(N)
        times[N-NS:], raw[N-NS:], hr[:N-NS]= grab_samples(NS)
        sleep(0.5);

        #filter
        ic, processed[N-NS:]  = flt.process_ir(raw[N-NS:],arr,ic)
        icHr, hrProcessed[N-NS:] = flt.process_ir(hr[N-NS:], arrHr, icHR)

        #live_plot = axes.plot(times, values, lw=2)[0]
        live_plots = []
        live_plots.append(axes[0].plot(times, hrProcessed, lw=2)[0])
        live_plots.append(axes[1].plot(times, processed, lw=2)[0])
        
        # initialize the y-axis limits and labels
        #axes.set_ylim(0,1023)
        [ax.set_xlim(times[0],times[N-1]) for ax in axes]
        #axes[0].set_ylim(0,10000)
        axes[0].set_ylim(min(hrProcessed)-(min(hrProcessed)*.25), max(hrProcessed)*1.5)
        axes[1].set_ylim(min(processed)-(min(processed)*.25), max(processed)*1.5)

        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('Value')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Value')

        axes[0].set_title('Heart Rate')
        axes[1].set_title('Processed')
        

        # set and start the animation and update at 1ms interval (if possible)
        anim = animation.FuncAnimation(fig, update_plots, interval=1)
        plt.show()

"""
        finally:
            ser.write(b"0")
            ser.close()
            sys.exit(0)
    ser.close()
    sys.exit(0)
ser.close()
sys.exit(0)

    while(True):
         
        #check if connected
        ser.write(("AT+DISC?").encode('utf-8'))
        sleep(1)
        check = read_BLE(ser)
        sleep(1)
        print(check + "checking\n");
        if("3403DE33F837" in check):
            print(check + "inside\n")
            #automate connection
            ser.write( ("AT+ROLE1").encode('utf-8'));
            sleep(0.5) # wait for a response
            ser.write( ("AT+IMME1").encode('utf-8'));
            sleep(0.5) # wait for a response
            ser.write(("AT+CON3403DE33F837").encode('utf-8'));
            sleep(0.5)

        command = input("Either (1) hit ENTER to read BLE, (2) send an AT command, or (3) press q to exit: ")
        sleep(0.5) # wait for a response
        if(command == ""):
            print( "> " + read_BLE(ser) )
        elif (command == 'q' or command == 'Q'):
            print("Goodbye")
            ser.close()
            sys.exit(0)
        else:
            ser.write( command.encode('utf-8') )
            sleep(0.5) # wait for a response
            print( "> " + read_BLE(ser) )

"""
        
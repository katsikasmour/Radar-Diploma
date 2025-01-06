#!/usr/bin/python3
import io
import sys
import queue
import struct
from scipy.fft import fftfreq
import serial
import numpy as np
from scipy.signal import get_window
from scipy.fft import fft, rfft, rfftfreq
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import pandas as pd


#sampling frequency
f_sample = 32000.0
#number of samples
n_sample = 1024

# File paths for I and Q data
i_file_path = 'I_data.csv'
q_file_path = 'Q_data.csv'
dft_i_file_path = 'dft_I_data.csv'
dft_q_file_path = 'dft_Q_data.csv'
combined_absI_data_path='combined_absI_data.csv'


# Initialize CSV files with headers if they do not exist
if not os.path.exists(i_file_path):
    with open(i_file_path, 'w') as f:
        f.write('I\n')

if not os.path.exists(q_file_path):
    with open(q_file_path, 'w') as f:
        f.write('Q\n')

if not os.path.exists(dft_i_file_path):
    with open(dft_i_file_path, 'w') as f:
        f.write('DFT_I\n')

if not os.path.exists(dft_q_file_path):
    with open(dft_q_file_path, 'w') as f:
        f.write('DFT_Q\n')

if not os.path.exists(combined_absI_data_path):
    with open(combined_absI_data_path, 'w') as f:
        f.write('absI\n')

def animate(i, ser, ax1, ax2, ax3, ax4, ax5, speed):
    
    #line =ser.read(4)
    #print(line)
    ser.read_until(b'\x00\xff\xff\x00')
    lineABS=ser.read(int(n_sample  * 4))
    #print(lineABS)
    line = ser.read(n_sample*4)
    
    abs_I=[]
    for i in range(0, int(n_sample* 4), 4):
        val = struct.unpack('<f', lineABS[i:i+4])[0]
        abs_I.append(val)
    
    print("abs_I is", abs_I)

    I = []
    Q = []
    for i in range(0, n_sample*4, 2):
        #if(i < n_sample*2):
        if i % 4 == 0:
            val = 3.3 * (struct.unpack('<H', line[i:i+2])[0]) /4095
            #val = (struct.unpack('<H', line[i:i+2])[0]) 
            #print(round(val,4))
            I.append(val)
        else:
            val = 3.3 * (struct.unpack('<H', line[i:i+2])[0]) /4095
            #val = (struct.unpack('<H', line[i:i+2])[0]) 
            #print(round(val,3))
            Q.append(val)


    # Save I data to CSV
    data_I = pd.DataFrame(I, columns=['I'])
    data_I.to_csv(i_file_path, mode='w', header=False, index=False)
    
    # Save Q data to CSV
    data_Q = pd.DataFrame(Q, columns=['Q'])
    data_Q.to_csv(q_file_path, mode='w', header=False, index=False)

    #Plot the I Signal
    ax1.clear()
    time = np.linspace(0, len(I)-1,len(I))
    ax1.plot(time, I)
    ax1.axis(ymin=0.5,ymax=3.9)
    #ax1.axis(ymin=0,ymax=4000)
    ax1.set_title("I")
    
    #Plot the Q Signal
    ax2.clear()
    time = np.linspace(0, len(Q)-1,len(Q))
    ax2.plot(time, Q)
    ax2.axis(ymin=0.5,ymax=3.9)
    #ax2.axis(ymin=0,ymax=4000)
    ax2.set_title("Q")
    
    
     #Plot the FT without the DC component
    ax3.clear()
    #No window for best frequency resolution
    #window = np.hanning(len(I))
    #I = I * window   
    #Q = Q * window   
    
   
    #Calculate DFT of I and Q signals
    DFT_I = rfft(I)
    # print("I BUFFER:",I)
    # print("Q BUFFER:",Q)
    #print("dft_I BUFFER:",(DFT_I))
    print("size of dft_I is", len(DFT_I))
    #print(DFT_I)
    # print("size of I buffer is", len(I))
    DFT_Q = rfft(Q)

    abs_I=abs_I[0:int(n_sample/2+1)]
# Combine the data into a single DataFrame
    combined_data = pd.DataFrame({
        'absI_STM32': abs_I,     # Data from STM32
        'absI_Script': np.abs(DFT_I)    # Data from the script
    })

    combined_data.to_csv(combined_absI_data_path, mode='w', header=False, index=False)

    
    #print dft_result abs_I of stm32
    ax4.clear()
    xf_I = rfftfreq(len(I), 1 / f_sample)
    ax4.plot(xf_I[1:], 20 * np.log10(abs_I[1:int(n_sample/2+1)]))  #convert y-axis to dB
    ax4.axis(ymin=-40.0,ymax=80)
    ax4.set_title("stm32 FFT of I")
         
    #Plot DFT of I signal
    xf_I = rfftfreq(len(I), 1 / f_sample)
    yf_I = np.abs(DFT_I)
    ax3.plot(xf_I[1:], 20 * np.log10(yf_I[1:]))  #convert y-axis to dB
    ax3.axis(ymin=-40.0,ymax=80)
    ax3.set_title("FFT of I")
    
    # # #Calculate noise floor
    NF = sum(yf_I[1:]) / len(yf_I[1:])
    print("log10(NF) IS:",np.log10(NF))

    if 20 * np.log10(max(yf_I[1:])) > 20 * np.log10(NF) + 30:        #### OFFSET 15 IS SMALL. 30 IS BETTER
        index = np.argmax(yf_I[1:]) + 1
        
        #Calculate and print the speed based on doppler effect
        s = 3.6 * index * (f_sample / len(I)) / 160.0
        if s > 8:
            # Path to your CSV file 
            csv_file_path = "radar_speed_data.csv"
            #Find speed sign using I/Q signals phase difference
            #Speed is considered positive if target is approaching towards the radar
            if np.angle(DFT_I[index] / DFT_Q[index]) < 0:
                s = -1 * s;
            
            print("I/Q phase difference: ", np.angle(DFT_I[index] / DFT_Q[index], deg=True))
            print("max amp bucket: ", index)
            print("frequency: ", index * (f_sample / len(I)))
            print("Detected object with speed: %f km/h" % s)
            speed.put(s)
            
            # Prepare data for saving
            data = {'max_amp_bucket': [index], 'speed': [s]}

            # Convert data to DataFrame
            df = pd.DataFrame(data)

            # Save to CSV, appending data without writing header if the file already exists
            df.to_csv(csv_file_path, mode='a', header=not pd.io.common.file_exists(csv_file_path), index=False)  
        else:
            speed.put(0)
    else:
            speed.put(0)
    
    #Plot the speed history
    ax5.clear()
    speed_val = list(speed.queue)
    time = np.linspace(0, len(speed_val)-1,len(speed_val));
    ax5.plot(time, speed_val)
    ax5.axis(ymin=-40.0,ymax=40)
    ax5.set_title("Speed over time")

def main(argv, argc):
    
    #Get the serial port and baud rate from cml
    if (argc != 3):
        baud = 460800
        com = 'COM3'
        print("Usage: python %s ComPort BaudRate" % argv[0])
    #Default values
    else:
        baud = int(argv[2])
        com = argv[1]
        
    #Open port at baud,8,N,1, blocking
    ser = serial.Serial(com, baud, bytesize=8, timeout=1000)
    print("Serial port %s is open: %s" % (ser.name, ser.is_open))

    #queue to hold the speed measurements over time
    speed = queue.Queue(1000)

    #Animated plots
    fig = plt.figure()
    ax1 = fig.add_subplot(3,2,1)
    ax2 = fig.add_subplot(3,2,2)
    ax3 = fig.add_subplot(3,2,3)
    ax4 = fig.add_subplot(3,2,4)
    ax5 = fig.add_subplot(3,2,5)

    ani = animation.FuncAnimation(fig, animate, fargs=(ser, ax1, ax2, ax3, ax4,ax5, speed, ), interval=50)
    fig.tight_layout()
    plt.show()


if __name__ == '__main__':
    main(sys.argv, len(sys.argv))

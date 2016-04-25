import sys
import serial
import argparse
import numpy as np
from time import sleep
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation


# plot class
class AnalogPlot:
  # constr
  def __init__(self, strPort, maxLen):
      # open serial port
      self.ser = serial.Serial(strPort, 9600)

      self.ax = deque([0.0]*maxLen)
      self.ay = deque([0.0]*maxLen)
      self.m1 = deque([0.0]*maxLen)
      self.m2 = deque([0.0]*maxLen)
      self.maxLen = maxLen

  # add to buffer
  def addToBuf(self, buf, val):
      if len(buf) < self.maxLen:
          buf.append(val)
      else:
          buf.pop()
          buf.appendleft(val)

  # add data
  def add(self, data):
      self.addToBuf(self.ax, data[0])
      self.addToBuf(self.ay, data[1])
      self.addToBuf(self.m1, data[2])
      self.addToBuf(self.m2, data[3])

  # update plot
  def update(self, frameNum, a0, a1, a2, a3):
      #a0 - setpoint, a1 - roll, a2 - error, a3 - PID roll out
      try:
          line = self.ser.readline().strip()

          data = [float(val) for val in line.split()]
          # print data
          #print(data)
          if(len(data) == 4):
              self.add(data)
              a0.set_data(range(self.maxLen), self.ax)
              a1.set_data(range(self.maxLen), self.ay)
              a2.set_data(range(self.maxLen), self.m1)
              a3.set_data(range(self.maxLen), self.m2)
      except KeyboardInterrupt:
          print('exiting')

      return a0,

  # clean up
  def close(self):
      # close serial
      self.ser.flush()
      self.ser.close()

# main() function
def main():
  # create parser
  parser = argparse.ArgumentParser(description="LDR serial")
  # add expected arguments
  parser.add_argument('--port', dest='port', required=True)

  # parse args
  args = parser.parse_args()

  #strPort = '/dev/tty.usbmodem1413'
  strPort = args.port

  print('reading from serial port %s...' % strPort)
  xlim = 1000
  # plot parameters
  analogPlot = AnalogPlot(strPort, xlim)

  print('plotting data...')
  major_ticks = np.arange(-200, 202, 100)
  minor_ticks = np.arange(-221, 221, 10)

  # set up animation
  fig = plt.figure()
  ax = plt.axes(xlim=(0, xlim+1), ylim=(-220, 220))
  ax.set_yticks(minor_ticks, minor=True)
  ax.set_yticks(major_ticks)
  ax.grid(which='both')
  a0, = ax.plot([], [], 'b--', label='Set Point')
  a1, = ax.plot([], [], 'g', label = 'roll')
  a2, = ax.plot([], [], 'r', label='error')
  a3, = ax.plot([], [], 'y', label='PID')
  legend = ax.legend(loc='upper center', shadow=True)
  frame = legend.get_frame()
  frame.set_facecolor('0.90')
  for label in legend.get_texts():
      label.set_fontsize('large')

  for label in legend.get_lines():
      label.set_linewidth(1.5)

  anim = animation.FuncAnimation(fig, analogPlot.update,
                                 fargs=(a0, a1, a2, a3),
                                 interval=1)

  # show plot
  plt.show()

  # clean up
  analogPlot.close()

  print('exiting.')


# call main
if __name__ == '__main__':
  main()
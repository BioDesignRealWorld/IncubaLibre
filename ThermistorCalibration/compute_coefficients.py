""" Thermistor calibration script

This script converts a table of thermistor calibration measurements into a
look-up table (LUT) in C that can be used in microcontroller such as AVR with
an ADC.

To linearize the response of the thermistor, the folowing simple network is used

    Vcc 
     |
     -
    | |
    | | R_T (the thermistor)
     -
     |
     --- Vout (input to ADC)
     |
     -
    | |
    | | R2 (the serie resistor)
     -
     |
    --- GND
     -

The AVX NTC thermistor application notes came handy when writing this script.
http://www.avx.com/docs/masterpubs/ntctherm.pdf

The script needs to be run by ipython. To do so with default parameters type
in the following in a terminal

> ipython -- thermistor_calibration.py <options>


(c) 2014, Robin Scheibler/fakufaku
This script is released in the public domain.
"""

from scipy.special import cbrt
import numpy as np
import matplotlib.pyplot as plt
import sys

def print_help():
  print sys.argv[0],' -i data_file -o lut_file -B bits -L lut_bits -D decimals'
  print 'This script converts a table of thermistor calibration measurements into a'
  print 'look-up table (LUT) in C that can be used in microcontroller such as AVR with'
  print 'an ADC.'
  print ''
  print 'Options:'
  print '   -i  the name of the file containing the calibration data'
  print '       in the following format: two columns, comma separated, '
  print '       "resistance [kOhms], temperature [C]" pairs. (default data.txt)'
  print '   -o  the name of the file where to save the look-up table (default lut.txt)'
  print '   -B  the number of bits of the ADC (default to 10)'
  print '   -L  the number of bits for the look-up table (default to 8)'
  print '   -D  the number of decimals to use in the look-up table. (default is 7)'
  print '   -R  the value of the serie resistance (in Ohms).'
  print '   -h  display this help'


# default values
datafile = 'data.txt'
lutfile = 'lut.c'
B = 10      # 10 bits ADC
L = 8       # 256 values in the LUT
D = 7       # values have 7 significant digits
R2 = 1500.  # The series resistance


# constants
zero_celsius = 273.15

# parse arguments
i = 1
while i < len(sys.argv):
  if sys.argv[i] == '-i':
    datafile = sys.argv[i+1]
    i += 2
  elif sys.argv[i] == '-o':
    lutfile = sys.argv[i+1]
    i += 2
  elif sys.argv[i] == '-B':
    B = int(sys.argv[i+1])
    i += 2
  elif sys.argv[i] == '-L':
    L = int(sys.argv[i+1])
    i += 2
  elif sys.argv[i] == '-D':
    D = int(sys.argv[i+1])
    i += 2
  elif sys.argv[i] == '-R':
    R2 = float(sys.argv[i+1])
    i += 2
  else:
    print_help()


data = np.loadtxt(datafile, dtype=float, comments='#')

# First we fit the data to the three parameters equation
# of section 2.1.5 of AVX document.
R = data[:,0]*1000.
T = data[:,1] + zero_celsius

A = np.vstack((np.ones((len(R))), np.log(R), np.log(R)**3)).T
b = 1./T

p = np.linalg.lstsq(A, b)

t = np.arange(0, 80) + zero_celsius

a,b,c = p[0][0], p[0][1], p[0][2]

it = 1./t
x = (a-it)/c
y = (b/c)
z1 = 27./2.*x
z2 = 3./2.*np.sqrt(3.)*np.sqrt(27*x**2+4*y**3)

r = np.exp(1./3.*(cbrt(-z1+z2) - cbrt(z1+z2)))    # cbrt: cubic root

# generate the lookup table
N = 2**B
div = 2**(B-L)
n = np.arange(0,2**L)
alpha = (n*div+(div-1)/2. + 0.5)/N
R_T = (1./alpha-1.)*R2
lut = 1./(a + b*np.log(R_T) + c*np.log(R_T)**3)

# open and save LUT to file
f_lut = open(lutfile, 'w')
f_lut.write('float therm_lut[] = { ')
format = '%.'+ str(D) +'f'
for i in xrange(0,2**L-1):
  f_lut.write(format % (lut[i]-zero_celsius))
  f_lut.write(', ')
  if (i+1)%10 == 0:
    f_lut.write('\n  ')
f_lut.write(format % lut[-1])
f_lut.write(' };\n')

# plot some stuff
plt.subplot(2,2,1)
plt.plot(t-zero_celsius, r,'k-', T-zero_celsius, R, 'x')
plt.xlabel('Temperature [Celsius]')
plt.ylabel('Resistance of thermistor [Ohm]')
plt.title('Raw')
plt.legend(('Fitted curve', 'Measurements'))
plt.axis('tight')

plt.subplot(2,2,2)
plt.plot(t-zero_celsius, (R2)/(r+R2),'k-', T-zero_celsius, (R2)/(R+R2), 'x')
plt.title('Linearized')
plt.xlabel('Temperature [Celsius]')
plt.ylabel('Value of resistance divider with $R_2=1.5k\Omega$')
plt.legend(('Fitted curve', 'Measurements'))
plt.axis('tight')

plt.subplot(2,2,3)
plt.stem(n, lut-zero_celsius)
plt.title('Look-up table')
plt.xlabel('ADC value')
plt.ylabel('Temperature')
plt.axis('tight')

plt.subplot(2,2,4)
plt.stem(n[:-1], np.diff(lut))
plt.title('Temperature precision')
plt.xlabel('ADC value')
plt.ylabel('$\Delta$T')
plt.axis('tight')


plt.show()


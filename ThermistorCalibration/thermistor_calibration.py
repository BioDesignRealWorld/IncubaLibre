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


def lasso(A, b, gam):
    import cvxpy as cp
    import numpy as np
    import cvxopt
    #from multiprocessing import Pool

    # Problem data.
    gamma = cp.Parameter(sign="positive")

    # Construct the problem.
    x = cp.Variable(A.shape[1])
    objective = cp.Minimize(cp.sum_squares(A * x - b) + gamma * cp.norm(x, 1))
    p = cp.Problem(objective)

    # Assign a value to gamma and find the optimal x.
    def get_x(gamma_value):
        gamma.value = gamma_value
        result = p.solve()
        return x.value

    #gammas = np.logspace(-1, 2, num=100)
    # Serial computation.
    #x_values = [get_x(value) for value in gammas]

    # Parallel computation.
    #pool = Pool(processes=4)
    #par_x = pool.map(get_x, gammas)

    #for v1, v2 in zip(x_values, par_x):
        #if np.linalg.norm(v1 - v2) > 1e-5:
            #print "error"

    return get_x(gam)


def print_help():
    print sys.argv[0], ' -i data_file -o lut_file -B bits -L lut_bits -D decimals'
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
        datafile = sys.argv[i + 1]
        i += 2
    elif sys.argv[i] == '-o':
        lutfile = sys.argv[i + 1]
        i += 2
    elif sys.argv[i] == '-B':
        B = int(sys.argv[i + 1])
        i += 2
    elif sys.argv[i] == '-L':
        L = int(sys.argv[i + 1])
        i += 2
    elif sys.argv[i] == '-D':
        D = int(sys.argv[i + 1])
        i += 2
    elif sys.argv[i] == '-R':
        R2 = float(sys.argv[i + 1])
        i += 2
    else:
        print_help()


data = np.loadtxt(datafile, dtype=float, comments='#')

# First we fit the data to the three parameters equation
# of section 2.1.5 of AVX document.
R = data[:, 0] * 1000.
T = data[:, 1] + zero_celsius

A = np.vstack((np.ones((len(R))), np.log(R), np.log(R) ** 3)).T
b_vec = 1. / T

p = np.linalg.lstsq(A, b_vec)
a, b, c = p[0][0], p[0][1], p[0][2]

#p = lasso(A, b_vec, 1e-4)
#a, b, c = p[1,0], p[1,0], p[2,0]

t = np.arange(0, 80) + zero_celsius

r = np.arange(1, 10001)
t = 1. / (a + b * np.log(r) + c * np.log(r) ** 3)


# generate the lookup table
N = 2 ** B
div = 2 ** (B - L)
n = np.arange(0, 2 ** L)
alpha = (n * div + (div - 1) / 2. + 0.5) / N
R_T = (1. / alpha - 1.) * R2
lut = 1. / (a + b * np.log(R_T) + c * np.log(R_T) ** 3)

# open and save LUT to file
f_lut = open(lutfile, 'w')
f_lut.write('float therm_lut[] = { ')
format = '%.' + str(D) + 'f'
for i in xrange(0, 2 ** L - 1):
    f_lut.write(format % (lut[i] - zero_celsius))
    f_lut.write(', ')
    if (i + 1) % 10 == 0:
        f_lut.write('\n  ')
f_lut.write(format % lut[-1])
f_lut.write(' };\n')

# plot some stuff
plt.subplot(2, 2, 1)
plt.plot(r, t - zero_celsius, 'k-', R, T - zero_celsius, 'x')
plt.xlabel('Resistance of thermistor [Ohm]')
plt.ylabel('Temperature [Celsius]')
plt.title('Raw')
plt.legend(('Fitted curve', 'Measurements'))
plt.axis('tight')
plt.ylim(-5, 150)

plt.subplot(2, 2, 2)
plt.plot((R2)/(r+R2), t-zero_celsius, 'k-', (R2)/(R+R2), T-zero_celsius, 'x')
plt.title('Linearized')
R2_str = '%.2f' % (R2 / 1000.)
plt.xlabel('Value of resistance divider with $R_2=' + R2_str + 'k\Omega$')
plt.ylabel('Temperature [Celsius]')
plt.legend(('Fitted curve', 'Measurements'))
plt.axis('tight')
plt.ylim(-5, 150)

plt.subplot(2, 2, 3)
plt.stem(np.arange(N), lut[np.arange(N) / div] - zero_celsius)
plt.title('Look-up table')
plt.xlabel('ADC value')
plt.ylabel('Temperature')
plt.axis('tight')

plt.subplot(2, 2, 4)
plt.stem(n[:-1], np.diff(lut))
plt.title('Temperature precision')
plt.xlabel('ADC value')
plt.ylabel('$\Delta$T')
plt.axis('tight')


plt.show()

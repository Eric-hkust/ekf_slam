import numpy as np
import matplotlib.pyplot as plt

###plot path
plt.figure(1)
real_x = []
real_y = []
real_x = [float(l.split()[0]) for l in open("5/real_path.txt")]
real_y = [float(l.split()[1]) for l in open("5/real_path.txt")]
l1 = plt.plot(real_x,real_y,'r--',label='real_path')

odom_x = []
odom_y = []
odom_x = [float(l.split()[0]) for l in open("5/odom_path.txt")]
odom_y = [float(l.split()[1]) for l in open("5/odom_path.txt")]
l2 = plt.plot(odom_x,odom_y,'g--',label='odom_estimator')

ekf_x = []
ekf_y = []
ekf_x = [float(l.split()[0]) for l in open("5/ekf_path.txt")]
ekf_y = [float(l.split()[1]) for l in open("5/ekf_path.txt")]
l3 = plt.plot(ekf_x,ekf_y,'y--',label='ekf_estimator')

plt.title('the real path and the estimation')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.savefig("path_plot")

### plot error
plt.figure(2)
error_odom = []
error_ekf = []
T = []
error_odom = [float(l.split()[0]) for l in open("5/error.txt")]
error_ekf = [float(l.split()[1]) for l in open("5/error.txt")]
T = [0.1*i for i in range(len(error_ekf))]
l4 = plt.plot(T,error_odom,'r',label='odom_error')
l5 = plt.plot(T,error_ekf,'g',label='error_ekf')
plt.title('the error')
plt.xlabel('t')
plt.ylabel('error')
plt.legend()
plt.savefig("error_plot")
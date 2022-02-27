import numpy as np
import matplotlib.pyplot as plt

##plot path
def plot_path(file_,color_,label_):
    file_path = file_+"/"+label_+".txt"
    x = []
    y = []
    x = [float(l.split()[0]) for l in open(file_path)]
    y = [float(l.split()[1]) for l in open(file_path)]
    plt.plot(x,y,color_,label=label_)

def plot_error(file_,color_,label_):
    file_path = file_+"/"+label_+".txt"
    error = []
    T = []
    error = [float(l.split()[0]) for l in open(file_path)]
    T = [0.1*i for i in range(len(error))]
    plt.plot(T,error,color_,label=label_)

if __name__=="__main__":
    file_name = "4"
    #plot path
    plt.figure(1)
    plot_path(file_name,"r","real_path")
    plot_path(file_name,"g","predict_path")
    plot_path(file_name,"y","ekf_path")
    plot_path(file_name,"b","icp_path")
    plt.title('the real path and the estimation')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.savefig(file_name+"/path")

    #plot error
    plt.figure(2)
    plot_error(file_name,"g","predict_error")
    plot_error(file_name,"y","ekf_error")
    plot_error(file_name,"b","icp_error")
    plt.title('the error')
    plt.xlabel('t')
    plt.ylabel('error')
    plt.legend()
    plt.savefig(file_name+"/error")
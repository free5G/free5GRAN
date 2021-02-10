import numpy as np
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import traceback

Fs = 7.68e6

x1 = []
x2=[]
x5=[]
x_comp_0 = []
y_comp_0 = []
x_comp_1 = []
y_comp_1 = []
x_comp_2 = []
y_comp_2 = []

try:
    f8 = open("/home/oehmichen/Documents/free5GRAN_merged/free5GRAN/cmake-build-debug/IFFT_ONEframe2_time_domain.txt", "r")

    for x in f8:
        x = x.split("(")[1]
        x = x.split(")")[0]
        re = float(x.split(',')[0])
        im = float(x.split(',')[1])
        x1.append(complex(re,im))

    fig8 = plt.figure()
    plt.specgram(x1, NFFT=256, Fs=Fs)
    #plt.xticks(np.arange(0, 0.001, 0.01))
    plt.title("ONEframe2_time_domain")
    plt.ylim(-Fs/2, Fs/2)
    plt.savefig("/home/oehmichen/Documents/free5GRAN_merged/free5GRAN/cmake-build-debug/ONEframe2_time_domain.pdf", bbox_inches='tight', pad_inches=0.5)
    plt.close(fig8)

except Exception:
    traceback.print_exc()

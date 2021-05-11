import numpy as np
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import traceback

Fs = 30.72e6

x1 = []
x2=[]
x5=[]
x6=[]
x_comp_0 = []
y_comp_0 = []
x_comp_1 = []
y_comp_1 = []
x_comp_2 = []
y_comp_2 = []


try:
    f1 = open("/root/.files/free5GRAN/execution_raw_files/studied_frame.txt", "r")

    for x in f1:
        x = x.split("(")[1]
        x = x.split(")")[0]
        re = float(x.split(',')[0])
        im = float(x.split(',')[1])
        x1.append(complex(re,im))

    fig1 = plt.figure()
    plt.specgram(x1, NFFT=1024, Fs=Fs)
    plt.xticks(np.arange(0, 0.01, 0.001))
    plt.title("Studied Frame")
    plt.ylim(-Fs/2, Fs/2)
    plt.savefig("/root/.files/free5GRAN/visualization_files/studied_frame.pdf", bbox_inches='tight', pad_inches=0.5)
    plt.close(fig1)

except Exception:
    traceback.print_exc()


try:
    f2 = open("/root/.files/free5GRAN/execution_raw_files/moniroting_slots.txt", "r")

    for x in f2:
        x = x.split("(")[1]
        x = x.split(")")[0]
        re = float(x.split(',')[0])
        im = float(x.split(',')[1])
        x2.append(complex(re,im))

    fig2 = plt.figure()
    plt.specgram(x2, NFFT=1024, Fs=Fs)
    plt.yticks(np.arange(-15e6, 15e6, 2e6))
    plt.title("Monitoring slots")
    plt.ylim(-Fs/2, Fs/2)
    plt.savefig("/root/.files/free5GRAN/visualization_files/moniroting_slots.pdf", bbox_inches='tight', pad_inches=0.5)
    plt.close(fig2)

except Exception:
    traceback.print_exc()

try:
    f5 = open("/root/.files/free5GRAN/execution_raw_files/global_signal.txt", "r")

    for x in f5:
        x = x.split("(")[1]
        x = x.split(")")[0]
        re = float(x.split(',')[0])
        im = float(x.split(',')[1])
        x5.append(complex(re,im))

    fig2 = plt.figure()
    plt.specgram(x5, NFFT=1024, Fs=Fs)
    plt.yticks(np.arange(-15e6, 15e6, 2e6))
    plt.title("Global signal")
    plt.ylim(-Fs/2, Fs/2)
    plt.savefig("/root/.files/free5GRAN/visualization_files/global_signal.pdf", bbox_inches='tight', pad_inches=0.5)
    plt.close(fig2)

except Exception:
    traceback.print_exc()

try:
    f3 = open("/root/.files/free5GRAN/execution_raw_files/pdcch_constellation.txt", "r")

    for x in f3:
        x = x.split("(")[1]
        x = x.split(")")[0]
        x_comp_0.append(float(x.split(',')[0]))
        y_comp_0.append(float(x.split(',')[1]))

    fig3 = plt.figure()
    plt.scatter(x_comp_0,y_comp_0, color='red')
    plt.title("PDCCH constellation")
    plt.savefig("/root/.files/free5GRAN/visualization_files/pdcch_constellation.pdf")
    plt.close(fig3)

except Exception:
    traceback.print_exc()

try:
    f4 = open("/root/.files/free5GRAN/execution_raw_files/pdsch_constellation.txt", "r")

    for x in f4:
        x = x.split("(")[1]
        x = x.split(")")[0]
        x_comp_1.append(float(x.split(',')[0]))
        y_comp_1.append(float(x.split(',')[1]))


    fig4 = plt.figure()
    plt.scatter(x_comp_1,y_comp_1, color='red')
    plt.title("PDSCH constellation")
    plt.savefig("/root/.files/free5GRAN/visualization_files/pdsch_constellation.pdf")
    plt.close(fig4)
except Exception:
    traceback.print_exc()
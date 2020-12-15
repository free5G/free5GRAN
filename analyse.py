import numpy as np
import matplotlib.pyplot as plt

Fs = 30.72e6

x1 = []
x2=[]
x_comp_0 = []
y_comp_0 = []
x_comp_1 = []
y_comp_1 = []
x_comp_2 = []
y_comp_2 = []

f1 = open("build/output_files/studied_frame.txt", "r")

for x in f1:
    x = x.split("(")[1]
    x = x.split(")")[0]
    re = float(x.split(',')[0])
    im = float(x.split(',')[1])
    x1.append(complex(re,im))

f2 = open("build/output_files/moniroting_slots.txt", "r")

for x in f2:
    x = x.split("(")[1]
    x = x.split(")")[0]
    re = float(x.split(',')[0])
    im = float(x.split(',')[1])
    x2.append(complex(re,im))

plt.specgram(x1, NFFT=1024, Fs=Fs)
plt.xticks(np.arange(0, 0.01, 0.001))
plt.title("Studied Frame")
plt.ylim(-Fs/2, Fs/2)
plt.savefig("files/studied_frame.pdf", bbox_inches='tight', pad_inches=0.5)
plt.close()

plt.specgram(x2, NFFT=1024, Fs=Fs)
plt.yticks(np.arange(-15e6, 15e6, 2e6))
plt.title("Monitoring slots")
plt.ylim(-Fs/2, Fs/2)
plt.savefig("files/moniroting_slots.pdf", bbox_inches='tight', pad_inches=0.5)
plt.close()


f3 = open("build/output_files/pdcch_constellation.txt", "r")

for x in f3:
    x = x.split("(")[1]
    x = x.split(")")[0]
    x_comp_0.append(float(x.split(',')[0]))
    y_comp_0.append(float(x.split(',')[1]))

f4 = open("build/output_files/pdsch_constellation.txt", "r")

for x in f4:
    x = x.split("(")[1]
    x = x.split(")")[0]
    x_comp_1.append(float(x.split(',')[0]))
    y_comp_1.append(float(x.split(',')[1]))


plt.scatter(x_comp_0,y_comp_0, color='red')
plt.title("PDCCH constellation")
plt.savefig("files/pdcch_constellation.pdf")
plt.close()

plt.scatter(x_comp_1,y_comp_1, color='red')
plt.title("PDSCH constellation")
plt.savefig("files/pdsch_constellation.pdf")
plt.close()

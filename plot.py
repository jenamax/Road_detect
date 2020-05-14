from matplotlib import pyplot as plt
import numpy as np

bord_f = open("bord.txt", "r")
s = bord_f.readlines()
bord = []
for line in s:
    l = line.split(' ')
    bord.append([int(l[0]), int(l[1])])


ang_f = open("ang.txt", "r")

ang_s = ang_f.readlines()

ang = [[] for i in range(0, 16)]
j = 0
for line in ang_s:
    l = line.split(' ')
    for i in l:
        if i != '' and i != '\n':
            ang[j].append(float(i))
    j += 1


dd_f = open("d_dist0.txt", "r")

d_dist_s = dd_f.readlines()

d_dist = [[] for i in range(0, 16)]
j = 0
for line in d_dist_s:
    l = line.split(' ')
    for i in l:
        if i != '' and i != '\n':
            d_dist[j].append(float(i))
    j += 1

peaks_f = open("peaks.txt", "r")
peaks_str = peaks_f.readlines()
peaks = [[] for i in range(0, 16)]
j = 0
for line in peaks_str:
    l = line.split(' ')
    for i in l:
        if i != '' and i != '\n' and int(i) < len(d_dist[j]):
            peaks[j].append(int(i))
    j += 1

d_f = open("dist0.txt", "r")
dist_str = d_f.readlines()

dist = [[] for i in range(0, 16)]
j = 0
for line in dist_str:
    l = line.split(' ')
    for i in l:
        if i != '' and i != '\n':
            dist[j].append(float(i))
    j += 1

scan_num = 1
mod = '2'
for scan_num in range(2, 8):
    plt.clf()
    d = d_dist[scan_num]
    a = ang[scan_num]
    if mod == '2':
        plt.plot(a, d)
        #plt.plot([a[j] for j in peaks[scan_num]], [d[i] for i in peaks[scan_num]], 'yo')
        if bord[scan_num][0] < len(d):
            #plt.plot([a[j] for j in bord[scan_num]], [d[i] for i in bord[scan_num]], 'ro')
            plt.vlines([a[j] for j in bord[scan_num]], 0, max(d), label='border')
        plt.ylim(0, 0.5)
        plt.legend()
        plt.xlabel('Angle of lidar rod in the point, rad')
        plt.ylabel('Distance derivative, m')
        plt.title("D_Dist for "  + str(scan_num) + " rod")
        plt.savefig("plots/d_dist" + str(scan_num) + ".png")
        #plt.show()

        plt.clf()
        d = dist[scan_num]
        plt.plot(a, d)
        #plt.plot([a[j] for j in peaks[scan_num]], [d[i] for i in peaks[scan_num]], 'yo')
        if bord[scan_num][0] < len(d):
            #plt.plot([a[j] for j in bord[scan_num]], [d[i] for i in bord[scan_num]], 'ro')
            plt.vlines([a[j] for j in bord[scan_num]], 0, max(d), label='border')
        plt.legend()
        plt.title("Dist for "  + str(scan_num) + " rod")
        plt.xlabel('Angle of lidar rod in the point, rad')
        plt.ylabel('Distance, m')
        plt.savefig("plots/dist" + str(scan_num) + ".png")
        #plt.show()
    else:
        plt.plot(d)
        plt.plot(peaks[scan_num], [d[i] for i in peaks[scan_num]], 'yo')
        if bord[scan_num][0] < len(d):
            plt.plot(bord[scan_num], [d[i] for i in bord[scan_num]], 'ro')
        plt.title("D_Dist for "  + str(scan_num) + " rod")
        plt.savefig("plots/d_dist" + str(scan_num) + ".png")
        #plt.show()

        plt.clf()
        d = dist[scan_num]
        plt.plot(d)
        plt.plot(peaks[scan_num], [d[i] for i in peaks[scan_num]], 'yo')
        if bord[scan_num][0] < len(d):
            plt.plot(bord[scan_num], [d[i] for i in bord[scan_num]], 'ro')
        plt.title("Dist for "  + str(scan_num) + " rod")
        plt.savefig("plots/dist" + str(scan_num) + ".png")
        #plt.show()

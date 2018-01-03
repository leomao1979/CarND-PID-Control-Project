import matplotlib.pyplot as plt
import sys

filename = "all_ctes.txt"
if len(sys.argv) == 2:
    filename = sys.argv[1]

text_file = open(filename, "r")
lines = text_file.read().split(' ')
all_ctes = []
for line in lines:
    line = line.strip()
    if (len(line) > 0):
        all_ctes.append(float(line)) 

plt.figure()
plt.xlabel("time")
plt.ylabel("CTE")
plt.plot(all_ctes)
fig_filename = filename.split('.')[0] + '.png'
plt.savefig(fig_filename ,bbox_inches='tight')
plt.show()

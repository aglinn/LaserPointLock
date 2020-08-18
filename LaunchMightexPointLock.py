import sys
import subprocess

num_instances = 1

procs = []
for i in range(num_instances):
    proc = subprocess.Popen([sys.executable, 'display.py', '{}in.csv'.format(i), '{}out.csv'.format(i)])
    procs.append(proc)

for proc in procs:
    proc.wait()
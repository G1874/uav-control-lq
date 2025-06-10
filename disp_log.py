from matplotlib import pyplot as plt
import numpy as np

x_str = []
x_error_str = []
x_ref_str = []
u_str = []
T_str = []

last_log = ""
i = 0
with open("./logs/python3_3120_1749499010345.log", 'r') as f:
    for line in f:
        if 'Switching to offboard mode' in line or 'Arm command sent' in line:
            continue

        if "x:" in line:
            x_str.append(line)
            last_log = "x:"
        elif "x_error:" in line:
            x_error_str.append(line)
            last_log = "x_error:"
        elif "u:" in line:
            u_str.append(line)
            last_log = "u:"
        elif "T:" in line:
            T_str.append(line)
            last_log = "T:"
        elif "x_ref" in line:
            x_ref_str.append(line)
            last_log = "x_ref:"
        
        elif last_log == "x:":
            x_str[-1] = x_str[-1] + line
        elif last_log == "x_error:":
            x_error_str[-1] = x_error_str[-1] + line
        elif last_log == "u:":
            u_str[-1] = u_str[-1] + line
        elif last_log == "T:":
            T_str[-1] = T_str[-1] + line
        elif last_log == "x_ref:":
            x_ref_str[-1] = x_ref_str[-1] + line

def log_str2num(log_str, str_start, data_len):
    log = []
    for line in log_str:
        if line[-2] != ']':
            print(line)
            break
        line = line[line.find(str_start) + len(str_start):-2]
        line = line.replace('\n',' ')

        log.append([float(h) for h in line.split()])

    return np.array(log)

x = log_str2num(x_str, 'x: [', 12)
# x_error = log_str2num(x_error_str, 'x_error: [', 12)
T = log_str2num(T_str, 'T: [', 4)
u = log_str2num(u_str, 'u: [', 4)
x_ref = log_str2num(x_ref_str, 'x_ref: [', 12)

plt.figure(1)
plt.subplot(4,1,1)
plt.plot(T[0:,0])
plt.subplot(4,1,2)
plt.plot(T[0:,1])
plt.subplot(4,1,3)
plt.plot(T[0:,2])
plt.subplot(4,1,4)
plt.plot(T[0:,3])

plt.figure(2)
plt.subplot(3,1,1)
plt.plot(x[0:,0],label='x')
plt.plot(x_ref[0:,0],label='x ref')
# plt.plot(x_error[0:,0],label='x error')
plt.legend()
plt.subplot(3,1,2)
plt.plot(x[0:,1],label='y')
plt.plot(x_ref[0:,1],label='y ref')
# plt.plot(x_error[0:,1],label='x error')
plt.legend()
plt.subplot(3,1,3)
plt.plot(x[0:,2],label='z')
plt.plot(x_ref[0:,2],label='z ref')
# plt.plot(x_error[0:,2],label='z error')
plt.legend()

plt.figure(3)
plt.subplot(3,1,1)
plt.plot(x[0:,6],label='roll')
plt.plot(x_ref[0:,6],label='roll ref')
# plt.plot(x_error[0:,6],label='roll error')
plt.legend()
plt.subplot(3,1,2)
plt.plot(x[0:,7],label='pitch')
plt.plot(x_ref[0:,7],label='pitch ref')
# plt.plot(x_error[0:,7],label='pitch error')
plt.legend()
plt.subplot(3,1,3)
plt.plot(x[0:,8],label='yaw')
plt.plot(x_ref[0:,8],label='yaw ref')
# plt.plot(x_error[0:,8],label='yaw error')
plt.legend()

plt.show()
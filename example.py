import numpy as np

w1 = [1,2]
w2 = [3,4]
W = np.vstack((w1, w2))

N = 2
M = 2

print(W)
s1 = 1
s2 = 2
S = np.vstack((s1, s2))

sigma_total = np.cov(W, S, rowvar=0)
sigma_ww = sigma_total[:N, :N]
sigma_sw = sigma_total[N, :N]
sigma_ws = sigma_total[:N, N]
sigma_ss = sigma_total[N:, N:]

mean_s = np.mean(S, 0)
mean_w = np.mean(W, 0)

print("sigma_ww = " + str(sigma_ww))
print("sigma_ws = " + str(sigma_ws))
print("sigma_sw = " + str(sigma_sw))
print("sigma_ss = " + str(sigma_ss))
print(mean_s)
print(mean_w)
print()
print(sigma_total)
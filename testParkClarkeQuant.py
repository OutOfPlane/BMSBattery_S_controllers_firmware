import numpy as np
import matplotlib.pyplot as plt

INT16_MAX = 32767

phi = np.linspace(0, np.pi*2, 256)
sin_a = (np.sin(phi)*127).astype(np.int16)
cos_a = (np.cos(phi)*127).astype(np.int16)

val = 127 #max power

	# Inverse park transform
alpha = -sin_a * val  # -sin(angle) * Uq;
beta = cos_a * val    #  cos(angle) * Uq;
beta = beta // 8 # 7/8 = 0.875 is close enough to sqrt(3)/2 = 0.866
beta *= 7 
#values alpha and beta now range +- 16384

# Clarke transform
Ua = alpha + (INT16_MAX >> 1)
Ub = -(alpha >> 1) + beta + (INT16_MAX >> 1)
Uc = -(alpha >> 1) - beta + (INT16_MAX >> 1)

Ua = Ua >> 7
Ub = Ub >> 7
Uc = Uc >> 7

print(np.max(Ua))
print(np.max(Ub))
print(np.max(Uc))

print(np.min(Ua))
print(np.min(Ub))
print(np.min(Uc))


plt.plot(Ua)
plt.plot(Ub)
plt.plot(Uc)
plt.show()
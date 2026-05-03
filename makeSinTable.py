import numpy as np

phi = np.linspace(0, np.pi/2, 64)
vals = (np.sin(phi)*127).astype(np.int16)

for v in vals:
    print(str(v) + ", ", end='')
print()
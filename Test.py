import matplotlib.pyplot as plt
import numpy as np

E = 12
list_x = np.arange(0.1,1,0.01)
list_x2 = list(range(0,10))
print(list_x)
s = 255/(1+(0.25/list_x)**E)
plt.plot(list_x,s)
plt.show()

# list_x = list(range(0,10))
# list_y = list(range(0,10))
# print(list_x)
# for x in list_x:
#     for y in list_y:
#         if y == 3:
#             break
#         print('x,y',x,y)


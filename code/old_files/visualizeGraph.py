import matplotlib.pyplot as plt
import numpy as np

graph= np.array([[0, 4, 0, 0, 0, 0, 0, 8, 0],
        [4, 0, 8, 0, 0, 0, 0, 11, 0],
        [0, 8, 0, 7, 0, 4, 0, 0, 2],
        [0, 0, 7, 0, 9, 14, 0, 0, 0],
        [0, 0, 0, 9, 0, 10, 0, 0, 0],
        [0, 0, 4, 14, 10, 0, 2, 0, 0],
        [0, 0, 0, 0, 0, 2, 0, 1, 6],
        [8, 11, 0, 0, 0, 0, 1, 0, 7],
        [0, 0, 2, 0, 0, 0, 6, 7, 0]
        ])

# turns off axis ticks and labels
plt.axis('off')

# Loop over data dimensions and create text annotations.
for i in range(9):
    for j in range(9):
        text = plt.text(j, i, graph[i, j], ha="center", va="center", color="w")


plt.imshow(graph)
plt.show()



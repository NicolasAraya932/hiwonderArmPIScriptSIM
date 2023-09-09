import numpy as np
import matplotlib.pyplot as plt

def cuadrado_trayectory(T):
    T = T * 10
    xeed = np.concatenate((0.08 * np.ones(T), np.linspace(0.08, 0.16, T), 0.16 * np.ones(T), np.linspace(0.16, 0.08, T)))
    yeed = np.concatenate((np.linspace(-0.04, 0.04, T), 0.04 * np.ones(T), np.linspace(0.04, -0.04, T), -0.04 * np.ones(T)))
    zeed = 0.08 * np.ones(4 * T)

    return [xeed, yeed, zeed]

def circular_trajectory(To, n, R):
    xeed = R * np.cos(To * n) + 0.15  # R * np.cos(To * n) + xc
    yeed = R * np.sin(To * n) + 0
    zeed = 0.08 * np.ones(len(n))  # 0.08 * np.ones(len(xeed)) + zc

    """xeedp = -To * R * sin(To * n)
    yeedp = To * R * cos(To * n)
    zeedp = -To * 0.018 * sin(To * n)"""

    return [xeed, yeed, zeed]

def arreglar_trayectoria(coords, Z, xc, yc, zc):
    # 0,393701 constante de pixel a cm

    X = []
    Y = []
    for (i, j) in coords:
        X.append(i)
        Y.append(j)

    xeed = np.add(X, xc)
    yeed = np.add(Y, yc)
    zeed = Z * zc

    """plt.plot(np.linspace(0, 1, len(yeed)), xeed, color='blue')
    plt.plot(np.linspace(0, 1, len(yeed)), yeed, color='red')
    plt.show()"""

    return [xeed, yeed, zeed]

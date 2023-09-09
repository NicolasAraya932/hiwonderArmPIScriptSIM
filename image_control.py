import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os


class Image_Processor:
    """
    Esta sirve solo para dibujos con un contorno negro.
    """

    def __init__(self, folder):
        self.pictures = os.listdir(folder)
        self.img = ''
        for a in self.pictures:
            if a.endswith('.png') or a.endswith('.jpg') or a.endswith(".jpeg"):
                self.img = cv2.imread(folder + '/' + a)

        self.size = self.img.shape
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, self.binary = cv2.threshold(self.gray, 100, 255, cv2.THRESH_BINARY_INV)

        self.coords = [[], []]
        self.cCoords = []

    def main_Control(self):
        self.binary = cv2.flip(self.binary, 0)
        cnts, hierarchy = cv2.findContours(self.binary.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        x = []
        y = []
        for c in cnts:
            for pos in c:
                for j in pos:
                    x.append(j[0] * 0.0264583333)
                    y.append(j[1] * 0.0264583333)
        for i in range(len(x)):
            self.cCoords.append((x[i], y[i]))

    def getCoords(self):
        return self.cCoords

    def saveImg(self):
        print('Saving resultant image')
        cv2.imwrite('Result.png', cv2.flip(self.binary, 0))

    def adecuar_al_brazo(self, origin_point, limit):
        self.cCoords = np.subtract(self.cCoords, min(self.cCoords))
        x = limit / (np.amax(self.cCoords) + 10)
        self.cCoords = np.dot(self.cCoords, x)
        self.cCoords = np.add(self.cCoords, origin_point)

    def display_Img(self):
        if self.coords is [[], []]:
            print('Suggestion: Do class.makeCoords()')
        else:
            print('Plotting coordinates to see the result')
            x = []
            y = []
            for (i, j) in self.cCoords:
                x.append(i)
                y.append(j)

            plt.scatter(x, y)
            plt.scatter(x[0], y[0], 10)
            plt.scatter(x[-1], y[-1], 10)
            plt.show()

            print('Showing resultant image')
            cv2.imshow('imagen', cv2.flip(self.binary, 0))
            cv2.waitKey(0)

class Drawed_Image_Processor:
    def __init__(self, folder):
        self.pictures = os.listdir(folder)
        self.img = ''
        for a in self.pictures:
            if a.endswith('.png') or a.endswith('.jpg') or a.endswith(".jpeg"):
                self.img = cv2.imread(folder + '/' + a)

        self.size = self.img.shape

        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        _, self.binary = cv2.threshold(self.gray, 100, 255, cv2.THRESH_BINARY_INV)

        self.coords = [[], []]
        self.cCoords = []

    def main_Control(self):
        img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(img, (0, 100, 10), (5, 255, 255))
        mask2 = cv2.inRange(img, (165, 100, 10), (180, 255, 255))

        mask = cv2.bitwise_or(mask1, mask2)
        red_def = cv2.bitwise_and(img, img, mask=mask)

        gray = cv2.cvtColor(red_def, cv2.COLOR_BGR2GRAY)
        _, self.binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        self.binary = cv2.flip(self.binary, 0)
        cnts, hierarchy = cv2.findContours(self.binary.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        for i in range(3):
            self.binary = cv2.drawContours(self.binary, cnts, -1, (255, 255, 255), 2)
            cnts, hierarchy = cv2.findContours(self.binary.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        x = []
        y = []
        for c in cnts:
            for pos in c:
                for j in pos:
                    x.append(j[0] * 0.0264583333)
                    y.append(j[1] * 0.0264583333)
        for i in range(len(x)):
            self.cCoords.append((x[i], y[i]))

    def getCoords(self):
        return self.cCoords

    def saveImg(self):
        print('Saving resultant image')
        cv2.imwrite('Result.png', cv2.flip(self.binary, 0))

    def adecuar_al_brazo(self, origin_point, limit):
        self.cCoords = np.subtract(self.cCoords, min(self.cCoords))
        x = limit / (np.amax(self.cCoords) + 10)
        self.cCoords = np.dot(self.cCoords, x)
        self.cCoords = np.add(self.cCoords, origin_point)

    def display_Img(self):
        if self.coords is [[], []]:
            print('Suggestion: Do class.makeCoords()')
        else:
            print('Plotting coordinates to see the result')
            x = []
            y = []
            for (i, j) in self.cCoords:
                x.append(i)
                y.append(j)

            plt.hist(self.cCoords)
            plt.figure()
            plt.plot(x, y)
            plt.figure()
            plt.scatter(x, y)

            plt.figure()
            print('Showing resultant image')
            plt.imshow(cv2.flip(self.binary, 0))
            plt.show()

if __name__ == '__main__':
    imagen = Drawed_Image_Processor('Micelaneos')
    imagen.main_Control()
    imagen.adecuar_al_brazo(0.05, 0.2)
    imagen.display_Img()
    imagen.saveImg()
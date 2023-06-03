'''
camera.py
Sample client for the Pioneer P3DX mobile robot that receives and
displays images from the camera.
Copyright (C) 2023 Javier de Lope
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import cv2
import robotica
import matplotlib.pyplot as plt
import numpy as np

kp_giros = 0.01
ki_giros = 0.001
kd_giros = 0.01

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd =kd
        self.error_previo = 0
        self.integral_previa = 0

    def calcular_salida(self, valor_deseado, valor_medido):
        error_actual = valor_deseado - valor_medido

        proporcional = self.kp * error_actual
        integral = self.integral_previa + (self.ki * error_actual)
        derivativo = self.kd * (error_actual - self.error_previo)

        salida = proporcional + integral + derivativo

        self.error_previo = error_actual
        self.integral_previa = integral

        return salida



def calcular_centro(img):
    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    bajo1 = np.array([0, 100, 20])
    alto1 = np.array([8, 255, 255])
    bajo2 = np.array([175, 100, 20])
    alto2 = np.array([179, 255, 255])
    mask1 = cv2.inRange(img_hsv, bajo1, alto1)
    mask2 = cv2.inRange(img_hsv, bajo2, alto2)
    mask = cv2.add(mask1, mask2)
    contours, hierarchy = cv2.findContours(image=mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    if contours:
        moments = cv2.moments(contours[0])
        if moments['m00'] != 0:
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])
            centro = (cx, cy)
            img_copy = img.copy()
            img_contornos = cv2.drawContours(image=img_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
            img_centro = cv2.circle(img_contornos, (cx, cy), 2, (0, 255, 0), -1)
        else:
            centro = None
            img_centro = img
    else:
        centro = None
        img_centro = img
    return(centro, img_centro)


def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX', True)
    pid_giros = PID(kp_giros, ki_giros, kd_giros)
    coppelia.start_simulation()
    while coppelia.is_running():
        img = robot.get_image()
        centro, img_centro = calcular_centro(img)
        if centro:
            salida_giros = pid_giros.calcular_salida(128, centro[0])
            print(salida_giros)
            cv2.imshow('opencv', img_centro)
            cv2.waitKey(1)
            robot.set_speed(-salida_giros +4, salida_giros +4) 
        else:
            cv2.imshow('opencv', img_centro)
            cv2.waitKey(1)
            robot.set_speed(0,0)
    coppelia.stop_simulation()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
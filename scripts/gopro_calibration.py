# Calibration parameters for the GoPro HERO 4 camera (720p, 60 Hz)
import numpy as np

newcameramtx = np.array([[510.35546875,   0.        , 347.95616469],
                         [  0.        , 505.87149048, 243.17950429],
                         [  0.        ,   0.        ,   1.        ]])

mtx = np.array([[585.47484723,   0.        , 345.85922634],
                [  0.        , 585.38323032, 243.36084742],
                [  0.        ,   0.        ,   1.        ]])

dist = np.array([[-2.69564625e-01, 1.12189731e-01, 5.04159086e-05, -1.52187322e-03, -1.15975669e-02]])

roi = (16, 22, 608, 435)

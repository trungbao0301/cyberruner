import numpy as np

OCAM = {
    "pol":    np.array([-2.019736e+03, 0.0, 1.721818e-03, -2.128158e-06, 1.185387e-09],
                       dtype=np.float64),
    "invpol": np.array([6527.896707, 27857.075672, 18825.582643, -214449.985674,
                        -792381.859202, -1374262.433263, -1419615.741175,
                        -917497.821432, -364331.873758, -81330.665119, -7808.821317],
                       dtype=np.float64),
    "cy": 598.837773, "cx": 958.882643,
    "c": 0.999326,    "d": 0.000071,   "e": 0.000066,
    "height": 1200,   "width":  1920,
}

OUT_W, OUT_H   = 960, 720
DEFAULT_FX     = 850.0
DEFAULT_FY     = 750.0

K_RECT = np.array([[DEFAULT_FX, 0,          OUT_W / 2.0],
                   [0,          DEFAULT_FY,  OUT_H / 2.0],
                   [0,          0,           1.0        ]], dtype=np.float64)
DIST_RECT = np.zeros(4, dtype=np.float64)

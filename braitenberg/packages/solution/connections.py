from typing import Tuple

import numpy as np
def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = get_np_zeros(shape)
    res[100:150, 100:150] = 0.5
    res[300:, 200:] = 0.5
    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = get_np_zeros(shape)
    res[100:150, 100:300] = 0.5
    return res


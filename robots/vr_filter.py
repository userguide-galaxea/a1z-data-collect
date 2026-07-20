import numpy as np


class WeightedMovingFilter:
    """加权滑动平均滤波器（参考 Unitree xr_teleoperate）。"""

    def __init__(self, weights, data_size=6):
        self._window_size = len(weights)
        self._weights = np.array(weights, dtype=np.float64)
        assert np.isclose(np.sum(self._weights), 1.0)
        self._data_size = data_size
        self._filtered_data = np.zeros(self._data_size)
        self._data_queue = []

    def next(self, new_data):
        assert len(new_data) == self._data_size
        if len(self._data_queue) > 0 and np.array_equal(new_data, self._data_queue[-1]):
            return self._filtered_data.copy()
        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)
        self._data_queue.append(new_data.copy())
        self._filtered_data = self._apply_filter()
        return self._filtered_data.copy()

    def _apply_filter(self):
        if len(self._data_queue) < self._window_size:
            return self._data_queue[-1].copy()
        data_array = np.array(self._data_queue)
        result = np.zeros(self._data_size)
        for i in range(self._data_size):
            result[i] = np.dot(data_array[:, i], self._weights)
        return result

    def reset(self):
        self._data_queue.clear()
        self._filtered_data = np.zeros(self._data_size)

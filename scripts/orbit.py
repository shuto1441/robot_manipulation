from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Union

import cv2
import numpy as np
from numpy.linalg import norm


FIELD_W = 400  # [mm]
FIELD_H = 300  # [mm]


class Orbit:

    def __init__(
        self, linearity_thresh: float, positional_resolution: int,
        max_pred_iter: int = 100,
        floorfriction_ratio: Tuple[float, float] = (1.0, 1.0),
        wallbounce_ratio: Tuple[float, float] = (1.0, 1.0)
    ) -> None:
        self.linearity_thresh = linearity_thresh
        self.positional_resolution = positional_resolution
        self.max_pred_iter = max_pred_iter
        self.floorfriction_ratio = floorfriction_ratio
        self.wallbounce_ratio = wallbounce_ratio
        self._points = deque(maxlen=3)

    def add(self, point: List[Union[float, int, int]]) -> None:
        """パックの観測時刻，座標を追加する．

        Parameters
        ----------
            points: [時刻, x座標, y座標]形式のリスト
        """
        self._points.append(Point(*point))

    def predict(self) -> Union[Union[float, int, int], None]:
        """これまでのパックの観測データから，先のステップまでの軌道を点群として
        予測する．

        Returns
        -------
            [[時刻1, x座標1, y座標1], [時刻2, x座標2, y座標2], ...]形式のリスト
            or None（軌道の予測が不可能な場合）
        """
        points = self._points.copy()
        # データ点数が3点未満と少ない場合
        if len(points) < 3:
            return None

        # 3点が，壁際の折り返し地点で直線上にない場合
        if _linearity(points) < self.linearity_thresh:
            return None

        # 軌道予測
        p_pre, p_cur = points[1], points[2]
        vec = p_cur.coord - p_pre.coord  # 過去2点のベクトル
        ratio = self.positional_resolution / norm(vec)  # 分解能あたりに変換
        norm_vec = vec * ratio
        time_step = (p_cur.t - p_pre.t) * ratio  # 分解能分進む時間を時間ステップにする

        preds = self._predict_points(
            p_cur.t, time_step, p_cur.coord, norm_vec, 0, [])
        return preds

    def _predict_points(
        self, time: float, time_step: float, pos: np.ndarray, vec: np.ndarray,
        i: int, preds: List[Union[float, int, int]]
        ) -> List[Union[float, int, int]]:
        """再帰的に軌道を予測する．床や壁との摩擦による減速を反映．"""

        i += 1
        vec_next = vec * self.floorfriction_ratio  # 床面摩擦による減速
        x_next_tmp, y_next_tmp = pos + vec_next

        if x_next_tmp < 0 or FIELD_W < x_next_tmp:  # 端への到達判定
            return preds

        if y_next_tmp < 0 or FIELD_H < y_next_tmp:  # 壁との衝突判定
            vec_next[0] *= self.wallbounce_ratio[0]  # 壁面摩擦による減速
            vec_next[1] *= -self.wallbounce_ratio[1]  # 跳ね返りによる減速

        x_next, y_next = pos_next = pos + vec_next
        preds.append([time+time_step*i, int(x_next), int(y_next)])

        if i >= self.max_pred_iter:
            return preds
        return self._predict_points(
            time, time_step, pos_next, vec_next, i, preds)


def _linearity(points: List['Point']):
    """観測された3点が直線状にあるかを判定する．"""
    p1, p2, p3 = points
    return norm(p1.coord-p3.coord) / \
           (norm(p1.coord-p2.coord) + norm(p2.coord-p3.coord))


@dataclass
class Point:
    """時刻，[x, y]座標を保持する．"""
    t: float
    x: int
    y: int

    def __post_init__(self) -> None:
        self.coord: np.ndarray = np.array([self.x, self.y])


class Test:

    def __init__(self) -> None:
        self.init_points = [
            [10.00, 200, 200],
            [10.04, 206, 218],
            [10.09, 214, 244]
        ]

    def simulate(self, orbit: 'Orbit') -> None:
        for point in self.init_points:
            orbit.add(point)
        pred_points = orbit.predict()
        self._show(pred_points)

    def _show(self, pred_points: List[Union[float, int, int]]) -> None:
        self.field = np.zeros((FIELD_H, FIELD_W, 3))
        for point in self.init_points:
            self._plot(point, (0, 255, 0))
        for point in pred_points:
            self._plot(point, (255, 255, 255))
            cv2.imshow('simulation', self.field)
            key = cv2.waitKey(0)
            if key == ord('n'):
                continue
            elif key == ord('q'):
                cv2.destroyAllWindows()
                break

    def _plot(
        self, point: Union[float, int, int], color: Tuple[int, int, int]
    ) -> None:
        t, x, y = point
        cv2.circle(self.field, (x, y), 3, color, thickness=-1)
        # cv2.putText(
        #     self.field, text=str(round(t, 2)), org=(x, y), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        #     fontScale=0.4, color=color, thickness=1, lineType=cv2.LINE_AA)


if __name__ == "__main__":
    test = Test()
    test.simulate(Orbit(0.8, 20, 100, (1.0, 1.0), (1.0, 1.0)))
    test.simulate(Orbit(0.8, 20, 100, (0.95, 0.95), (0.95, 0.5)))

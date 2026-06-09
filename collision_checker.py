#!/usr/bin/env python3
import numpy as np
import scipy.spatial
from math import sin, cos, pi, sqrt

class CollisionChecker:
    def __init__(self, circle_offsets, circle_radii, weight):
        self._circle_offsets = np.array(circle_offsets)
        self._circle_radii   = np.array(circle_radii)
        self._weight         = weight

    def collision_check(self, paths, obstacles):
        collision_check_array = np.zeros(len(paths), dtype=bool)
        for i in range(len(paths)):
            collision_free = True
            path = paths[i]

            for j in range(len(path[0])):
                circle_locations = np.zeros((len(self._circle_offsets), 2))

                circle_locations[:, 0] = path[0][j] + self._circle_offsets * cos(path[2][j])
                circle_locations[:, 1] = path[1][j] + self._circle_offsets * sin(path[2][j])

                for k in range(len(obstacles)):
                    collision_dists = scipy.spatial.distance.cdist(obstacles[k], circle_locations)
                    collision_dists = np.subtract(collision_dists, self._circle_radii)
                    collision_free = collision_free and not np.any(collision_dists < 0)
                    if not collision_free:
                        break
                if not collision_free:
                    break

            collision_check_array[i] = collision_free

        return collision_check_array

    def select_best_path_index(self, paths, collision_check_array, goal_state):
        best_index = None
        best_score = float('Inf')
        for i in range(len(paths)):
            if collision_check_array[i]:
                # Distance from the last path point to the goal state (centerline)
                score = np.linalg.norm([
                    paths[i][0][-1] - goal_state[0],
                    paths[i][1][-1] - goal_state[1]
                ])

                # Penalize proximity to colliding paths
                for j in range(len(paths)):
                    if j == i:
                        continue
                    if not collision_check_array[j]:
                        score += self._weight * (1.0 / (np.linalg.norm([
                            paths[i][0][-1] - paths[j][0][-1],
                            paths[i][1][-1] - paths[j][1][-1]
                        ]) + 1e-5))
            else:
                score = float('Inf')

            if score < best_score:
                best_score = score
                best_index = i

        return best_index

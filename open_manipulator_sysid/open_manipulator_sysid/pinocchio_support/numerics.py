"""Shared numerical helpers for regressor conditioning."""

from __future__ import annotations

from typing import Tuple

import numpy as np

SVD_RANK_RTOL = 1e-6


def effective_condition_number(matrix: np.ndarray) -> Tuple[int, float]:
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    if singular_values.size == 0:
        return 0, float('inf')
    threshold = SVD_RANK_RTOL * singular_values[0]
    rank = int(np.sum(singular_values > threshold))
    if rank == 0:
        return 0, float('inf')
    return rank, float(singular_values[0] / singular_values[rank - 1])

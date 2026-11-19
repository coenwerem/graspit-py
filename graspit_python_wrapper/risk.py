"""Tail-risk measures over sampled outcomes.

`cvar` is the exact upper-tail mean of a cost distribution given by weighted
samples: the atom that straddles the tail boundary contributes only its
fractional mass, so the value is continuous in the tail mass and needs no
rounding of the sample count.
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np


def cvar(costs: Sequence[float], *, alpha: float,
         weights: Optional[Sequence[float]] = None) -> float:
    """Mean of the worst (largest) ``alpha`` probability mass of ``costs``.

    ``alpha`` in (0, 1] is the upper-tail mass; ``alpha = 1`` gives the
    weighted mean. For a quality q to be maximized, the lower-tail mean is
    ``-cvar(-q, alpha=alpha)``.
    """
    if not 0.0 < alpha <= 1.0:
        raise ValueError('alpha must be in (0, 1]')
    x = np.asarray(costs, dtype=float)
    if x.ndim != 1 or x.size == 0 or not np.all(np.isfinite(x)):
        raise ValueError('costs must be a finite, non-empty 1D array')
    w = np.ones(x.size) if weights is None else np.asarray(weights, dtype=float)
    if w.shape != x.shape or not np.all(np.isfinite(w)) or np.any(w < 0) or w.sum() <= 0:
        raise ValueError('weights must be finite, nonnegative and have positive sum')
    order = np.argsort(-x, kind='stable')
    w = w[order] / w.sum()
    mass = np.minimum(w, np.maximum(0.0, alpha - (np.cumsum(w) - w)))
    return float(np.dot(mass, x[order]) / alpha)


def lower_tail_mean(values: Sequence[float], *, alpha: float,
                    weights: Optional[Sequence[float]] = None) -> float:
    """Mean of the lowest ``alpha`` probability mass of ``values``."""
    return -cvar(-np.asarray(values, dtype=float), alpha=alpha, weights=weights)

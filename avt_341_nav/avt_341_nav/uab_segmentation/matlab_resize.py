"""MATLAB-faithful image resizing (antialiased bicubic + nearest) reimplementation in python.

References: MATLAB ``images.internal.resize.contributions`` and the widely-used
``fatheral/matlab_imresize`` port. Border handling uses MATLAB's mirror-reflect indexing.
"""

from __future__ import annotations

import numpy as np

_CUBIC_SUPPORT = 4.0  # bicubic kernel support (covers [-2, 2])


def _cubic(x: np.ndarray) -> np.ndarray:
    """Keys bicubic kernel (a = -0.5), matching MATLAB's default ``cubic``."""
    x = np.asarray(x, dtype=np.float64)
    absx = np.abs(x)
    absx2 = absx * absx
    absx3 = absx2 * absx
    return (
        (1.5 * absx3 - 2.5 * absx2 + 1.0) * (absx <= 1.0)
        + (-0.5 * absx3 + 2.5 * absx2 - 4.0 * absx + 2.0) * ((absx > 1.0) & (absx <= 2.0))
    )


def _contrib_matrix(in_length: int, out_length: int, antialias: bool,
                    kernel=_cubic, kernel_width: float = _CUBIC_SUPPORT) -> np.ndarray:
    """Dense (out_length x in_length) resampling weight matrix for one axis.

    Reproduces MATLAB ``contributions``: subpixel sample centers, antialiased kernel
    stretching on downscale, weight normalization, and mirror-reflect border indexing.
    Dense is fine here (a few hundred rows/cols) and lets us resize with a tensordot.
    """
    scale = float(out_length) / float(in_length)

    if antialias and scale < 1.0:
        h = lambda t: scale * kernel(scale * t)  # noqa: E731
        width = kernel_width / scale
    else:
        h = kernel
        width = kernel_width

    out_coords = np.arange(1, out_length + 1, dtype=np.float64)          # 1-based
    u = out_coords / scale + 0.5 * (1.0 - 1.0 / scale)                   # source centers (1-based)
    left = np.floor(u - width / 2.0)
    p = int(np.ceil(width)) + 2
    cols = left[:, None] + np.arange(p)[None, :]                         # (out, p) 1-based source idx
    weights = h(u[:, None] - cols)                                       # (out, p)
    weights = weights / weights.sum(axis=1, keepdims=True)

    # Mirror-reflect out-of-range indices, exactly like MATLAB's aux = [1:n, n:-1:1].
    aux = np.concatenate([np.arange(in_length), np.arange(in_length - 1, -1, -1)])
    idx0 = (cols - 1).astype(np.int64)                                  # 0-based (may be < 0)
    idx0 = aux[np.mod(idx0, aux.size)]                                  # -> valid 0-based

    weight_mat = np.zeros((out_length, in_length), dtype=np.float64)
    rows = np.repeat(np.arange(out_length), p)
    np.add.at(weight_mat, (rows, idx0.ravel()), weights.ravel())
    return weight_mat


def _to_hwc(img):
    if img.ndim == 2:
        return img[:, :, None], True
    return img, False


def imresize_bicubic(img: np.ndarray, out_hw) -> np.ndarray:
    """Antialiased bicubic resize matching MATLAB ``imresize(img,out_hw,"bicubic")``.

    ``out_hw`` is ``(out_height, out_width)``. uint8 input -> rounded/clipped uint8 output
    (as MATLAB does); float input -> float output.
    """
    arr, squeezed = _to_hwc(img)
    in_h, in_w = arr.shape[0], arr.shape[1]
    out_h, out_w = int(out_hw[0]), int(out_hw[1])
    f = arr.astype(np.float64)

    if (out_h, out_w) != (in_h, in_w):
        w_h = _contrib_matrix(in_h, out_h, antialias=True)   # (out_h, in_h)
        w_w = _contrib_matrix(in_w, out_w, antialias=True)   # (out_w, in_w)
        f = np.tensordot(w_h, f, axes=([1], [0]))            # (out_h, in_w, C)
        f = np.tensordot(f, w_w, axes=([1], [1]))            # (out_h, C, out_w)
        f = np.swapaxes(f, 1, 2)                             # (out_h, out_w, C)

    if np.issubdtype(img.dtype, np.integer):
        info = np.iinfo(img.dtype)
        f = np.clip(np.round(f), info.min, info.max).astype(img.dtype)
    else:
        f = f.astype(img.dtype, copy=False)
    return f[:, :, 0] if squeezed else f


def imresize_nearest(img: np.ndarray, out_hw) -> np.ndarray:
    """Nearest-neighbor resize matching MATLAB ``imresize(img,out_hw,"nearest")``.

    Used for the label/colormap upscale back to the original resolution.
    """
    arr, squeezed = _to_hwc(img)
    in_h, in_w = arr.shape[0], arr.shape[1]
    out_h, out_w = int(out_hw[0]), int(out_hw[1])

    def src_index(in_len, out_len):
        scale = float(out_len) / float(in_len)
        x = np.arange(1, out_len + 1, dtype=np.float64)
        u = x / scale + 0.5 * (1.0 - 1.0 / scale)           # source centers (1-based)
        idx = np.floor(u + 0.5) - 1.0                        # MATLAB box kernel -> 0-based
        return np.clip(idx, 0, in_len - 1).astype(np.int64)

    ri = src_index(in_h, out_h)
    ci = src_index(in_w, out_w)
    out = arr[np.ix_(ri, ci, np.arange(arr.shape[2]))]
    return out[:, :, 0] if squeezed else out

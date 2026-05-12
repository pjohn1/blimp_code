#!/usr/bin/env python3
"""
Plot rolling mean of the EKF innovation chi-squared statistic vs χ² reference bands.

Each per-step statistic is nu' S^{-1} nu; under ideal linear-Gaussian matching it is
chi-squared with m = dim(nu) degrees of freedom. The running average uses the last N_ts
statistics; if those were i.i.d. chi-squared(m), then N_ts * mean ~ chi-squared(N_ts * m),
so two-sided (1 - alpha) prediction limits for the mean are::

    [ chi2.ppf(alpha/2, N_ts*m) / N_ts , chi2.ppf(1-alpha/2, N_ts*m) / N_ts ]

Correlation between consecutive EKF innovations makes these bands approximate in practice.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "matplotlib is required for this script. Install with: pip install matplotlib"
    ) from exc

from scipy.stats import chi2


def mean_chi2_bounds(n_ts: int, dof: int, alpha: float) -> tuple[float, float]:
    """
    Central (1-alpha) interval for the mean of N_ts i.i.d. chi-squared(dof) variables.

    Equivalent: if S ~ chi2(dof * N_ts), bounds for S / N_ts.
    """
    if n_ts < 1:
        raise ValueError("n_ts must be >= 1")
    if dof < 1:
        raise ValueError("dof must be >= 1")
    if not (0.0 < alpha < 1.0):
        raise ValueError("alpha must be in (0, 1)")

    df_sum = n_ts * dof
    lo = chi2.ppf(alpha / 2.0, df_sum) / float(n_ts)
    hi = chi2.ppf(1.0 - alpha / 2.0, df_sum) / float(n_ts)
    return lo, hi


def plot_avg_test_statistic(
    avg: np.ndarray,
    n_ts: int,
    dof: int,
    alpha: float,
    *,
    title: str | None = None,
    save_path: str | None = None,
    show: bool = True,
) -> None:
    avg = np.asarray(avg, dtype=float).ravel()
    if avg.size == 0:
        raise ValueError("avg_test_statistics array is empty.")

    lo, hi = mean_chi2_bounds(n_ts, dof, alpha)
    # x-axis: rolling average index (one point per EKF update after warm-up)
    t = np.arange(avg.size)

    fig, ax = plt.subplots(figsize=(9, 5), layout="constrained")
    ax.axhspan(lo, hi, color="tab:green", alpha=0.2, label=r"reference $(1-\alpha)$ band (i.i.d. χ²)")
    ax.axhline(dof, color="tab:gray", linestyle=":", linewidth=1.0, label=f"E[χ²] = {dof} per step")
    ax.plot(t, avg, color="tab:blue", linewidth=1.2, label="rolling mean test statistic")

    ax.set_xlabel("rolling average index")
    ax.set_ylabel("average innovation χ² statistic")
    ax.set_title(
        title
        or rf"Average test statistic vs χ² band (N_ts={n_ts}, dof={dof}, α={alpha:g})"
    )
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    caption = (
        rf"Band assumes N_ts i.i.d. χ²({dof}) summands; Σ ~ χ²({n_ts * dof}), "
        rf"mean ∈ [{lo:.3f}, {hi:.3f}]. EKF innovations are correlated so this is approximate."
    )
    fig.text(0.02, 0.02, caption, fontsize=8, color="0.35")

    if save_path:
        os.makedirs(os.path.dirname(os.path.abspath(save_path)) or ".", exist_ok=True)
        fig.savefig(save_path, dpi=150)
        print(f"Saved figure to {save_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Plot rolling average EKF innovation statistic with χ²-derived bounds."
    )
    parser.add_argument(
        "npy_path",
        nargs="?",
        default="/home/c3/blimp_code/blimp_clean/blimp_clean/agent_0/avg_test_statistics.npy",
        help="Path to avg_test_statistics.npy (saved by param_estimation)",
    )
    parser.add_argument(
        "--n-ts",
        type=int,
        default=10,
        help="Rolling window length N_ts (must match param_estimation)",
    )
    parser.add_argument(
        "--dof",
        type=int,
        default=2,
        help="Innovation dimension m (1 if only z measured, 2 if z and v)",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.05,
        help="Two-sided level; central (1-alpha) band for the mean under i.i.d. χ²(m)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open a window (use with --output)",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Save figure to this path (png/pdf)",
    )
    args = parser.parse_args(argv)

    path = args.npy_path
    if not os.path.isfile(path):
        print(f"Error: file not found: {path}", file=sys.stderr)
        return 1

    avg = np.load(path)
    plot_avg_test_statistic(
        avg,
        n_ts=args.n_ts,
        dof=args.dof,
        alpha=args.alpha,
        save_path=args.output,
        show=not args.no_show,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

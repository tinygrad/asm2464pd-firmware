#!/usr/bin/env python3
"""
Stability fuzz of the GPU through the USB->ASM->PCIe path via tinygrad.

Randomized workloads checking correctness against CPU reference:
  - random-size copyin/copyout integrity (stresses bulk + SDMA + PCIe)
  - matmuls with random shapes
  - reductions
  - mixed chains of elementwise ops

Runs for a fixed number of iterations (default 200) or a time budget in
seconds with --seconds N (stops cleanly when the budget is exhausted).

Usage:
  DEV=USB+AMD PYTHONPATH=. python3 pcie/gpu_fuzz.py [iterations] [seed]
  DEV=USB+AMD PYTHONPATH=. python3 pcie/gpu_fuzz.py --seconds 15
"""
import os
import sys
import time
import random

import numpy as np

from tinygrad import Tensor

failures = 0


def check(name, got, want, tol=1e-3):
    global failures
    if got.shape != want.shape or not np.allclose(got, want, atol=tol, rtol=tol, equal_nan=True):
        err = np.abs(got - want).max() if got.shape == want.shape else float("nan")
        print(f"FAIL {name}: shape {got.shape} vs {want.shape}, max err {err}")
        failures += 1


def fuzz_copyio(it):
    """Random size copyin -> identity -> copyout, byte-level integrity."""
    n = random.choice([1, 3, 100, 4096, 65536, 1 << 20, 4 << 20, 16 << 20])
    x = np.random.randint(-100, 100, size=n).astype(np.int32)
    y = Tensor(x).numpy()
    check(f"iter{it} copyio int32[{n}]", y, x, tol=0)


def fuzz_matmul(it):
    M = random.choice([1, 7, 32, 128, 256, 512])
    K = random.choice([1, 7, 64, 256, 384])
    N = random.choice([1, 7, 32, 128, 256, 512])
    a = np.random.randn(M, K).astype(np.float32)
    b = np.random.randn(K, N).astype(np.float32)
    c = (Tensor(a) @ Tensor(b)).numpy()
    check(f"iter{it} matmul {M}x{K}x{N}", c, a @ b, tol=1e-2)


def fuzz_reduce(it):
    n = random.choice([1000, 65536, 1 << 20, 8 << 20])
    x = np.random.randn(n).astype(np.float32)
    s = Tensor(x).sum().numpy()
    check(f"iter{it} sum[{n}]", np.array([s]), np.array([x.sum()], dtype=np.float64), tol=1e-1)


def fuzz_chain(it):
    n = random.choice([1024, 65536, 1 << 20])
    x = np.random.randn(n).astype(np.float32)
    k1, k2 = random.uniform(-3, 3), random.uniform(-3, 3)
    y = ((Tensor(x) * k1) + k2).relu().numpy()
    check(f"iter{it} chain[{n}]", y, np.maximum(x * k1 + k2, 0), tol=1e-4)


FUZZERS = [fuzz_copyio, fuzz_matmul, fuzz_reduce, fuzz_chain]


def main():
    seconds = None
    args = sys.argv[1:]
    if args and args[0] == "--seconds":
        seconds = float(args[1])
        iters = 1 << 30
        seed = int(args[2]) if len(args) > 2 else int(time.time())
    else:
        iters = int(args[0]) if args else 200
        seed = int(args[1]) if len(args) > 1 else int(time.time())
    random.seed(seed)
    np.random.seed(seed % (2**32))
    global failures
    print(f"GPU fuzz: {'%ss' % seconds if seconds else iters} iterations max, seed {seed}")
    t0 = time.monotonic()
    ops = 0
    it = 0
    while it < iters:
        if seconds is not None and time.monotonic() - t0 >= seconds:
            break
        fn = random.choice(FUZZERS)
        try:
            fn(it)
        except Exception as e:
            print(f"iter{it} {fn.__name__}: EXCEPTION {type(e).__name__}: {e}")
            failures += 1
        ops += 1
        it += 1
        if it % 20 == 0:
            dt = time.monotonic() - t0
            print(f"  [{it}] {dt:.0f}s elapsed, {ops} ops, {failures} failures")
        if failures >= 5:
            print("too many failures, aborting")
            break
    dt = time.monotonic() - t0
    print(f"DONE: {ops} ops in {dt:.0f}s, {failures} failures")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

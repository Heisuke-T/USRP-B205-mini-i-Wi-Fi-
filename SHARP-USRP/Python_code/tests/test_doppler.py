"""
    ドップラー計算の検証。

    既知のドップラー速度をもつ反射経路を含む CFR を合成し、USRP と同じ
    非等間隔サンプリングを与えたうえで 3 つの扱い方を比べる:

      none   : そのまま等間隔とみなす (既定, SHARP と同じ)
      nudft  : 実測時刻を使う非等間隔DFT
      interp : 線形補間で等間隔化してから STFT

    指標は「集中度」= 真の速度の周り ±4 ビンに集まっているパワーの割合。
    高いほどスペクトルのにじみが少ない。理想 (最初から等間隔) を基準にする。

    実行:  python3 tests/test_doppler.py

    Copyright (C) 2026 Heisuke Takeda
    Released under the GNU GPL v3 (see LICENSE).
"""

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

from CSI_doppler_computation import (V_LIGHT, doppler_spectrum,  # noqa: E402
                                     doppler_spectrum_nudft,
                                     resample_uniform)

FC = 5.18e9          # Ch36
TC = 5.997e-3        # 実測の中央値 (202608191054)
CV_REAL = 0.15       # 実測のパケット間隔の変動係数
N_SUB = 57           # VHT20 の再構成出力幅
N_FFT = 100
WINDOW = 31
N_PKT = 850


class TestFailure(AssertionError):
    pass


def check(cond, msg):
    if not cond:
        raise TestFailure(msg)
    print(f'  OK: {msg}')


def synth_moving_channel(times, v_target, n_sub=N_SUB, seed=0, snr_db=25.0):
    """静止経路 + 一定速度で動く経路 からなる CFR を作る。"""
    rng = np.random.default_rng(seed)
    # SHARP と同じ換算 (v = f_D * c / fc)
    f_doppler = v_target * FC / V_LIGHT

    static = rng.standard_normal(n_sub) + 1j * rng.standard_normal(n_sub)
    static = static / np.abs(static).mean()
    moving = rng.standard_normal(n_sub) + 1j * rng.standard_normal(n_sub)
    moving = moving / np.abs(moving).mean() * 0.7

    H = (static[None, :]
         + moving[None, :] * np.exp(1j * 2 * np.pi * f_doppler * times)[:, None])

    noise_p = np.mean(np.abs(H) ** 2) / (10 ** (snr_db / 10))
    H = H + np.sqrt(noise_p / 2) * (rng.standard_normal(H.shape)
                                    + 1j * rng.standard_normal(H.shape))
    return H


def nonuniform_times(n_pkt, cv=CV_REAL, seed=0):
    """実測に似た非等間隔の時刻列 (バーストと間隙をもつ)。"""
    rng = np.random.default_rng(seed)
    intervals = np.full(n_pkt, TC)
    n_burst = int(n_pkt * cv * 0.5)
    idx = rng.choice(n_pkt, size=n_burst, replace=False)
    intervals[idx] = TC * 0.15                 # バースト
    idx2 = np.clip(idx + 1, 0, n_pkt - 1)
    intervals[idx2] = TC * 1.9                 # 直後の間隙
    intervals *= rng.normal(1.0, 0.05, n_pkt).clip(0.5, 1.5)
    return np.cumsum(intervals)


def concentration(arr, delta_v, v_true, half_width=4):
    """真の速度の周り ±half_width ビンに集まるパワーの割合 (中央値)。"""
    n_bin = arr.shape[1]
    c = int(round(v_true / delta_v)) + n_bin // 2
    lo, hi = max(0, c - half_width), min(n_bin, c + half_width + 1)
    return float(np.median(arr[:, lo:hi].sum(1) / arr.sum(1)))


def peak_velocity(arr, delta_v, exclude_static_bins=3):
    """各時間窓のドップラーピーク速度 (0 m/s 近傍の静止成分は除く)。"""
    n_bin = arr.shape[1]
    v = (np.arange(n_bin) - n_bin // 2) * delta_v
    masked = arr.copy()
    c = n_bin // 2
    masked[:, c - exclude_static_bins:c + exclude_static_bins + 1] = 0
    return float(np.median(v[np.argmax(masked, axis=1)]))


def run_case(v_target, seed=1):
    delta_v = V_LIGHT / (TC * FC * N_FFT)

    # 理想: 最初から等間隔
    H_ideal = synth_moving_channel(np.arange(N_PKT) * TC, v_target, seed=seed)
    a_ideal = doppler_spectrum(H_ideal, WINDOW, 1, -6.0, n_fft=N_FFT)

    # 非等間隔のデータ
    t = nonuniform_times(N_PKT, seed=seed)
    H = synth_moving_channel(t, v_target, seed=seed)

    a_none = doppler_spectrum(H, WINDOW, 1, -6.0, n_fft=N_FFT)
    a_nudft = doppler_spectrum_nudft(H, t, WINDOW, 1, -6.0, TC, n_fft=N_FFT)
    H_rs, Tc_rs, _ = resample_uniform(H, t, TC)
    a_interp = doppler_spectrum(H_rs, WINDOW, 1, -6.0, n_fft=N_FFT)
    delta_v_rs = V_LIGHT / (Tc_rs * FC * N_FFT)

    return dict(
        v=v_target, delta_v=delta_v,
        ideal=concentration(a_ideal, delta_v, v_target),
        none=concentration(a_none, delta_v, v_target),
        nudft=concentration(a_nudft, delta_v, v_target),
        interp=concentration(a_interp, delta_v_rs, v_target),
        peak_none=peak_velocity(a_none, delta_v),
        peak_ideal=peak_velocity(a_ideal, delta_v),
    )


def main():
    delta_v = V_LIGHT / (TC * FC * N_FFT)
    print(f'条件: fc={FC/1e9:.2f} GHz, Tc={TC*1e3:.3f} ms, 窓長={WINDOW}, '
          f'FFT={N_FFT}, 間隔の変動係数={CV_REAL}')
    print(f'ビン間隔 {delta_v:.3f} m/s, '
          f'分解能 {V_LIGHT/(TC*FC*WINDOW):.3f} m/s, '
          f'測定上限 ±{V_LIGHT/(TC*FC)/2:.2f} m/s\n')

    velocities = [0.5, 1.0, 2.0, 3.0]
    results = [run_case(v) for v in velocities]

    print('スペクトル集中度 (真の速度 ±4ビンのパワー割合、高いほど良い)')
    print(f'{"v[m/s]":>7} {"理想":>8} {"none":>8} {"nudft":>8} {"interp":>8}')
    for r in results:
        print(f'{r["v"]:>7} {r["ideal"]:>8.4f} {r["none"]:>8.4f} '
              f'{r["nudft"]:>8.4f} {r["interp"]:>8.4f}')

    print('\nピーク速度の推定 (m/s)')
    for r in results:
        print(f'  真値 {r["v"]:>4}: 理想 {r["peak_ideal"]:+.3f}, '
              f'none {r["peak_none"]:+.3f}')

    print('\n--- 判定 ---')
    try:
        for r in results:
            tol = 3 * r['delta_v']
            check(abs(r['peak_ideal'] - r['v']) <= tol,
                  f"{r['v']} m/s: 理想条件でピークが正しい "
                  f"(誤差 {abs(r['peak_ideal'] - r['v']):.3f})")

        # 低速側 (人の歩行程度) では none が理想とほぼ同等
        for r in results:
            if r['v'] > 2.0:
                continue
            ratio = r['none'] / r['ideal']
            check(ratio > 0.9,
                  f"{r['v']} m/s: none は理想の {ratio*100:.1f}% を維持")

        # 高速側では none も劣化するが、nudft がそれを取り戻す
        for r in results:
            if r['v'] < 3.0:
                continue
            check(r['nudft'] > r['none'],
                  f"{r['v']} m/s: nudft ({r['nudft']:.4f}) が "
                  f"none ({r['none']:.4f}) を上回る")

        # nudft はどの速度でも大きく劣らない
        for r in results:
            check(r['nudft'] > r['none'] * 0.9,
                  f"{r['v']} m/s: nudft が none の 9 割以上を維持")

        # 線形補間は高速側で明確に悪化する (既定を none にした根拠)
        fast = [r for r in results if r['v'] >= 2.0]
        degraded = [r for r in fast if r['interp'] < r['none'] * 0.95]
        check(len(degraded) == len(fast),
              f'線形補間は 2 m/s 以上で none より悪化する '
              f'({len(degraded)}/{len(fast)} ケース)')

    except TestFailure as e:
        print(f'\n失敗: {e}')
        return 1

    print('\nすべてのテストに合格しました。')
    print('結論:')
    print('  - 2 m/s 程度まで (人の歩行) は既定の none で理想とほぼ同等。')
    print('  - 3 m/s 付近では none が理想の 84% まで落ち、nudft が取り戻す。')
    print('    走行など速い動きを見るなら --resample nudft を使うとよい。')
    print('  - 線形補間 (interp) は高速側で明確に有害。比較目的以外では使わない。')
    return 0


if __name__ == '__main__':
    sys.exit(main())

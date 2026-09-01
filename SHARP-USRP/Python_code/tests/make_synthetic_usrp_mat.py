"""
    合成データ生成: USRP デコーダ (decodeIQ_VHT.m / decodeIQ_HE.m) と
    同じ変数構成の .mat を作る。

    既知の多重波チャネルに、パケットごとのランダムな位相・時刻オフセットを
    載せる。USRP は AP と位相同期していないため、実測データでも同種の
    オフセットが必ず乗る。位相サニタイゼーションはこれを除去するのが目的で
    あり、合成データではその除去量を厳密に確認できる。

    Copyright (C) 2026 Heisuke Takeda
    Released under the GNU GPL v3 (see LICENSE).
"""

import argparse
import os
import sys

import numpy as np
import scipy.io as sio

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from wifi_config import get_config  # noqa: E402


def synth_csi(cfg, n_pkt=120, n_paths=3, seed=0, add_offsets=True,
              snr_db=30.0):
    """[パケット x 使用サブキャリア] の複素 CSI を生成する。"""
    rng = np.random.default_rng(seed)

    k = cfg.occupied.astype(float)
    f = k * cfg.delta_f                       # 各サブキャリアの周波数 [Hz]

    # --- 静的な多重波チャネル (全パケットで共通) ---
    #     遅延は 20MHz の分解能 (50ns) より広く散らす
    delays = np.array([0.0, 8e-8, 1.8e-7])[:n_paths]
    gains = np.array([1.0, 0.45, 0.2])[:n_paths] * np.exp(
        1j * rng.uniform(0, 2 * np.pi, n_paths))

    H0 = np.zeros(k.size, dtype=complex)
    for g, tau in zip(gains, delays):
        H0 += g * np.exp(-1j * 2 * np.pi * f * tau)

    H = np.tile(H0, (n_pkt, 1))

    if add_offsets:
        # パケットごとのランダムな共通位相 (位相同期がないことによる)
        phi = rng.uniform(-np.pi, np.pi, (n_pkt, 1))
        # パケットごとのランダムな時刻オフセット (パケット検出位置のずれ)
        # → 周波数方向の位相傾斜として現れる
        tau_off = rng.uniform(-1e-7, 1e-7, (n_pkt, 1))
        H = H * np.exp(1j * phi) * np.exp(-1j * 2 * np.pi * f[None, :] * tau_off)

    # 雑音
    sig_p = np.mean(np.abs(H) ** 2)
    noise_p = sig_p / (10 ** (snr_db / 10))
    H = H + np.sqrt(noise_p / 2) * (rng.standard_normal(H.shape)
                                    + 1j * rng.standard_normal(H.shape))
    return H


def write_mat(out_path, cfg, csi, packet_rate=170.0, seed=0):
    """USRP デコーダと同じ変数名で .mat を書き出す。"""
    rng = np.random.default_rng(seed + 1)
    n_pkt = csi.shape[0]
    # 実測同様に間隔を揺らす
    intervals = np.abs(rng.normal(1.0 / packet_rate, 0.3 / packet_rate, n_pkt))
    time_sec = np.cumsum(intervals)

    suffix = {'VHT': 'VHT', 'HE': 'HE'}[cfg.standard]
    sub_var = {'VHT': 'subcarrierIndicesVHT20',
               'HE': 'subcarrierIndicesHE20'}[cfg.standard]

    mdic = {
        'csi' + suffix: csi,
        sub_var: cfg.occupied.astype(np.float64).reshape(1, -1),
        'timeSec' + suffix: time_sec.reshape(1, -1),
        'fcs' + suffix: np.ones((1, n_pkt), dtype=np.uint8),
        'csi': csi,
        'subcarrierIndices': cfg.occupied.astype(np.float64).reshape(1, -1),
        'timeSec': time_sec.reshape(1, -1),
        'csiMeta': {
            'centerFrequency': 5.18e9,
            'sampleRate': 20e6,
            'gain': 40.0,
            'wifiChannel': 36.0,
            'primaryFormat': cfg.standard,
            'platform': 'B205mini (synthetic)',
            'captureDatetime': '000000000000',
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    sio.savemat(out_path, mdic)
    return time_sec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', choices=['VHT20', 'HE20'])
    parser.add_argument('output')
    parser.add_argument('--n_pkt', type=int, default=120)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--snr_db', type=float, default=30.0)
    parser.add_argument('--no_offsets', action='store_true',
                        help='パケットごとのオフセットを載せない')
    args = parser.parse_args()

    cfg = get_config(args.config)
    csi = synth_csi(cfg, n_pkt=args.n_pkt, seed=args.seed,
                    add_offsets=not args.no_offsets, snr_db=args.snr_db)
    write_mat(args.output, cfg, csi, seed=args.seed)
    print(f'{cfg.name}: {csi.shape} -> {args.output}')


if __name__ == '__main__':
    main()

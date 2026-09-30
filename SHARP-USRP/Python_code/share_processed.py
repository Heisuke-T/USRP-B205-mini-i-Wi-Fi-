"""
    位相サニタイゼーション結果 (processed_phase/*.mat) を共有用に縮小する。

    手順3 の出力は float64 で、30 秒のキャプチャでも 1 ファイル 20 MB 近くになる。
    パケット数の多い測定では 100 MB を超え、GitHub に置けない。

    振幅と位相に double の精度は不要なので single に落とす。これだけで半分に
    なり、ドップラー計算の結果は実質変わらない (差は single の丸め誤差のみ)。
    必要ならパケット数も絞れる。

    用途: 設定を変えたドップラー計算を他の人 (や別環境) に試してもらうとき、
    50MB を超える生データや 20MB の float64 を渡さずに済ませる。

    例:
      python3 share_processed.py ./processed_phase/ ./processed_share/
      python3 share_processed.py ./processed_phase/ ./processed_share/ --max_packets 10000

    Copyright (C) 2026 Heisuke Takeda
    Released under the GNU GPL v3 (see LICENSE).
"""

import argparse
import os
from os import listdir, path

import numpy as np
import scipy.io as sio


def shrink(in_path, out_path, max_packets=None, verbose=True):
    src = sio.loadmat(in_path)
    a = src['csi_matrix_processed']
    n_before = a.shape[0]

    if max_packets is not None and a.shape[0] > max_packets:
        a = a[:max_packets]

    out = {'csi_matrix_processed': a.astype(np.float32)}

    # 後段が実サンプリング間隔を使えるよう、時刻も同じ長さに切って引き継ぐ
    if 'time_sec' in src:
        t = np.asarray(src['time_sec'], dtype=float).ravel()
        if t.size >= a.shape[0]:
            out['time_sec'] = t[:a.shape[0]].astype(np.float64)

    os.makedirs(path.dirname(path.abspath(out_path)), exist_ok=True)
    sio.savemat(out_path, out, do_compression=True)

    if verbose:
        b0 = path.getsize(in_path) / 1e6
        b1 = path.getsize(out_path) / 1e6
        print(f'  {path.basename(in_path)}: {n_before} -> {a.shape[0]} パケット, '
              f'{b0:.1f} -> {b1:.1f} MB ({100*(1-b1/b0):.0f}% 削減)')
    return path.getsize(out_path)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('in_dir', help='processed_phase のディレクトリ')
    parser.add_argument('out_dir', help='縮小版の出力先')
    parser.add_argument('--max_packets', type=int, default=None,
                        help='残す最大パケット数 (既定: 全部)')
    args = parser.parse_args()

    names = sorted(f for f in listdir(args.in_dir) if f.endswith('.mat'))
    if not names:
        print(f'{args.in_dir}: .mat がありません')
        return

    print(f'{len(names)} ファイルを縮小します')
    total = 0
    for n in names:
        total += shrink(path.join(args.in_dir, n),
                        path.join(args.out_dir, n),
                        max_packets=args.max_packets)
    print(f'\n合計 {total/1e6:.1f} MB -> {args.out_dir}')
    if total > 90e6:
        print('注意: 合計が大きいため、ファイルを分けて push するか '
              '--max_packets で絞ってください。')


if __name__ == '__main__':
    main()

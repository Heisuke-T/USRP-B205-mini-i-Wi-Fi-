"""
    ドップラーマップの描画

    CSI_doppler_computation.py が出力した .txt (pickle) と _meta.json を読み、
    横軸を時間、縦軸をドップラー速度としたマップを画像に保存する。

    軸の物理量は _meta.json に記録された Tc / fc / n_fft から決まるため、
    測定ごとに正しい目盛りが付く (SHARP オリジナルは Tc=6ms, fc=5GHz の
    ハードコードだった)。

    Copyright (C) 2026 Heisuke Takeda
    Based on SHARP (C) 2022 Francesca Meneghello, GNU GPL v3.
    Released under the GNU GPL v3 (see LICENSE).
"""

import argparse
import json
import os
import pickle
from os import listdir, path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load_doppler(txt_file):
    with open(txt_file, 'rb') as fp:
        arr = pickle.load(fp)
    meta_file = txt_file[:-4] + '_meta.json'
    meta = {}
    if path.exists(meta_file):
        with open(meta_file, encoding='utf-8') as fp:
            meta = json.load(fp)
    return arr, meta


def plot_doppler(arr, meta, out_png, title=None, vmin_db=None):
    """arr: [時間窓 x ドップラービン] (正規化済みパワー)"""
    n_win, n_bin = arr.shape

    delta_v = meta.get('delta_v_bin')
    Tc = meta.get('Tc')
    sliding = meta.get('sliding', 1)

    # 軸ラベルは英語にする。日本語フォントが無い環境で文字化けするため。
    # 縦軸 (ドップラー速度)
    if delta_v is not None:
        v_axis = (np.arange(n_bin) - n_bin // 2) * delta_v
        y_label = 'Doppler velocity [m/s]'
    else:
        v_axis = np.arange(n_bin) - n_bin // 2
        y_label = 'Doppler bin'

    # 横軸 (時間)
    if Tc is not None:
        t_axis = np.arange(n_win) * sliding * Tc
        x_label = 'Time [s]'
    else:
        t_axis = np.arange(n_win)
        x_label = 'Window index'

    power_db = 10 * np.log10(np.maximum(arr, 1e-12))
    if vmin_db is None:
        vmin_db = float(np.percentile(power_db, 5))

    fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
    mesh = ax.pcolormesh(t_axis, v_axis, power_db.T,
                         shading='auto', cmap='viridis',
                         vmin=vmin_db, vmax=0)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.axhline(0.0, color='w', linewidth=0.5, alpha=0.35)
    if title:
        ax.set_title(title)
    cbar = fig.colorbar(mesh, ax=ax)
    cbar.set_label('Normalized power [dB]')

    os.makedirs(path.dirname(path.abspath(out_png)), exist_ok=True)
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    return out_png


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dir', help='ドップラーデータ (.txt) のディレクトリ')
    parser.add_argument('out_dir', help='画像の保存先')
    parser.add_argument('--name', default=None,
                        help='特定のファイルだけ描画する (拡張子なし)')
    parser.add_argument('--vmin_db', type=float, default=None,
                        help='カラースケールの下限 [dB] (既定は5パーセンタイル)')
    args = parser.parse_args()

    if args.name:
        names = [args.name]
    else:
        names = [f[:-4] for f in sorted(listdir(args.dir))
                 if f.endswith('.txt')]
    if not names:
        print(f'{args.dir}: .txt がありません')
        return

    for name in names:
        arr, meta = load_doppler(path.join(args.dir, name + '.txt'))
        out_png = path.join(args.out_dir, name + '.png')
        plot_doppler(arr, meta, out_png, title=name, vmin_db=args.vmin_db)
        print(f'{name}: {arr.shape} -> {out_png}')


if __name__ == '__main__':
    main()

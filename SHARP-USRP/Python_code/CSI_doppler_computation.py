"""
    ドップラースペクトル (ドップラーマップ) の計算

    位相サニタイゼーション済みの CFR に短時間フーリエ変換を掛け、時間 x
    ドップラー速度のマップを得る。アルゴリズムは SHARP オリジナルと同じ
    (ハン窓 -> FFT -> パワー -> サブキャリア方向に加算 -> 正規化と雑音床)。

    SHARP オリジナルからの変更点と、その理由:

      1. --bandwidth / --sub_band による部分帯域の切り出しを廃止した
         オリジナルの --bandwidth 20 は「80MHz で取得したデータから 20MHz
         幅を切り出す」オプションで、245 列 (80MHz 分) の配列を前提とした
         固定インデックスだった。本システムの入力はすでに 20MHz なので、
         これを指定すると HE20 (245 列) では 20MHz のさらに 1/4 を切り出す
         誤りになる (エラーは出ないので気付きにくい)。
         部分帯域を見たい場合は --subcarrier_range で明示する。

      2. Tc (パケット間隔) と fc (中心周波数) をハードコードしない
         オリジナルは Tc=6ms, fc=5GHz 固定。USRP は受動受信のため取得間隔が
         装置ごと・測定ごとに異なる。既定では手順3 が保存した実測の
         パケット時刻から Tc を求める。

      3. 非等間隔サンプリングへの対応 (--resample)
         STFT は等間隔サンプリングを前提とするが、USRP は AP が実際に送信した
         パケットしか捉えられないため間隔が揺れる。実測 (変動係数 0.15) を
         模した合成データで 3 通りを比較した結果:

           速度      理想    none   nudft  interp
           0.5 m/s  0.595   0.592   0.547   0.592
           1.0 m/s  0.330   0.323   0.303   0.309
           2.0 m/s  0.330   0.302   0.302   0.247
           3.0 m/s  0.330   0.277   0.306   0.165
           (数値は真の速度±4ビンへのパワー集中度。高いほどにじみが少ない)

         - 2 m/s 程度まで (人の歩行) は 'none' で理想とほぼ同等
         - 3 m/s 付近では 'none' が理想の 84% まで落ち 'nudft' が取り戻す
         - 線形補間 'interp' は高速側で明確に悪化する (低域通過として働くため)

         よって既定は 'none' (SHARP と同じ扱い)。走行など速い動きを見る場合は
         'nudft' を推奨する。検証は tests/test_doppler.py。

    Copyright (C) 2026 Heisuke Takeda
    Based on SHARP (C) 2022 Francesca Meneghello, GNU GPL v3.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import argparse
import json
import math as mt
import os
import pickle
from os import listdir, path

import numpy as np
import scipy.io as sio
from scipy.fftpack import fft, fftshift
from scipy.signal.windows import hann

V_LIGHT = 3e8


def resample_uniform(csi_complex, time_sec, target_interval=None):
    """複素 CFR を等間隔の時間グリッドへ線形補間する。

    csi_complex: [パケット x サブキャリア]
    time_sec:    [パケット]  実測のパケット時刻 [s]

    戻り値: (補間後の CFR, 実際の間隔 Tc, 補間の統計)
    """
    order = np.argsort(time_sec)
    t = time_sec[order]
    x = csi_complex[order, :]

    # 同時刻のパケットがあると補間できないため除く
    keep = np.concatenate(([True], np.diff(t) > 0))
    t, x = t[keep], x[keep, :]

    dt = np.diff(t)
    if target_interval is None:
        target_interval = float(np.median(dt))

    n_out = int(np.floor((t[-1] - t[0]) / target_interval)) + 1
    t_uniform = t[0] + np.arange(n_out) * target_interval

    # 実部と虚部を別々に補間する (複素数の線形補間と等価)
    out = (np.apply_along_axis(lambda c: np.interp(t_uniform, t, c), 0, x.real)
           + 1j * np.apply_along_axis(lambda c: np.interp(t_uniform, t, c), 0, x.imag))

    stats = dict(
        n_in=int(t.size), n_out=int(n_out),
        target_interval_ms=target_interval * 1e3,
        median_interval_ms=float(np.median(dt)) * 1e3,
        min_interval_ms=float(dt.min()) * 1e3,
        max_interval_ms=float(dt.max()) * 1e3,
        cv=float(dt.std() / dt.mean()),
        max_gap_in_samples=float(dt.max() / target_interval),
    )
    return out, target_interval, stats


def _finalize(profiles, n_pkt, num_symbols, noise_lev):
    if not profiles:
        raise ValueError(
            f'窓長 {num_symbols} に対してパケット数 {n_pkt} が '
            f'足りません。--sample_length を小さくしてください。')
    arr = np.asarray(profiles)
    arr = arr / np.max(arr, axis=1, keepdims=True)
    arr[arr < mt.pow(10, noise_lev)] = mt.pow(10, noise_lev)
    return arr


def doppler_spectrum(csi_complex, num_symbols, sliding, noise_lev, n_fft=100):
    """短時間フーリエ変換でドップラースペクトルを得る (SHARP と同じ手順)。"""
    profiles = []
    hann_window = np.expand_dims(hann(num_symbols), axis=-1)

    for i in range(0, csi_complex.shape[0] - num_symbols, sliding):
        cut = np.nan_to_num(csi_complex[i:i + num_symbols, :])
        wind = np.multiply(cut, hann_window)
        prof = fftshift(fft(wind, n=n_fft, axis=0), axes=0)
        # パワーをサブキャリア方向に加算
        profiles.append(np.sum(np.abs(prof * np.conj(prof)), axis=1))

    return _finalize(profiles, csi_complex.shape[0], num_symbols, noise_lev)


def doppler_spectrum_nudft(csi_complex, time_sec, num_symbols, sliding,
                           noise_lev, Tc, n_fft=100):
    """非等間隔DFT によるドップラースペクトル。

    実測のパケット時刻をそのまま指数の中に入れるため、補間を伴わずに
    非等間隔サンプリングを扱える。周波数グリッドは等間隔 FFT と同じ
    (中心 0、間隔 1/(Tc*n_fft)) に取るので速度軸の意味は変わらない。

    等間隔サンプリングであれば通常の FFT と一致する。
    """
    f_grid = (np.arange(n_fft) - n_fft // 2) / (Tc * n_fft)
    profiles = []

    for i in range(0, csi_complex.shape[0] - num_symbols, sliding):
        x = np.nan_to_num(csi_complex[i:i + num_symbols, :])
        t = time_sec[i:i + num_symbols]
        t = t - t[0]
        span = t[-1] if t[-1] > 0 else 1.0

        # ハン窓は標本番号ではなく時間位置で与える
        w = 0.5 * (1 - np.cos(2 * np.pi * t / span))
        # 各標本が代表する時間幅で重み付けする (標本密度の偏りの補正)
        dt = np.gradient(t)
        w = w * dt / dt.mean()

        prof = np.exp(-1j * 2 * np.pi * np.outer(f_grid, t)) @ (x * w[:, None])
        profiles.append(np.sum(np.abs(prof) ** 2, axis=1))

    return _finalize(profiles, csi_complex.shape[0], num_symbols, noise_lev)


def process_one(mat_file, out_file, args):
    mdic = sio.loadmat(mat_file)
    csi_matrix_processed = mdic['csi_matrix_processed']

    end = csi_matrix_processed.shape[0] - args.end if args.end > 0 else None
    csi_matrix_processed = csi_matrix_processed[args.start:end, :, :]

    time_sec = None
    if 'time_sec' in mdic:
        t = np.asarray(mdic['time_sec'], dtype=float).ravel()
        if t.size >= csi_matrix_processed.shape[0] + args.start:
            time_sec = t[args.start:end]

    # 振幅をパケットごとに正規化してから複素数へ戻す
    amp = csi_matrix_processed[:, :, 0]
    amp = amp / np.mean(amp, axis=1, keepdims=True)
    csi_complex = amp * np.exp(1j * csi_matrix_processed[:, :, 1])

    # 部分帯域の切り出し (既定は全サブキャリア)
    if args.subcarrier_range is not None:
        lo, hi = args.subcarrier_range
        csi_complex = csi_complex[:, lo:hi]

    # --- サンプリング間隔 Tc を決める ---
    resample_stats = None
    Tc = args.Tc
    cv = None
    if time_sec is not None and time_sec.size == csi_complex.shape[0]:
        dt = np.diff(np.sort(time_sec))
        cv = float(dt.std() / dt.mean()) if dt.size else 0.0
        if Tc is None:
            Tc = float(np.median(dt))
        print(f'  パケット間隔: 中央値 {np.median(dt)*1e3:.3f} ms, '
              f'変動係数 {cv:.3f} -> Tc={Tc*1e3:.3f} ms')
        if cv > args.cv_warn_threshold and args.resample == 'none':
            print(f'  注意: 間隔のばらつきが大きめです (変動係数 {cv:.2f})。'
                  f'--resample nudft も試してください。')
    elif Tc is None:
        raise ValueError(
            'Tc を決められません。time_sec を含む .mat を使うか '
            '--Tc で明示してください。')

    # --- ドップラースペクトル ---
    if args.resample == 'nudft':
        if time_sec is None or time_sec.size != csi_complex.shape[0]:
            raise ValueError(f'{mat_file}: time_sec が無いため nudft は使えません。')
        order = np.argsort(time_sec)
        arr = doppler_spectrum_nudft(
            csi_complex[order], time_sec[order], args.sample_length,
            args.sliding, args.noise_level, Tc, n_fft=args.n_fft)
        print('  非等間隔DFT で計算')
    elif args.resample == 'interp':
        if time_sec is None or time_sec.size != csi_complex.shape[0]:
            raise ValueError(f'{mat_file}: time_sec が無いため interp は使えません。')
        print('  警告: 線形補間による等間隔化は高いドップラー速度でスペクトルを'
              'にじませます。比較目的以外では nudft か none を推奨します。')
        csi_complex, Tc, resample_stats = resample_uniform(
            csi_complex, time_sec, args.Tc)
        print(f'  等間隔化: {resample_stats["n_in"]} -> '
              f'{resample_stats["n_out"]} サンプル, Tc={Tc*1e3:.3f} ms')
        arr = doppler_spectrum(csi_complex, args.sample_length, args.sliding,
                               args.noise_level, n_fft=args.n_fft)
    else:
        arr = doppler_spectrum(csi_complex, args.sample_length, args.sliding,
                               args.noise_level, n_fft=args.n_fft)

    # --- 速度軸 ---
    # ビン間隔: FFT のビン数から決まる (ゼロ詰めを含む)
    delta_v_bin = V_LIGHT / (Tc * args.fc * args.n_fft)
    # 分解能: 窓長から決まる (ゼロ詰めでは向上しない)
    delta_v_res = V_LIGHT / (Tc * args.fc * args.sample_length)
    v_max = V_LIGHT / (Tc * args.fc) / 2      # 一意に測れる速度の上限

    os.makedirs(path.dirname(path.abspath(out_file)), exist_ok=True)
    with open(out_file, 'wb') as fp:
        pickle.dump(arr, fp)

    meta = dict(
        source=path.basename(mat_file),
        shape=list(arr.shape),
        Tc=Tc, fc=args.fc, n_fft=args.n_fft,
        sample_length=args.sample_length, sliding=args.sliding,
        noise_level=args.noise_level,
        delta_v_bin=delta_v_bin, delta_v_resolution=delta_v_res,
        v_max=v_max,
        n_subcarriers=int(csi_complex.shape[1]),
        resample_mode=args.resample,
        interval_cv=cv,
        resample=resample_stats,
    )
    with open(out_file[:-4] + '_meta.json', 'w', encoding='utf-8') as fp:
        json.dump(meta, fp, ensure_ascii=False, indent=2)

    print(f'  {arr.shape} (時間窓 x ドップラービン) -> {out_file}')
    print(f'  速度軸: ±{v_max:.2f} m/s, ビン間隔 {delta_v_bin:.3f} m/s, '
          f'分解能 {delta_v_res:.3f} m/s')
    return meta


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dir', help='位相サニタイゼーション結果のディレクトリ')
    parser.add_argument('subdirs', help='サブディレクトリ (カンマ区切り)。'
                                        'サブディレクトリを使っていない場合は "." ')
    parser.add_argument('dir_doppler', help='ドップラーデータの保存先')
    parser.add_argument('start', help='先頭から捨てるパケット数', type=int)
    parser.add_argument('end', help='末尾から捨てるパケット数', type=int)
    parser.add_argument('sample_length', help='1窓あたりのパケット数', type=int)
    parser.add_argument('sliding', help='窓をずらすパケット数', type=int)
    parser.add_argument('noise_level', help='雑音床 (10^x で切り捨て)', type=float)
    parser.add_argument('--fc', type=float, default=5.18e9,
                        help='中心周波数 [Hz] (既定 5.18e9 = Ch36)')
    parser.add_argument('--Tc', type=float, default=None,
                        help='パケット間隔 [s]。既定は time_sec から実測')
    parser.add_argument('--n_fft', type=int, default=100,
                        help='FFT のビン数 (既定 100, SHARP と同じ)')
    parser.add_argument('--resample', choices=['none', 'nudft', 'interp'],
                        default='none',
                        help='非等間隔サンプリングの扱い。'
                             'none: そのまま等間隔とみなす (既定, SHARP と同じ。'
                             '実測の変動係数 0.15 程度なら理想とほぼ同等)。'
                             'nudft: 実測時刻を使う非等間隔DFT (間隔が大きく乱れる場合)。'
                             'interp: 線形補間で等間隔化 (比較用。高速側で悪化する)')
    parser.add_argument('--cv_warn_threshold', type=float, default=0.3,
                        help='この変動係数を超えたら nudft を勧める警告を出す')
    parser.add_argument('--subcarrier_range', type=int, nargs=2, default=None,
                        metavar=('LO', 'HI'),
                        help='使用するサブキャリアの範囲 (既定は全部)')
    args = parser.parse_args()

    for subdir in args.subdirs.split(','):
        exp_dir = path.join(args.dir, subdir) if subdir != '.' else args.dir
        out_dir = (path.join(args.dir_doppler, subdir) if subdir != '.'
                   else args.dir_doppler)
        os.makedirs(out_dir, exist_ok=True)

        names = [f[:-4] for f in sorted(listdir(exp_dir)) if f.endswith('.mat')]
        if not names:
            print(f'{exp_dir}: .mat がありません')
            continue

        for name in names:
            out_file = path.join(out_dir, name + '.txt')
            if path.exists(out_file):
                print(f'{name}: 処理済みのためスキップ')
                continue
            print(name)
            process_one(path.join(exp_dir, name + '.mat'), out_file, args)


if __name__ == '__main__':
    main()

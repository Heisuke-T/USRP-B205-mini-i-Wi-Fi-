"""
    位相サニタイゼーション 手順1: 信号の前処理

    SHARP オリジナル (80MHz / Nexmon / 4コア前提) を、20MHz・SISO・
    USRP B205mini-i のデータに対応させたもの。主な変更点:

      * FFT 長・ヌルサブキャリアを wifi_config で規格ごとに切り替え
      * ストリーム数を n_ss * n_core から決定 (SISO では 1)
      * Nexmon 固有の符号反転 (上半分の符号を反転) を既定で行わない
        → USRP + MATLAB WLAN Toolbox のチャネル推定値には不要なため

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
import os
import pickle
from os import listdir, path

import numpy as np
import scipy.io as sio

from pipeline_meta import save_meta
from wifi_config import get_config


def hampel_filter(input_matrix, window_size, n_sigmas=3):
    """外れ値を移動中央値で置き換える (SHARP 由来、現状は未使用)。"""
    n = input_matrix.shape[1]
    new_matrix = np.zeros_like(input_matrix)
    k = 1.4826  # scale factor for Gaussian distribution

    for ti in range(n):
        start_time = max(0, ti - window_size)
        end_time = min(n, ti + window_size)
        x0 = np.nanmedian(input_matrix[:, start_time:end_time], axis=1, keepdims=True)
        s0 = k * np.nanmedian(np.abs(input_matrix[:, start_time:end_time] - x0), axis=1)
        mask = (np.abs(input_matrix[:, ti] - x0[:, 0]) > n_sigmas * s0)
        new_matrix[:, ti] = mask * x0[:, 0] + (1 - mask) * input_matrix[:, ti]

    return new_matrix


def resolve_config(mat_contents, override):
    """.mat 内の sharp_config を優先しつつ、指定があればそれを使う。"""
    if override:
        return get_config(override)
    if 'sharp_config' in mat_contents:
        name = str(np.asarray(mat_contents['sharp_config']).ravel()[0]).strip()
        return get_config(name)
    raise ValueError(
        'sharp_config が .mat に無いため規格を判定できません。'
        '--config VHT20 のように明示してください。')


def process_file(exp_dir, name, out_dir, n_ss, n_core, start_idx,
                 config_override=None, nexmon_sign_flip=False,
                 normalize=True, overwrite=False):
    out_file = path.join(out_dir, 'signal_' + name + '.txt')
    if path.exists(out_file) and not overwrite:
        print(f'{name}: 処理済みのためスキップ (--overwrite で再処理)')
        return None

    csi_buff_file = path.join(exp_dir, name + '.mat')
    contents = sio.loadmat(csi_buff_file)
    cfg = resolve_config(contents, config_override)

    csi_buff = contents['csi_buff']
    if csi_buff.shape[1] != cfg.fft_size:
        raise ValueError(
            f'{name}: csi_buff の列数 {csi_buff.shape[1]} が '
            f'{cfg.name} の FFT 長 {cfg.fft_size} と一致しません。')

    # FFT 順 -> 中心寄せ順 (index i がサブキャリア i - FFT長/2 に対応)
    csi_buff = np.fft.fftshift(csi_buff, axes=1)

    # 全ゼロのパケットを除く
    delete_idxs = np.argwhere(np.sum(np.abs(csi_buff), axis=1) == 0)[:, 0]
    if delete_idxs.size:
        csi_buff = np.delete(csi_buff, delete_idxs, axis=0)
        print(f'  空パケットを {delete_idxs.size} 件除外')

    n_tot = n_ss * n_core
    n_pkt_per_stream = int(np.floor(csi_buff.shape[0] / n_tot))
    if n_pkt_per_stream <= start_idx:
        raise ValueError(
            f'{name}: パケット数 ({csi_buff.shape[0]}) に対して '
            f'start_idx={start_idx} が大きすぎます。')

    n_time = n_pkt_per_stream - start_idx
    signal_complete = np.zeros((cfg.n_occupied, n_time, n_tot), dtype=complex)

    for stream in range(n_tot):
        # 複数ストリームの場合、パケットは行方向に交互配置されている
        signal_stream = csi_buff[stream::n_tot, :][start_idx:n_pkt_per_stream, :]

        if nexmon_sign_flip:
            # Nexmon CSI 固有の符号規約。USRP のデータでは不要。
            signal_stream[:, cfg.half:] = -signal_stream[:, cfg.half:]

        # ヌル (ガードバンド + DC) を除去
        signal_stream = np.delete(signal_stream, cfg.delete_idxs, axis=1)

        if normalize:
            # パケットごとに平均振幅で正規化 (自動利得制御の影響を除く)
            mean_signal = np.mean(np.abs(signal_stream), axis=1, keepdims=True)
            mean_signal[mean_signal == 0] = 1.0
            signal_stream = signal_stream / mean_signal

        signal_complete[:, :, stream] = signal_stream.T

    os.makedirs(out_dir, exist_ok=True)
    with open(out_file, 'wb') as fp:
        pickle.dump(signal_complete, fp)

    # 後段 (H 推定・再構成) が規格を引き継げるよう記録しておく
    save_meta(out_dir, name, config=cfg.name, n_ss=n_ss, n_core=n_core,
              n_tot=n_tot, n_packets=int(n_time),
              n_occupied=int(cfg.n_occupied), start_idx=start_idx,
              source=path.basename(csi_buff_file))

    print(f'{name}: {cfg.name} '
          f'{signal_complete.shape} (サブキャリア x パケット x ストリーム) '
          f'-> {out_file}')
    return signal_complete.shape


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dir', help='入力 .mat があるディレクトリ')
    parser.add_argument('all_dir', help='ディレクトリ内の全ファイルを処理するか (1/0)',
                        type=int, default=0)
    parser.add_argument('name', help='処理するファイル名 (拡張子なし。all_dir=0 のとき)')
    parser.add_argument('nss', help='空間ストリーム数 (SISO なら 1)', type=int)
    parser.add_argument('ncore', help='受信コア数 (アンテナ1本なら 1)', type=int)
    parser.add_argument('start_idx', help='各ストリームで処理を開始するパケット番号',
                        type=int)
    parser.add_argument('--config', default=None,
                        help='規格設定を明示する (VHT20 / HE20)。既定は .mat 内の指定')
    parser.add_argument('--out_dir', default='./phase_processing/',
                        help='中間ファイルの出力先')
    parser.add_argument('--nexmon_sign_flip', action='store_true',
                        help='Nexmon 固有の符号反転を行う (USRP データでは不要)')
    parser.add_argument('--no_normalize', action='store_true',
                        help='パケットごとの振幅正規化を行わない')
    parser.add_argument('--overwrite', action='store_true',
                        help='処理済みでも再処理する')
    args = parser.parse_args()

    names = []
    if args.all_dir:
        for f in sorted(listdir(args.dir)):
            if f.endswith('.mat'):
                names.append(f[:-4])
    else:
        names.append(args.name)

    for name in names:
        process_file(args.dir, name, args.out_dir, args.nss, args.ncore,
                     args.start_idx, config_override=args.config,
                     nexmon_sign_flip=args.nexmon_sign_flip,
                     normalize=not args.no_normalize,
                     overwrite=args.overwrite)


if __name__ == '__main__':
    main()

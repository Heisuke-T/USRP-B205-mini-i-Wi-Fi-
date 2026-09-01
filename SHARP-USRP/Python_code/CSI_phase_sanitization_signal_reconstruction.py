"""
    位相サニタイゼーション 手順3: サニタイズ済み CFR の再構成と保存

    手順2 が出力した Tr (位相基準化済みの周波数応答) から、振幅と
    アンウラップ済み位相を取り出し、パケット間の残留位相傾斜を最小二乗で
    除いて .mat に保存する。

    SHARP オリジナルからの変更点:
      * FFT 長と切り出し範囲を wifi_config で規格ごとに切り替え
        (オリジナルは 256 点・[6:-5] のハードコード)
      * 出力先サブディレクトリをファイル名先頭 3 文字から決めていた挙動を
        --label で明示できるようにした (USRP のファイル名は日時始まりのため)

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
import math as mt
import os
import pickle
from os import listdir, path

import numpy as np
import scipy.io as sio

from pipeline_meta import load_meta
from wifi_config import get_config


def unwrap_and_detrend(H_est):
    """位相のアンウラップと、パケット間の残留位相傾斜の除去。

    H_est: [サブキャリア x パケット] の複素行列 (使用帯域のみ)
    戻り値: 同じ形の実数位相行列
    """
    phase_before = np.unwrap(np.angle(H_est), axis=0)

    ones_vector = np.ones((2, phase_before.shape[0]))
    ones_vector[1, :] = np.arange(0, phase_before.shape[0])

    # 隣接パケット間で 2π 跳びが生じている箇所をそろえる
    for tidx in range(1, phase_before.shape[1]):
        stop = False
        idx_prec = -1
        while not stop:
            phase_err = phase_before[:, tidx] - phase_before[:, tidx - 1]
            diff_phase_err = np.diff(phase_err)
            idxs_invert_up = np.argwhere(diff_phase_err > 0.9 * mt.pi)[:, 0]
            idxs_invert_down = np.argwhere(diff_phase_err < -0.9 * mt.pi)[:, 0]
            if idxs_invert_up.shape[0] > 0:
                idx_act = idxs_invert_up[0]
                if idx_act == idx_prec:  # 同じ位置で跳び続けるのを防ぐ
                    stop = True
                else:
                    phase_before[idx_act + 1:, tidx] -= 2 * mt.pi
                    idx_prec = idx_act
            elif idxs_invert_down.shape[0] > 0:
                idx_act = idxs_invert_down[0]
                if idx_act == idx_prec:
                    stop = True
                else:
                    phase_before[idx_act + 1:, tidx] += 2 * mt.pi
                    idx_prec = idx_act
            else:
                stop = True

    # 直前パケットとの差分を1次式で近似し、その傾斜と定数分を差し引く
    for tidx in range(1, phase_before.shape[1] - 1):
        error = phase_before[:, tidx:tidx + 1] - phase_before[:, tidx - 1:tidx]
        temp2 = np.linalg.lstsq(ones_vector.T, error, rcond=None)[0]
        phase_before[:, tidx] = phase_before[:, tidx] - (np.dot(ones_vector.T, temp2)).T

    return phase_before


def reconstruct_file(tr_name, exp_dir, save_dir, cfg, start_idx, end_idx,
                     label=None, overwrite=False, verbose=True):
    # tr_name は 'Tr_vector_<name>_stream_<s>' 形式
    stem = tr_name[len('Tr_vector_'):]
    out_name = stem + '.mat'

    subdir = save_dir if label is None else path.join(save_dir, label)
    out_path = path.join(subdir, out_name)
    if path.isfile(out_path) and not overwrite:
        print(f'{stem}: 処理済みのためスキップ (--overwrite で再処理)')
        return None

    with open(path.join(exp_dir, tr_name + '.txt'), 'rb') as fp:
        H_est = pickle.load(fp)

    if H_est.shape[0] != cfg.fft_size:
        raise ValueError(
            f'{stem}: Tr のサブキャリア数 {H_est.shape[0]} が '
            f'{cfg.name} の FFT 長 {cfg.fft_size} と一致しません。')

    end_H = H_est.shape[1]
    H_est = H_est[:, start_idx:end_H - end_idx]
    if H_est.shape[1] <= 1:
        raise ValueError(f'{stem}: 切り出し後のパケット数が足りません。')

    crop = cfg.crop_slice
    H_crop = H_est[crop, :]

    n_time = H_crop.shape[1]
    n_sub = H_crop.shape[0]
    csi_matrix_processed = np.zeros((n_time, n_sub, 2))

    # 振幅
    csi_matrix_processed[:, :, 0] = np.abs(H_crop).T
    # 位相
    csi_matrix_processed[:, :, 1] = unwrap_and_detrend(H_crop).T

    os.makedirs(subdir, exist_ok=True)
    sio.savemat(out_path, {'csi_matrix_processed': csi_matrix_processed})

    if verbose:
        print(f'{stem}: {csi_matrix_processed.shape} '
              f'(パケット x サブキャリア x [振幅,位相]) -> {out_path}')
    return csi_matrix_processed.shape


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dir', help='中間ファイル (Tr_vector_*.txt) のディレクトリ')
    parser.add_argument('dir_save', help='再構成結果の保存先ディレクトリ')
    parser.add_argument('nss', help='空間ストリーム数 (SISO なら 1)', type=int)
    parser.add_argument('ncore', help='受信コア数 (アンテナ1本なら 1)', type=int)
    parser.add_argument('start_idx', help='先頭から捨てるパケット数', type=int)
    parser.add_argument('end_idx', help='末尾から捨てるパケット数', type=int)
    parser.add_argument('--config', default=None,
                        help='規格設定 (VHT20 / HE20)。既定は前段の記録を使用')
    parser.add_argument('--label', default=None,
                        help='保存先のサブディレクトリ名 (活動ラベル等)。'
                             '既定はサブディレクトリを作らない')
    parser.add_argument('--overwrite', action='store_true',
                        help='処理済みでも再処理する')
    args = parser.parse_args()

    tr_names = [f[:-4] for f in sorted(listdir(args.dir))
                if f.startswith('Tr_vector_') and f.endswith('.txt')]
    if not tr_names:
        print(f'{args.dir} に Tr_vector_*.txt がありません。'
              f'手順2 を先に実行してください。')
        return

    for tr_name in tr_names:
        stem = tr_name[len('Tr_vector_'):]
        base = stem.rsplit('_stream_', 1)[0]
        cfg_name = args.config or load_meta(args.dir, base).get('config')
        if cfg_name is None:
            raise ValueError(
                f'{stem}: 規格を判定できません。--config を指定してください。')
        reconstruct_file(tr_name, args.dir, args.dir_save,
                         get_config(cfg_name), args.start_idx, args.end_idx,
                         label=args.label, overwrite=args.overwrite)


if __name__ == '__main__':
    main()

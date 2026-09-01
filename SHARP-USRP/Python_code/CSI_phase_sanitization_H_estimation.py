"""
    位相サニタイゼーション 手順2: 多重波の推定と位相の基準化

    観測 CFR を「遅延の異なる複数経路の和」としてスパース推定 (lasso/OSQP)
    し、最強経路の位相を基準に全経路を再合成する。これにより送受信機間の
    時刻・位相オフセット (STO / CFO 残差 / 位相同期のずれ) が相殺され、
    伝搬路そのものに由来する位相のみが残る。
    詳細は Meneghello et al., "SHARP" (IEEE TMC 2022) 3.1 節。

    SHARP オリジナル (80MHz / 4ストリーム固定) からの変更点:
      * FFT 長・Δf・ヌル配置を wifi_config で規格ごとに切り替え
      * ストリーム数を固定 4 から n_ss * n_core に変更 (SISO では 1)
      * サブキャリア間引き幅と精細グリッド刻みを規格既定 + CLI で調整可能に
        20MHz は 80MHz に比べサブキャリア数が少ないため、既定値のままでは
        観測数に対して未知数が多くなりすぎる (README「20MHz での注意」参照)

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
import time
from os import listdir, path

import numpy as np
import scipy.sparse

from optimization_utility import build_T_matrix, lasso_regression_osqp_fast
from pipeline_meta import load_meta, save_meta
from wifi_config import get_config


def _build_aux(row_T, col_T):
    """OSQP に渡す補助行列 (SHARP と同じ構成)。"""
    m = 2 * row_T
    n = 2 * col_T
    In = scipy.sparse.eye(n)
    Im = scipy.sparse.eye(m)
    On = scipy.sparse.csc_matrix((n, n))
    Onm = scipy.sparse.csc_matrix((n, m))
    P = scipy.sparse.block_diag([On, Im, On], format='csc')
    q = np.zeros(2 * n + m)
    A2 = scipy.sparse.hstack([In, Onm, -In])
    A3 = scipy.sparse.hstack([In, Onm, In])
    return dict(Im=Im, Onm=Onm, P=P, q=q, A2=A2, A3=A3,
                ones_n=np.ones(n), zeros_n=np.zeros(n), zeros_nm=np.zeros(n + m))


def estimate_file(name, work_dir, n_tot, start_r, end_r, cfg,
                  subcarriers_space=None, delta_t_refined=None,
                  t_min=-3e-7, t_max=5e-7, delta_t=1e-7,
                  range_refined_down=2e-7, range_refined_up=2.5e-7,
                  overwrite=False, verbose=True):
    last_out = path.join(work_dir,
                         f'r_vector_{name}_stream_{n_tot - 1}.txt')
    if path.exists(last_out) and not overwrite:
        print(f'{name}: 処理済みのためスキップ (--overwrite で再処理)')
        return

    with open(path.join(work_dir, 'signal_' + name + '.txt'), 'rb') as fp:
        signal_complete = pickle.load(fp)

    if signal_complete.shape[0] != cfg.n_occupied:
        raise ValueError(
            f'{name}: 前処理結果のサブキャリア数 {signal_complete.shape[0]} が '
            f'{cfg.name} の {cfg.n_occupied} と一致しません。'
            f'--config の指定を確認してください。')
    if signal_complete.shape[2] != n_tot:
        raise ValueError(
            f'{name}: 前処理結果のストリーム数 {signal_complete.shape[2]} が '
            f'指定の n_ss*n_core={n_tot} と一致しません。')

    if subcarriers_space is None:
        subcarriers_space = cfg.subcarriers_space
    if delta_t_refined is None:
        delta_t_refined = cfg.delta_t_refined

    if end_r == -1:
        end_r = signal_complete.shape[1]
    end_r = min(end_r, signal_complete.shape[1])
    n_time = end_r - start_r
    if n_time <= 0:
        raise ValueError(f'{name}: 処理対象のパケットがありません '
                         f'(start_r={start_r}, end_r={end_r})')

    frequency_vector_complete = cfg.frequency_vector_complete()
    frequency_vector = cfg.frequency_vector()

    T_matrix, time_matrix = build_T_matrix(frequency_vector, delta_t, t_min, t_max)
    r_length = int((t_max - t_min) / delta_t_refined)

    select_subcarriers = np.arange(0, frequency_vector.shape[0], subcarriers_space)
    row_T = select_subcarriers.shape[0]
    col_T = T_matrix.shape[1]

    # 精細化段での未知数の上限 (観測数と比べて劣決定の度合いを見る)
    max_cols_refined = int(round((range_refined_down + range_refined_up)
                                 / delta_t_refined))
    if verbose:
        print(f'{name}: {cfg.name}, ストリーム {n_tot}, パケット {n_time}')
        print(f'  観測サブキャリア {row_T} 本 (間引き {subcarriers_space}), '
              f'粗グリッド {col_T} 列, 精細グリッド 最大 {max_cols_refined} 列 '
              f'(刻み {delta_t_refined*1e9:.1f} ns)')
    if max_cols_refined > 2 * row_T:
        print(f'  警告: 精細グリッドの未知数 ({max_cols_refined}) が観測数 '
              f'({row_T}) に対して多すぎます。--delta_t_refined を粗く、'
              f'または --subcarriers_space を小さくすることを検討してください。')

    aux = _build_aux(row_T, col_T)

    t0 = time.time()
    for stream in range(n_tot):
        signal_considered = signal_complete[:, start_r:end_r, stream]
        r_optim = np.zeros((r_length, n_time), dtype=complex)
        Tr_matrix = np.zeros((frequency_vector_complete.shape[0], n_time),
                             dtype=complex)

        for time_step in range(n_time):
            signal_time = signal_considered[:, time_step]

            # --- 粗いグリッドで主要経路のおおよその遅延を求める ---
            complex_opt_r = lasso_regression_osqp_fast(
                signal_time, T_matrix, select_subcarriers, row_T, col_T,
                aux['Im'], aux['Onm'], aux['P'], aux['q'], aux['A2'], aux['A3'],
                aux['ones_n'], aux['zeros_n'], aux['zeros_nm'])

            time_max_r = time_matrix[np.argmax(abs(complex_opt_r))]

            # --- 最強経路の周辺を精細グリッドで解き直す ---
            t_lo = max(time_max_r - range_refined_down, t_min)
            t_hi = min(time_max_r + range_refined_up, t_max)
            T_matrix_refined, time_matrix_refined = build_T_matrix(
                frequency_vector, delta_t_refined, t_lo, t_hi)

            col_T_refined = T_matrix_refined.shape[1]
            aux_r = _build_aux(row_T, col_T_refined)

            complex_opt_r_refined = lasso_regression_osqp_fast(
                signal_time, T_matrix_refined, select_subcarriers, row_T,
                col_T_refined, aux_r['Im'], aux_r['Onm'], aux_r['P'],
                aux_r['q'], aux_r['A2'], aux_r['A3'], aux_r['ones_n'],
                aux_r['zeros_n'], aux_r['zeros_nm'])

            position_max_r_refined = np.argmax(abs(complex_opt_r_refined))

            # --- 全サブキャリア (ヌル含む) 上で経路を再合成する ---
            T_matrix_refined, time_matrix_refined = build_T_matrix(
                frequency_vector_complete, delta_t_refined, t_lo, t_hi)

            Tr = np.multiply(T_matrix_refined, complex_opt_r_refined)

            # 最強経路の複素共役を掛けて位相の基準を揃える。
            # ここで送受信機由来の共通位相オフセットが相殺される。
            Trr = np.multiply(
                Tr, np.conj(Tr[:, position_max_r_refined:position_max_r_refined + 1]))
            Tr_matrix[:, time_step] = np.sum(Trr, axis=1)

            start_r_opt = int(round((time_matrix_refined[0] - t_min) / delta_t_refined))
            end_r_opt = start_r_opt + complex_opt_r_refined.shape[0]
            r_optim[start_r_opt:end_r_opt, time_step] = complex_opt_r_refined

        os.makedirs(work_dir, exist_ok=True)
        with open(path.join(work_dir,
                            f'r_vector_{name}_stream_{stream}.txt'), 'wb') as fp:
            pickle.dump(r_optim, fp)
        with open(path.join(work_dir,
                            f'Tr_vector_{name}_stream_{stream}.txt'), 'wb') as fp:
            pickle.dump(Tr_matrix, fp)

        if verbose:
            print(f'  ストリーム {stream}: 完了 '
                  f'({time.time() - t0:.1f} 秒経過)')

    meta = load_meta(work_dir, name)
    meta.update(config=cfg.name, n_tot=n_tot, start_r=start_r, end_r=end_r,
                subcarriers_space=int(subcarriers_space),
                delta_t_refined=float(delta_t_refined))
    save_meta(work_dir, name, **meta)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dir', help='入力 .mat があるディレクトリ (名前の一覧取得用)')
    parser.add_argument('all_dir', help='ディレクトリ内の全ファイルを処理するか (1/0)',
                        type=int, default=0)
    parser.add_argument('name', help='処理するファイル名 (拡張子なし)')
    parser.add_argument('nss', help='空間ストリーム数 (SISO なら 1)', type=int)
    parser.add_argument('ncore', help='受信コア数 (アンテナ1本なら 1)', type=int)
    parser.add_argument('start_r', help='処理を開始するパケット番号', type=int)
    parser.add_argument('end_r', help='処理を終了するパケット番号 (-1 で最後まで)',
                        type=int)
    parser.add_argument('--config', default=None,
                        help='規格設定 (VHT20 / HE20)。既定は前処理の記録を使用')
    parser.add_argument('--work_dir', default='./phase_processing/',
                        help='中間ファイルのディレクトリ')
    parser.add_argument('--subcarriers_space', type=int, default=None,
                        help='サブキャリアの間引き幅 (既定は規格ごとの推奨値)')
    parser.add_argument('--delta_t_refined', type=float, default=None,
                        help='精細グリッドの時間刻み [s] (既定は規格ごとの推奨値)')
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

    n_tot = args.nss * args.ncore

    for name in names:
        meta = load_meta(args.work_dir, name)
        cfg_name = args.config or meta.get('config')
        if cfg_name is None:
            raise ValueError(
                f'{name}: 規格を判定できません。前処理を先に実行するか '
                f'--config を指定してください。')
        estimate_file(name, args.work_dir, n_tot, args.start_r, args.end_r,
                      get_config(cfg_name),
                      subcarriers_space=args.subcarriers_space,
                      delta_t_refined=args.delta_t_refined,
                      overwrite=args.overwrite)


if __name__ == '__main__':
    main()

"""
    USRP の CSI ファイルを 1 コマンドで位相サニタイゼーションまで通す。

    内部で手順0〜3 を順に呼ぶだけで、個別スクリプトを直接実行するのと
    結果は同じ。実験条件が Wi-Fi 5/6・20MHz・SISO に固定されている間は
    こちらのほうが扱いやすい。

    例:
      python3 run_phase_sanitization.py ../../CSI/202608191054_OpenWrt-A_CSI.mat
      python3 run_phase_sanitization.py --name walk01 --label W ../../CSI/xxx.mat

    Copyright (C) 2026 Heisuke Takeda
    Released under the GNU GPL v3 (see LICENSE).
"""

import argparse
import os
import time

import usrp_to_sharp
from CSI_phase_sanitization_H_estimation import estimate_file
from CSI_phase_sanitization_signal_preprocessing import process_file
from CSI_phase_sanitization_signal_reconstruction import reconstruct_file
from wifi_config import get_config


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', help='decodeIQ_VHT.m / decodeIQ_HE.m の出力 .mat')
    parser.add_argument('--name', default=None,
                        help='中間・出力ファイルに使う名前 (既定: 入力の basename)')
    parser.add_argument('--format', choices=['VHT', 'HE'], default=None,
                        help='使用する PHY 形式 (既定: パケット数が最多のもの)')
    parser.add_argument('--label', default=None,
                        help='再構成結果を入れるサブディレクトリ名 (活動ラベル等)')
    parser.add_argument('--nss', type=int, default=1, help='空間ストリーム数')
    parser.add_argument('--ncore', type=int, default=1, help='受信コア数')
    parser.add_argument('--start_idx', type=int, default=0,
                        help='先頭から捨てるパケット数')
    parser.add_argument('--end_idx', type=int, default=0,
                        help='末尾から捨てるパケット数')
    parser.add_argument('--fcs_only', action='store_true',
                        help='FCS 検証済みパケットのみ使う')
    parser.add_argument('--subcarriers_space', type=int, default=None,
                        help='サブキャリア間引き幅 (既定は規格ごとの推奨値)')
    parser.add_argument('--delta_t_refined', type=float, default=None,
                        help='精細グリッドの時間刻み [s] (既定は規格ごとの推奨値)')
    parser.add_argument('--input_dir', default='./input_files/',
                        help='変換後 .mat の置き場')
    parser.add_argument('--work_dir', default='./phase_processing/',
                        help='中間ファイルの置き場')
    parser.add_argument('--out_dir', default='./processed_phase/',
                        help='サニタイズ結果の置き場')
    args = parser.parse_args()

    name = args.name or os.path.splitext(os.path.basename(args.input))[0]
    t0 = time.time()

    print('--- 手順0: SHARP 形式へ変換 ---')
    sharp_mat = os.path.join(args.input_dir, name + '.mat')
    info = usrp_to_sharp.convert(args.input, sharp_mat, fmt=args.format,
                                 fcs_only=args.fcs_only)
    cfg = get_config(info['config'])

    print('\n--- 手順1: 前処理 ---')
    process_file(args.input_dir, name, args.work_dir, args.nss, args.ncore,
                 start_idx=0, overwrite=True)

    print('\n--- 手順2: 多重波推定と位相の基準化 ---')
    estimate_file(name, args.work_dir, args.nss * args.ncore, 0, -1, cfg,
                  subcarriers_space=args.subcarriers_space,
                  delta_t_refined=args.delta_t_refined, overwrite=True)

    print('\n--- 手順3: 再構成 ---')
    for stream in range(args.nss * args.ncore):
        reconstruct_file(f'Tr_vector_{name}_stream_{stream}', args.work_dir,
                         args.out_dir, cfg, args.start_idx, args.end_idx,
                         label=args.label, overwrite=True)

    print(f'\n完了 ({time.time() - t0:.1f} 秒)')


if __name__ == '__main__':
    main()

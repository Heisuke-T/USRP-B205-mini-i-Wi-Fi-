"""
    複数の CSI ファイルをまとめて処理する (手順0〜5 を一括実行)。

    変換 → 前処理 → 多重波推定と位相の基準化 → 再構成 → ドップラー計算 →
    マップ描画 までを、指定した全ファイルに対して順に行う。

    夜間に流しっぱなしにすることを想定しているため:
      * 1 ファイルが失敗しても残りを処理し続ける
      * 経過をログファイルに残す (画面が閉じても残る)
      * 途中で止めて再実行すると、完了済みのファイルは飛ばす
      * 最後に成否の一覧を出す

    例:
      python run_batch.py "D:\\IQ_csi\\0927*_WAX202_CSI.mat"
      python run_batch.py a.mat b.mat --label W
      python run_batch.py "D:\\IQ_csi\\*.mat" --resample nudft

    Copyright (C) 2026 Heisuke Takeda
    Released under the GNU GPL v3 (see LICENSE).
"""

import argparse
import glob
import os
import sys
import time
import traceback
from datetime import datetime

import usrp_to_sharp
from CSI_doppler_computation import process_one as doppler_one
from CSI_doppler_plot import load_doppler, plot_doppler
from CSI_phase_sanitization_H_estimation import estimate_file
from CSI_phase_sanitization_signal_preprocessing import process_file
from CSI_phase_sanitization_signal_reconstruction import reconstruct_file
from wifi_config import get_config


class Tee:
    """画面とログファイルの両方へ書く。"""

    def __init__(self, path):
        self.log = open(path, 'a', encoding='utf-8', buffering=1)
        self.stdout = sys.stdout

    def write(self, s):
        self.stdout.write(s)
        self.log.write(s)

    def flush(self):
        self.stdout.flush()
        self.log.flush()

    def close(self):
        self.log.close()


class _DopplerArgs:
    """CSI_doppler_computation.process_one が期待する引数の入れ物。"""

    def __init__(self, a, cfg_fc):
        self.start = a.dop_start
        self.end = a.dop_end
        self.sample_length = a.sample_length
        self.sliding = a.sliding
        self.noise_level = a.noise_level
        self.fc = cfg_fc
        self.Tc = a.Tc
        self.n_fft = a.n_fft
        self.resample = a.resample
        self.cv_warn_threshold = 0.3
        self.subcarrier_range = None
        self.remove_static = a.remove_static


def expand_inputs(patterns):
    """glob を展開し、重複を除いて順に並べる。"""
    files = []
    for pat in patterns:
        hits = sorted(glob.glob(pat))
        if not hits:
            if os.path.exists(pat):
                hits = [pat]
            else:
                print(f'  警告: 該当なし: {pat}')
                continue
        files.extend(hits)
    seen, out = set(), []
    for f in files:
        fp = os.path.abspath(f)
        if fp not in seen:
            seen.add(fp)
            out.append(f)
    return out


def run_one(in_path, args):
    """1 ファイルを手順0〜5 まで通す。戻り値は結果の要約。"""
    name = args.name or os.path.splitext(os.path.basename(in_path))[0]
    t0 = time.time()

    # --- 手順0: SHARP 形式へ変換 ---
    sharp_mat = os.path.join(args.input_dir, name + '.mat')
    info = usrp_to_sharp.convert(in_path, sharp_mat, fmt=args.format,
                                 fcs_only=args.fcs_only)
    cfg = get_config(info['config'])

    n_tot = args.nss * args.ncore

    # --- 手順1: 前処理 ---
    process_file(args.input_dir, name, args.work_dir, args.nss, args.ncore,
                 start_idx=0, overwrite=True)

    # --- 手順2: 多重波推定と位相の基準化 (最も時間が掛かる) ---
    print('  手順2: 多重波推定 (時間が掛かります)')
    estimate_file(name, args.work_dir, n_tot, 0, -1, cfg,
                  subcarriers_space=args.subcarriers_space,
                  delta_t_refined=args.delta_t_refined, overwrite=True)

    # --- 手順3: 再構成 ---
    out_dir = (args.out_dir if args.label is None
               else os.path.join(args.out_dir, args.label))
    for stream in range(n_tot):
        reconstruct_file(f'Tr_vector_{name}_stream_{stream}', args.work_dir,
                         args.out_dir, cfg, args.start_idx, args.end_idx,
                         label=args.label, overwrite=True)

    # --- 手順4 & 5: ドップラー計算と描画 ---
    dop_dir = (args.doppler_dir if args.label is None
               else os.path.join(args.doppler_dir, args.label))
    plot_dir = (args.plot_dir if args.label is None
                else os.path.join(args.plot_dir, args.label))
    os.makedirs(dop_dir, exist_ok=True)

    pngs = []
    for stream in range(n_tot):
        stem = f'{name}_stream_{stream}'
        mat_file = os.path.join(out_dir, stem + '.mat')
        txt_file = os.path.join(dop_dir, stem + '.txt')
        print('  手順4: ドップラー計算')
        meta = doppler_one(mat_file, txt_file, _DopplerArgs(args, args.fc))

        print('  手順5: 描画')
        arr, m = load_doppler(txt_file)
        png = os.path.join(plot_dir, stem + '.png')
        plot_doppler(arr, m, png, title=stem)
        pngs.append(png)
        print(f'    -> {png}')

    return dict(name=name, config=info['config'],
                n_packets=info['n_packets'], pngs=pngs,
                seconds=time.time() - t0, meta=meta)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('inputs', nargs='+',
                        help='入力 .mat (ワイルドカード可、複数指定可)')
    parser.add_argument('--name', default=None,
                        help='出力名。複数ファイルのときは指定しないこと')
    parser.add_argument('--label', default=None,
                        help='出力を入れるサブディレクトリ名 (活動ラベル等)')
    parser.add_argument('--format', choices=['VHT', 'HE'], default=None,
                        help='PHY 形式 (既定: パケット数が最多のもの)')
    parser.add_argument('--fcs_only', action='store_true',
                        help='FCS 検証済みパケットのみ使う')
    parser.add_argument('--nss', type=int, default=1)
    parser.add_argument('--ncore', type=int, default=1)
    # 位相サニタイゼーション
    parser.add_argument('--start_idx', type=int, default=0,
                        help='再構成で先頭から捨てるパケット数')
    parser.add_argument('--end_idx', type=int, default=0,
                        help='再構成で末尾から捨てるパケット数')
    parser.add_argument('--subcarriers_space', type=int, default=None)
    parser.add_argument('--delta_t_refined', type=float, default=None)
    # ドップラー
    parser.add_argument('--dop_start', type=int, default=10,
                        help='ドップラー計算で先頭から捨てるパケット数')
    parser.add_argument('--dop_end', type=int, default=10,
                        help='ドップラー計算で末尾から捨てるパケット数')
    parser.add_argument('--sample_length', type=int, default=31,
                        help='1窓あたりのパケット数')
    parser.add_argument('--sliding', type=int, default=1,
                        help='窓をずらすパケット数')
    parser.add_argument('--noise_level', type=float, default=-1.5,
                        help='雑音床 (10^x で切り捨て)')
    parser.add_argument('--fc', type=float, default=5.18e9,
                        help='中心周波数 [Hz] (既定 5.18e9 = Ch36)')
    parser.add_argument('--Tc', type=float, default=None,
                        help='パケット間隔 [s] (既定: 実測から)')
    parser.add_argument('--n_fft', type=int, default=100)
    parser.add_argument('--resample', choices=['none', 'nudft', 'interp'],
                        default='none')
    parser.add_argument('--remove_static', action='store_true',
                        help='窓内の時間平均 (静止経路) を引く。直接波が強く'
                             '動きが埋もれる場合に使う')
    # 置き場所
    parser.add_argument('--input_dir', default='./input_files/')
    parser.add_argument('--work_dir', default='./phase_processing/')
    parser.add_argument('--out_dir', default='./processed_phase/')
    parser.add_argument('--doppler_dir', default='./doppler_traces/')
    parser.add_argument('--plot_dir', default='./plots/')
    parser.add_argument('--log', default=None,
                        help='ログの出力先 (既定: ./batch_<日時>.log)')
    parser.add_argument('--redo', action='store_true',
                        help='完了済み (PNG がある) のファイルもやり直す')
    args = parser.parse_args()

    files = expand_inputs(args.inputs)
    if not files:
        print('処理対象のファイルがありません。')
        return 1
    if args.name and len(files) > 1:
        print('--name は複数ファイルには使えません (出力が上書きされるため)。')
        return 1

    log_path = args.log or f'./batch_{datetime.now():%Y%m%d_%H%M%S}.log'
    tee = Tee(log_path)
    sys.stdout = tee

    t_all = time.time()
    print('=' * 68)
    print(f'一括処理を開始: {datetime.now():%Y-%m-%d %H:%M:%S}')
    print(f'対象 {len(files)} ファイル / ログ {log_path}')
    for f in files:
        sz = os.path.getsize(f) / 1e6 if os.path.exists(f) else float('nan')
        print(f'  - {f}  ({sz:.1f} MB)')
    print('=' * 68)

    results, failures = [], []
    for i, f in enumerate(files, 1):
        name = args.name or os.path.splitext(os.path.basename(f))[0]
        print(f'\n[{i}/{len(files)}] {name}  '
              f'({datetime.now():%H:%M:%S})')
        print('-' * 68)

        # 完了済みなら飛ばす
        plot_dir = (args.plot_dir if args.label is None
                    else os.path.join(args.plot_dir, args.label))
        done = os.path.join(plot_dir, f'{name}_stream_0.png')
        if os.path.exists(done) and not args.redo:
            print(f'  完了済みのため飛ばします ({done})。'
                  f'やり直すには --redo')
            continue

        try:
            r = run_one(f, args)
            results.append(r)
            print(f'  完了 ({r["seconds"]/60:.1f} 分)')
        except Exception as e:                       # noqa: BLE001
            # 夜間実行のため、1件の失敗で全体を止めない
            failures.append((f, str(e)))
            print(f'  *** 失敗: {e}')
            traceback.print_exc(file=sys.stdout)

    print('\n' + '=' * 68)
    print(f'全体の所要時間: {(time.time() - t_all)/60:.1f} 分')
    print(f'成功 {len(results)} 件 / 失敗 {len(failures)} 件')
    if results:
        print('\n成功:')
        for r in results:
            m = r['meta']
            print(f"  {r['name']}: {r['config']}, {r['n_packets']} パケット, "
                  f"{r['seconds']/60:.1f} 分")
            print(f"    速度軸 ±{m['v_max']:.2f} m/s, "
                  f"ビン間隔 {m['delta_v_bin']:.3f} m/s, "
                  f"Tc={m['Tc']*1e3:.3f} ms")
            for p in r['pngs']:
                print(f"    {p}")
    if failures:
        print('\n失敗:')
        for f, e in failures:
            print(f'  {f}\n    {e}')
    print('=' * 68)

    sys.stdout = tee.stdout
    tee.close()
    print(f'\nログ: {log_path}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

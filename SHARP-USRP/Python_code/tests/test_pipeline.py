"""
    SHARP-USRP パイプラインの自己診断テスト。

    合成データ (既知の多重波 + パケットごとのランダム位相/時刻オフセット) を
    Wi-Fi 5 (VHT20) と Wi-Fi 6 (HE20) の両方で作り、変換 → 前処理 →
    H 推定 → 再構成 まで通したうえで、

      1. 変換が可逆であること (使用サブキャリアの値が完全一致)
      2. 位相のパケット間ばらつきが大幅に減ること (サニタイゼーションの効果)
      3. 出力の形が規格どおりであること

    を確認する。実行:  python3 tests/test_pipeline.py

    Copyright (C) 2026 Heisuke Takeda
    Released under the GNU GPL v3 (see LICENSE).
"""

import os
import shutil
import sys
import tempfile

import numpy as np
import scipy.io as sio

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, _HERE)

from make_synthetic_usrp_mat import synth_csi, write_mat  # noqa: E402

import usrp_to_sharp  # noqa: E402
from CSI_phase_sanitization_H_estimation import estimate_file  # noqa: E402
from CSI_phase_sanitization_signal_preprocessing import process_file  # noqa: E402
from CSI_phase_sanitization_signal_reconstruction import reconstruct_file  # noqa: E402
from wifi_config import get_config  # noqa: E402


class TestFailure(AssertionError):
    pass


def check(cond, msg):
    if not cond:
        raise TestFailure(msg)
    print(f'    OK: {msg}')


def phase_spread(phase):
    """パケット間の位相差のばらつき [rad] (サブキャリア平均)。"""
    d = np.angle(np.exp(1j * np.diff(phase, axis=0)))
    return float(np.std(d, axis=0).mean())


def run_case(config_name, workdir, n_pkt=100):
    print(f'\n=== {config_name} ===')
    cfg = get_config(config_name)

    in_dir = os.path.join(workdir, 'raw')
    sharp_dir = os.path.join(workdir, 'input_files')
    work_dir = os.path.join(workdir, 'phase_processing')
    out_dir = os.path.join(workdir, 'processed_phase')
    name = 'synth_' + config_name.lower()

    # --- 合成データ生成 ---
    csi = synth_csi(cfg, n_pkt=n_pkt, seed=7)
    src_mat = os.path.join(in_dir, name + '_src.mat')
    write_mat(src_mat, cfg, csi, seed=7)

    # --- 手順0: SHARP 形式へ変換 ---
    sharp_mat = os.path.join(sharp_dir, name + '.mat')
    info = usrp_to_sharp.convert(src_mat, sharp_mat, verbose=False)
    check(info['config'] == cfg.name,
          f'変換で規格を自動判定 ({info["config"]})')
    check(info['n_packets'] == n_pkt, f'パケット数が保存される ({n_pkt})')

    # 変換の可逆性: fftshift で戻した使用サブキャリアが元と完全一致するか
    buff = sio.loadmat(sharp_mat)['csi_buff']
    check(buff.shape == (n_pkt, cfg.fft_size),
          f'csi_buff の形が [パケット x FFT長] = {buff.shape}')
    centered = np.fft.fftshift(buff, axes=1)
    back = centered[:, cfg.k_to_centered_idx(cfg.occupied)]
    check(np.array_equal(back, csi), '変換が可逆 (使用サブキャリアが完全一致)')
    check(np.abs(centered[:, cfg.delete_idxs]).max() == 0,
          'ヌルサブキャリアが 0 である')

    # --- 手順1: 前処理 ---
    shape = process_file(sharp_dir, name, work_dir, n_ss=1, n_core=1,
                         start_idx=0, overwrite=True)
    check(shape == (cfg.n_occupied, n_pkt, 1),
          f'前処理の出力形 {shape} = (使用サブキャリア, パケット, ストリーム)')

    # --- 手順2: H 推定 ---
    estimate_file(name, work_dir, n_tot=1, start_r=0, end_r=-1, cfg=cfg,
                  overwrite=True, verbose=False)
    check(os.path.exists(os.path.join(work_dir,
                                      f'Tr_vector_{name}_stream_0.txt')),
          'H 推定が Tr を出力')

    # --- 手順3: 再構成 ---
    reconstruct_file(f'Tr_vector_{name}_stream_0', work_dir, out_dir, cfg,
                     start_idx=0, end_idx=0, overwrite=True, verbose=False)
    out_mat = os.path.join(out_dir, f'{name}_stream_0.mat')
    san = sio.loadmat(out_mat)['csi_matrix_processed']
    check(san.shape == (n_pkt, cfg.n_crop, 2),
          f'再構成の出力形 {san.shape} = (パケット, 帯域内サブキャリア, [振幅,位相])')

    # --- 効果の確認: 位相のばらつきが減っているか ---
    occ_in_crop = cfg.k_to_centered_idx(cfg.occupied) - cfg.crop_slice.start
    raw_norm = csi / np.mean(np.abs(csi), axis=1, keepdims=True)
    spread_raw = phase_spread(np.unwrap(np.angle(raw_norm), axis=1))
    spread_san = phase_spread(san[:, occ_in_crop, 1])
    ratio = spread_raw / spread_san
    print(f'    位相ばらつき: 生 {spread_raw:.3f} rad -> '
          f'サニタイズ後 {spread_san:.3f} rad ({ratio:.1f} 倍改善)')
    check(ratio > 3.0, f'位相ばらつきが 3 倍以上改善 (実測 {ratio:.1f} 倍)')

    # --- 振幅が保存されているか ---
    corr = np.corrcoef(np.abs(csi).mean(0),
                       san[:, occ_in_crop, 0].mean(0))[0, 1]
    check(corr > 0.95, f'平均振幅プロファイルが保存される (相関 {corr:.4f})')

    return dict(config=cfg.name, ratio=ratio, corr=corr)


def main():
    workdir = tempfile.mkdtemp(prefix='sharp_usrp_test_')
    try:
        results = [run_case('VHT20', workdir), run_case('HE20', workdir)]
    except TestFailure as e:
        print(f'\n失敗: {e}')
        return 1
    finally:
        shutil.rmtree(workdir, ignore_errors=True)

    print('\n=== まとめ ===')
    for r in results:
        print(f"  {r['config']}: 位相ばらつき {r['ratio']:.1f} 倍改善, "
              f"振幅相関 {r['corr']:.4f}")
    print('\nすべてのテストに合格しました。')
    return 0


if __name__ == '__main__':
    sys.exit(main())

"""
    MATLAB 版 (matlab/usrpCSItoSHARP.m) と Python 版 (usrp_to_sharp.py) の
    変換結果が一致することを確認する。

    MATLAB 本体が無い環境でも検証できるよう GNU Octave で実行する
    (Octave が無ければスキップ)。実行:

        python3 tests/test_matlab_parity.py

    Copyright (C) 2026 Heisuke Takeda
    Released under the GNU GPL v3 (see LICENSE).
"""

import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import scipy.io as sio

_HERE = os.path.dirname(os.path.abspath(__file__))
_PY = os.path.dirname(_HERE)
_MATLAB = os.path.join(os.path.dirname(_PY), 'matlab')
sys.path.insert(0, _PY)
sys.path.insert(0, _HERE)

import usrp_to_sharp  # noqa: E402
from make_synthetic_usrp_mat import synth_csi, write_mat  # noqa: E402
from wifi_config import get_config  # noqa: E402


def find_octave():
    return shutil.which('octave') or shutil.which('octave-cli')


def run_octave(octave, src, dst, fmt):
    script = (
        f"addpath('{_MATLAB}');"
        f"usrpCSItoSHARP('{src}', '{dst}', 'Format', '{fmt}');"
    )
    res = subprocess.run([octave, '--no-gui', '--quiet', '--eval', script],
                         capture_output=True, text=True, timeout=600)
    if res.returncode != 0:
        raise RuntimeError(f'Octave 実行に失敗:\n{res.stdout}\n{res.stderr}')
    return res.stdout


def main():
    octave = find_octave()
    if octave is None:
        print('Octave が見つからないためスキップします。')
        print('  (Ubuntu: apt-get install octave)')
        return 0

    workdir = tempfile.mkdtemp(prefix='sharp_usrp_parity_')
    failures = []
    try:
        for cfg_name, fmt in [('VHT20', 'VHT'), ('HE20', 'HE')]:
            print(f'\n=== {cfg_name} ===')
            cfg = get_config(cfg_name)
            src = os.path.join(workdir, f'src_{cfg_name}.mat')
            write_mat(src, cfg, synth_csi(cfg, n_pkt=40, seed=3), seed=3)

            py_out = os.path.join(workdir, f'py_{cfg_name}.mat')
            oct_out = os.path.join(workdir, f'oct_{cfg_name}.mat')
            usrp_to_sharp.convert(src, py_out, fmt=fmt, verbose=False)
            run_octave(octave, src, oct_out, fmt)

            a = sio.loadmat(py_out)
            b = sio.loadmat(oct_out)

            same_csi = np.array_equal(a['csi_buff'], b['csi_buff'])
            max_diff = float(np.abs(a['csi_buff'] - b['csi_buff']).max())
            same_cfg = str(a['sharp_config'][0]) == str(b['sharp_config'][0])
            same_sub = np.array_equal(a['subcarrier_indices'].ravel(),
                                      b['subcarrier_indices'].ravel())

            print(f'  csi_buff  一致={same_csi} (最大絶対差 {max_diff:.3e})')
            print(f'  設定名     一致={same_cfg}')
            print(f'  サブキャリア 一致={same_sub}')

            if not (same_csi and same_cfg and same_sub):
                failures.append(cfg_name)
    finally:
        shutil.rmtree(workdir, ignore_errors=True)

    if failures:
        print(f'\n失敗: {failures}')
        return 1
    print('\nMATLAB 版と Python 版の出力は完全に一致しました。')
    return 0


if __name__ == '__main__':
    sys.exit(main())

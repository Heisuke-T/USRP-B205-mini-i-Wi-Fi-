"""
    USRP B205mini-i のデコード済み CSI を SHARP 位相サニタイゼーションの
    入力形式へ変換する。

    入力: decodeIQ_VHT.m / decodeIQ_HE.m が出力した .mat
          (csiVHT / csiHE などを含む。MATLAB v7.3 = HDF5 と v7 の両対応)
    出力: SHARP が読む .mat
          csi_buff [パケット数 x FFT長] complex, FFT 順 (Nexmon と同じ並び)

    Nexmon CSI (SHARP のオリジナル入力) は FFT 出力そのままの並びで
    csi_buff に格納されており、SHARP 側は先頭で np.fft.fftshift を掛けて
    中心寄せに直している。互換性を保つため本スクリプトも FFT 順で書き出す。

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

import numpy as np
import scipy.io as sio

from wifi_config import get_config, CONFIGS


# =====================================================================
#  MATLAB .mat の読み込み (v7.3=HDF5 / v7 以前の両対応)
# =====================================================================

def _is_hdf5(path):
    """MATLAB v7.3 (HDF5) かどうか。

    v7.3 は先頭 512 バイトが MATLAB のヘッダで、HDF5 署名はその後に
    現れる。h5py.is_hdf5 は規格どおり 512, 1024, ... のオフセットも
    調べるため、これを使う。
    """
    try:
        import h5py
        return bool(h5py.is_hdf5(path))
    except ImportError:
        with open(path, 'rb') as fp:
            head = fp.read(128)
        return b'MATLAB 7.3' in head


class UsrpMat:
    """USRP デコーダ出力 .mat への読み出しラッパ。

    MATLAB の行列は v7.3 (HDF5) では転置されて格納されるため、
    どちらの形式でも「MATLAB 上の向き」で返すよう吸収する。
    """

    def __init__(self, path):
        self.path = path
        self._h5 = None
        self._mat = None
        if _is_hdf5(path):
            import h5py
            self._h5 = h5py.File(path, 'r')
        else:
            self._mat = sio.loadmat(path)

    def close(self):
        if self._h5 is not None:
            self._h5.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def has(self, name):
        if self._h5 is not None:
            return name in self._h5
        return name in self._mat

    def complex_matrix(self, name):
        """複素行列を [行 x 列] (MATLAB 上の向き) で返す。"""
        if self._h5 is not None:
            data = self._h5[name][()]
            if data.dtype.names and 'real' in data.dtype.names:
                arr = data['real'] + 1j * data['imag']
            else:
                arr = np.asarray(data, dtype=complex)
            # HDF5 は列優先で転置保存されている
            return arr.T
        return np.asarray(self._mat[name], dtype=complex)

    def vector(self, name):
        """数値ベクトルを 1 次元で返す。"""
        if self._h5 is not None:
            return np.asarray(self._h5[name][()], dtype=float).ravel()
        return np.asarray(self._mat[name], dtype=float).ravel()

    def string(self, name):
        """文字列変数を str で返す (読めない場合は空文字)。"""
        try:
            if self._h5 is not None:
                raw = self._h5[name][()]
                return ''.join(chr(int(c)) for c in np.asarray(raw).ravel())
            val = self._mat[name]
            return str(np.asarray(val).ravel()[0])
        except Exception:
            return ''

    def meta_scalar(self, field, default=None):
        """csiMeta の数値フィールドを取り出す。"""
        try:
            if self._h5 is not None:
                return float(np.asarray(self._h5['csiMeta'][field][()]).ravel()[0])
            meta = self._mat['csiMeta']
            return float(np.asarray(meta[field][0, 0]).ravel()[0])
        except Exception:
            return default

    def meta_string(self, field, default=''):
        try:
            if self._h5 is not None:
                raw = self._h5['csiMeta'][field][()]
                return ''.join(chr(int(c)) for c in np.asarray(raw).ravel())
            meta = self._mat['csiMeta']
            return str(np.asarray(meta[field][0, 0]).ravel()[0])
        except Exception:
            return default


# =====================================================================
#  変換本体
# =====================================================================

# デコーダが使う変数名 (形式ごと)
_VARS = {
    'VHT': dict(csi='csiVHT', sub='subcarrierIndicesVHT20',
                time='timeSecVHT', fcs='fcsVHT'),
    'HE':  dict(csi='csiHE',  sub='subcarrierIndicesHE20',
                time='timeSecHE',  fcs='fcsHE'),
}


def detect_format(mat):
    """ファイル内で最もパケット数が多い対応形式を選ぶ。"""
    best, best_n = None, 0
    for fmt, names in _VARS.items():
        if not mat.has(names['csi']):
            continue
        try:
            n = mat.complex_matrix(names['csi']).shape[0]
        except Exception:
            continue
        if n > best_n:
            best, best_n = fmt, n
    if best is None:
        raise ValueError(
            'csiVHT / csiHE が見つかりません。decodeIQ_VHT.m または '
            'decodeIQ_HE.m の出力ファイルを指定してください。')
    return best


def load_usrp_csi(mat, fmt):
    """指定形式の CSI 行列・サブキャリア番号・時刻・FCS を取り出す。"""
    names = _VARS[fmt]
    csi = mat.complex_matrix(names['csi'])      # [パケット数 x サブキャリア数]
    if csi.ndim != 2 or csi.size == 0:
        raise ValueError(f"{names['csi']} が空、または 2 次元ではありません。")

    sub = mat.vector(names['sub']).astype(int)

    time_sec = mat.vector(names['time']) if mat.has(names['time']) else None
    if time_sec is not None and time_sec.size != csi.shape[0]:
        time_sec = None

    fcs = None
    if mat.has(names['fcs']):
        try:
            f = mat.vector(names['fcs'])
            if f.size == csi.shape[0]:
                fcs = f.astype(bool)
        except Exception:
            fcs = None

    return csi, sub, time_sec, fcs


def build_csi_buff(csi, sub, cfg):
    """[パケット x 使用サブキャリア] -> [パケット x FFT長] (FFT 順)。

    使用サブキャリアを中心寄せ配列の所定位置に配置し、ヌルは 0 のまま
    残したうえで、ifftshift で FFT 順 (Nexmon と同じ並び) に直す。
    """
    if sub.size != csi.shape[1]:
        raise ValueError(
            f'サブキャリア番号の数 ({sub.size}) と CSI の列数 '
            f'({csi.shape[1]}) が一致しません。')

    expected = set(cfg.occupied.tolist())
    got = set(sub.tolist())
    if got != expected:
        missing = sorted(expected - got)
        extra = sorted(got - expected)
        raise ValueError(
            f'{cfg.name} の使用サブキャリア構成と一致しません。\n'
            f'  不足: {missing[:12]}{"..." if len(missing) > 12 else ""}\n'
            f'  余分: {extra[:12]}{"..." if len(extra) > 12 else ""}\n'
            f'  20MHz / SISO のデータか確認してください。')

    n_pkt = csi.shape[0]
    centered = np.zeros((n_pkt, cfg.fft_size), dtype=complex)
    centered[:, cfg.k_to_centered_idx(sub)] = csi

    # 中心寄せ -> FFT 順 (SHARP 側の fftshift で元に戻る)
    return np.fft.ifftshift(centered, axes=1)


def filter_packets(csi, time_sec, fcs, fcs_only, verbose=True):
    """無効パケット (全ゼロ / NaN) と、必要なら FCS 未検証パケットを除く。"""
    n0 = csi.shape[0]
    keep = np.ones(n0, dtype=bool)

    bad = (np.abs(csi).sum(axis=1) == 0) | (~np.isfinite(csi).all(axis=1))
    keep &= ~bad
    if verbose and bad.any():
        print(f'  除外: 全ゼロ/非有限の CSI {int(bad.sum())} パケット')

    if fcs_only:
        if fcs is None:
            print('  警告: FCS 情報が無いため --fcs_only を無視します')
        else:
            drop = keep & ~fcs
            keep &= fcs
            if verbose and drop.any():
                print(f'  除外: FCS 未検証 {int(drop.sum())} パケット')

    csi = csi[keep]
    time_sec = time_sec[keep] if time_sec is not None else None
    if verbose:
        print(f'  採用パケット数: {csi.shape[0]} / {n0}')
    return csi, time_sec


def packet_rate_stats(time_sec):
    """パケット時刻から取得レートのばらつきを算出する。"""
    if time_sec is None or time_sec.size < 2:
        return None
    dt = np.diff(np.sort(time_sec))
    dt = dt[dt > 0]
    if dt.size == 0:
        return None
    return dict(
        n_pkt=int(time_sec.size),
        duration=float(time_sec[-1] - time_sec[0]),
        mean_rate=float(1.0 / dt.mean()),
        median_interval_ms=float(np.median(dt) * 1e3),
        mean_interval_ms=float(dt.mean() * 1e3),
        min_interval_ms=float(dt.min() * 1e3),
        max_interval_ms=float(dt.max() * 1e3),
        cv=float(dt.std() / dt.mean()),
    )


def convert(in_path, out_path, fmt=None, fcs_only=False, verbose=True):
    """1 ファイルを変換して保存し、メタ情報を返す。"""
    with UsrpMat(in_path) as mat:
        fmt = fmt or detect_format(mat)
        cfg = get_config(fmt)

        if verbose:
            print(f'入力: {in_path}')
            print(f'形式: {fmt} -> 設定 {cfg.name} ({cfg.description})')

        csi, sub, time_sec, fcs = load_usrp_csi(mat, fmt)
        if verbose:
            print(f'  読み込み: {csi.shape[0]} パケット x '
                  f'{csi.shape[1]} サブキャリア')

        csi, time_sec = filter_packets(csi, time_sec, fcs, fcs_only, verbose)
        if csi.shape[0] == 0:
            raise ValueError('有効なパケットが 1 つも残りませんでした。')

        csi_buff = build_csi_buff(csi, sub, cfg)

        stats = packet_rate_stats(time_sec)

        source_meta = dict(
            center_frequency=mat.meta_scalar('centerFrequency', np.nan),
            sample_rate=mat.meta_scalar('sampleRate', np.nan),
            gain=mat.meta_scalar('gain', np.nan),
            wifi_channel=mat.meta_scalar('wifiChannel', np.nan),
            capture_datetime=mat.meta_string('captureDatetime'),
            platform=mat.meta_string('platform'),
        )

    mdic = {
        'csi_buff': csi_buff,
        # --- SHARP 側および後段の Doppler 処理が使うメタ情報 ---
        'sharp_config': cfg.name,
        'standard': cfg.standard,
        'bandwidth_mhz': cfg.bandwidth_mhz,
        'fft_size': cfg.fft_size,
        'delta_f': cfg.delta_f,
        'subcarrier_indices': cfg.occupied.astype(np.float64),
        'n_ss': 1,
        'n_core': 1,
        'source_file': os.path.basename(in_path),
    }
    if time_sec is not None:
        # パケット取得時刻。Doppler 解析では等間隔サンプリングが前提に
        # なるため、後段で再標本化する際に必要となる。
        mdic['time_sec'] = time_sec.astype(np.float64)
    for k, v in source_meta.items():
        mdic['src_' + k] = v

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    # scipy.io.loadmat は v7.3 (HDF5) を読めないため v7 形式で保存する
    sio.savemat(out_path, mdic, do_compression=True)

    if verbose:
        print(f'  csi_buff: {csi_buff.shape} (パケット x FFT長, FFT順)')
        if stats:
            print(f'  取得レート: 平均 {stats["mean_rate"]:.1f} pkt/s '
                  f'(間隔 中央値 {stats["median_interval_ms"]:.2f} ms, '
                  f'{stats["min_interval_ms"]:.2f}..'
                  f'{stats["max_interval_ms"]:.2f} ms, '
                  f'変動係数 {stats["cv"]:.2f})')
            if stats['cv'] > 0.2:
                print('  注意: パケット間隔のばらつきが大きく、等間隔サンプリングを')
                print('        前提とする Doppler 解析にはリサンプリングが必要です。')
                print('        (位相サニタイゼーション自体はパケット単位処理のため影響なし)')
        print(f'出力: {out_path}')

    return dict(config=cfg.name, n_packets=int(csi_buff.shape[0]),
                stats=stats, out_path=out_path)


def main():
    parser = argparse.ArgumentParser(
        description='USRP のデコード済み CSI を SHARP 入力形式へ変換する',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='例:\n'
               '  python usrp_to_sharp.py ../../CSI/xxx_CSI.mat '
               './input_files/exp01.mat\n'
               '  python usrp_to_sharp.py --format HE in.mat out.mat\n')
    parser.add_argument('input', help='decodeIQ_VHT.m / decodeIQ_HE.m の出力 .mat')
    parser.add_argument('output', help='SHARP 入力用に書き出す .mat')
    parser.add_argument('--format', choices=sorted(_VARS),
                        help='使用する PHY 形式 (既定: パケット数が最多のもの)')
    parser.add_argument('--fcs_only', action='store_true',
                        help='FCS 検証済みパケットのみ使用する')
    args = parser.parse_args()

    convert(args.input, args.output, fmt=args.format, fcs_only=args.fcs_only)


if __name__ == '__main__':
    main()

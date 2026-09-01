"""
    Wi-Fi PHY のサブキャリア構成定義 (SHARP-USRP)

    SHARP / SHARPax のオリジナル実装は 80MHz 帯域が前提で、FFT 長・
    ヌルサブキャリア番号・再構成時の切り出し範囲がすべてハードコード
    されていた。本モジュールはそれらを「規格 × 帯域」ごとの設定として
    切り出し、20MHz の VHT / HE を扱えるようにする。

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

import numpy as np


class WiFiConfig:
    """1つの (規格, 帯域) 組み合わせにおける OFDM 構成。

    サブキャリア番号 k は論理番号 (DC を 0 とする符号付き整数)。
    配列上の「中心寄せ順」インデックス i とは i = k + fft_size/2 で対応する。
    これは numpy.fft.fftshift を掛けた後の並び (SHARP が前提とする並び) と
    一致する。
    """

    def __init__(self, name, standard, bandwidth_mhz, fft_size, delta_f,
                 occupied, pilots, subcarriers_space, delta_t_refined,
                 description):
        self.name = name
        self.standard = standard                  # 'VHT' (Wi-Fi 5) / 'HE' (Wi-Fi 6)
        self.bandwidth_mhz = bandwidth_mhz
        self.fft_size = fft_size                  # FFT 長 (= 全サブキャリア数)
        self.delta_f = delta_f                    # サブキャリア間隔 [Hz]
        self.occupied = np.asarray(occupied, dtype=int)   # 使用サブキャリア番号 k
        self.pilots = np.asarray(pilots, dtype=int)       # パイロット副搬送波 k
        # 位相サニタイゼーションの既定パラメータ (根拠は README 参照)
        self.subcarriers_space = subcarriers_space
        self.delta_t_refined = delta_t_refined
        self.description = description

    # ---- 配列インデックス変換 -------------------------------------------
    @property
    def half(self):
        return self.fft_size // 2

    def k_to_centered_idx(self, k):
        """論理サブキャリア番号 k -> 中心寄せ配列インデックス。"""
        return np.asarray(k, dtype=int) + self.half

    @property
    def occupied_idx(self):
        """使用サブキャリアの中心寄せインデックス (昇順)。"""
        return np.sort(self.k_to_centered_idx(self.occupied))

    @property
    def delete_idxs(self):
        """ヌル (ガードバンド + DC) の中心寄せインデックス。

        SHARP の ``delete_idxs`` に相当する。使用サブキャリア以外すべて。
        """
        all_idx = np.arange(self.fft_size)
        return np.setdiff1d(all_idx, self.occupied_idx)

    @property
    def n_occupied(self):
        return self.occupied.size

    # ---- 周波数軸 --------------------------------------------------------
    def frequency_vector_complete(self):
        """全 FFT ビンの周波数 [Hz] (中心寄せ順)。

        SHARP の H_estimation が構築していたものと同じ並び:
        index i の周波数は (i - fft_size/2) * delta_f。
        """
        return (np.arange(self.fft_size) - self.half) * self.delta_f

    def frequency_vector(self):
        """使用サブキャリアのみの周波数 [Hz] (ヌル除去後、昇順)。"""
        return np.delete(self.frequency_vector_complete(), self.delete_idxs)

    # ---- 再構成時の切り出し ---------------------------------------------
    @property
    def crop_slice(self):
        """再構成で出力する連続帯域の範囲。

        SHARP は 80MHz/256点に対し ``[6:-5]`` を決め打ちしていた。これは
        「最低位の使用サブキャリアから最高位の使用サブキャリアまで」を表す
        (間に挟まる DC ヌルは 0 のまま残す)。同じ意味を一般化する。
        """
        occ = self.occupied_idx
        return slice(int(occ[0]), int(occ[-1]) + 1)

    @property
    def n_crop(self):
        s = self.crop_slice
        return s.stop - s.start

    # ---- 分解能の目安 ----------------------------------------------------
    @property
    def delay_resolution(self):
        """遅延分解能 [s] (= 1/帯域幅)。多重波の分離能力の目安。"""
        return 1.0 / (self.bandwidth_mhz * 1e6)

    @property
    def max_unambiguous_delay(self):
        """曖昧さのない最大遅延 [s] (= 1/delta_f)。"""
        return 1.0 / self.delta_f

    def summary(self):
        return (
            f"{self.name}: {self.description}\n"
            f"  FFT長={self.fft_size}, Δf={self.delta_f/1e3:.3f} kHz, "
            f"帯域={self.bandwidth_mhz} MHz\n"
            f"  使用サブキャリア={self.n_occupied} 本 "
            f"(k={self.occupied.min()}..{self.occupied.max()}), "
            f"ヌル={self.delete_idxs.size} 本\n"
            f"  遅延分解能={self.delay_resolution*1e9:.1f} ns, "
            f"最大非曖昧遅延={self.max_unambiguous_delay*1e6:.2f} us\n"
            f"  既定: subcarriers_space={self.subcarriers_space}, "
            f"delta_t_refined={self.delta_t_refined*1e9:.1f} ns"
        )


def _vht20():
    # IEEE 802.11ac CBW20: 使用 56 本 (データ 52 + パイロット 4), DC ヌル
    occupied = list(range(-28, 0)) + list(range(1, 29))
    pilots = [-21, -7, 7, 21]
    return WiFiConfig(
        name='VHT20', standard='VHT', bandwidth_mhz=20,
        fft_size=64, delta_f=312.5e3,
        occupied=occupied, pilots=pilots,
        # 56 本しかないため、間引くと最適化の観測数が足りなくなる → 全数使用
        subcarriers_space=1,
        # 20MHz の遅延分解能は 50ns。5ns 刻みでは列数が観測数を超えて
        # 劣決定が過度になるため 10ns 刻みにする
        delta_t_refined=1e-8,
        description='Wi-Fi 5 (802.11ac) 20MHz SU-VHT',
    )


def _he20():
    # IEEE 802.11ax CBW20: 使用 242 本 (データ 234 + パイロット 8), DC 3本ヌル
    occupied = list(range(-122, -1)) + list(range(2, 123))
    pilots = [-116, -90, -48, -22, 22, 48, 90, 116]
    return WiFiConfig(
        name='HE20', standard='HE', bandwidth_mhz=20,
        fft_size=256, delta_f=78.125e3,
        occupied=occupied, pilots=pilots,
        # 242 本はオリジナル SHARP (80MHz VHT) と同数のため既定値を踏襲
        subcarriers_space=2,
        delta_t_refined=5e-9,
        description='Wi-Fi 6 (802.11ax) 20MHz HE-SU',
    )


CONFIGS = {
    'VHT20': _vht20(),
    'HE20': _he20(),
}

# USRP デコーダ (decodeIQ_VHT.m / decodeIQ_HE.m) が使う形式名との対応
FORMAT_TO_CONFIG = {
    'VHT': 'VHT20',
    'HE': 'HE20',
}


def get_config(name):
    """設定名 ('VHT20' / 'HE20') から WiFiConfig を得る。"""
    key = str(name).strip().upper()
    if key in CONFIGS:
        return CONFIGS[key]
    if key in FORMAT_TO_CONFIG:
        return CONFIGS[FORMAT_TO_CONFIG[key]]
    raise ValueError(
        f"未知の設定 '{name}'。利用可能: {sorted(CONFIGS)} "
        f"(または形式名 {sorted(FORMAT_TO_CONFIG)})")


if __name__ == '__main__':
    for cfg in CONFIGS.values():
        print(cfg.summary())
        print()

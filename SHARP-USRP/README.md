# SHARP-USRP

USRP B205mini-i で取得・デコードした Wi-Fi CSI を、SHARP の位相サニタイゼーション
アルゴリズムに入力するためのシステム。

対象とする実験条件:

| | 規格 | 帯域 | チャネル | アンテナ | 設定名 |
|---|---|---|---|---|---|
| パターン1 | Wi-Fi 5 (802.11ac, VHT) | 20 MHz | Ch36 | 1 (SISO) | `VHT20` |
| パターン2 | Wi-Fi 6 (802.11ax, HE) | 20 MHz | Ch36 | 1 (SISO) | `HE20` |

## 結論: 変換は可能

SHARP の位相サニタイゼーションが必要とするのは **複素チャネル周波数応答 (CFR) の
サブキャリア方向の並び** だけで、それがどのハードウェアで得られたかには依存しない。
Nexmon CSI (pcap 経由) でも USRP + MATLAB WLAN Toolbox のチャネル推定でも、
「パケットごとの複素 CFR ベクトル」という点では同じものである。

必要なのは次の 2 つだけ:

1. **形式の変換** — USRP デコーダは使用サブキャリアのみ (VHT20 なら 56 本) を
   出力するが、SHARP は FFT 長ぶんの配列 (ヌル込み) を前提とする。
2. **帯域のパラメータ化** — SHARP / SHARPax はどちらも 80 MHz 決め打ちで、
   FFT 長・ヌルサブキャリア番号・切り出し範囲がハードコードされている。
   20 MHz 用に一般化する必要がある。

本リポジトリはこの 2 点を実装したもの。実データでの検証結果は「[検証](#検証)」を参照。

### なぜ USRP のデータで位相サニタイゼーションが特に必要か

USRP は AP と位相同期していないため、パケットごとに**ランダムな共通位相**と
**時刻オフセット** (パケット検出位置のずれ = 周波数方向の位相傾斜) が乗る。
実測データでは生の CSI の位相はパケット間でほぼランダム (標準偏差 1.96 rad)
であり、そのままでは位相を使った解析ができない。

SHARP の手法は、CFR を「遅延の異なる複数経路の和」としてスパース推定し、
**最強経路の位相を基準に全経路を再合成する**ことでこれらのオフセットを相殺する。
これはまさに USRP のデータが必要としている処理である。

## データの流れ

```
USRP B205mini-i
   │  captureIQ.m
   ▼
生IQ (.mat)
   │  decodeIQ_VHT.m  (Wi-Fi 5)  /  decodeIQ_HE.m (Wi-Fi 6)
   ▼
デコード済み CSI (.mat)              csiVHT [パケット x 56] など
   │
   │  ★ 手順0: usrp_to_sharp.py  または  matlab/usrpCSItoSHARP.m
   ▼
SHARP 入力形式 (.mat)                csi_buff [パケット x FFT長] (FFT順)
   │
   │  ★ 手順1: CSI_phase_sanitization_signal_preprocessing.py
   ▼
signal_<name>.txt                    [使用サブキャリア x パケット x ストリーム]
   │
   │  ★ 手順2: CSI_phase_sanitization_H_estimation.py
   ▼
Tr_vector_<name>_stream_N.txt        位相基準化済みの CFR
   │
   │  ★ 手順3: CSI_phase_sanitization_signal_reconstruction.py
   ▼
processed_phase/<name>_stream_N.mat  csi_matrix_processed
                                     [パケット x サブキャリア x (振幅,位相)]
```

最後の `csi_matrix_processed` は SHARP オリジナルの出力と同じ変数名・同じ意味を持つ
ため、後段 (Doppler 計算など) に接続できる。ただし「[今後の課題](#今後の課題)」の
サンプリング間隔の問題に注意。

## 準備

```bash
pip install numpy scipy osqp h5py
```

`h5py` は MATLAB v7.3 (HDF5) 形式の .mat を読むために必要。デコーダは `-v7.3` で
保存するため実質必須。

## 使い方

### 一番簡単な方法 (手順0〜3を一括実行)

```bash
cd Python_code
python3 run_phase_sanitization.py ../../CSI/202608191054_OpenWrt-A_CSI.mat
```

規格 (VHT / HE) はファイルの中身から自動判定される。結果は
`processed_phase/<入力名>_stream_0.mat` に出る。

活動ラベルごとにサブディレクトリを分けたい場合:

```bash
python3 run_phase_sanitization.py ../../CSI/xxx.mat --name walk01 --label W
# -> processed_phase/W/walk01_stream_0.mat
```

主なオプション:

| オプション | 意味 |
|---|---|
| `--format VHT` / `--format HE` | 使う PHY 形式を明示 (既定はパケット数が最多のもの) |
| `--fcs_only` | FCS 検証済みパケットのみ使う |
| `--start_idx` / `--end_idx` | 先頭・末尾から捨てるパケット数 |
| `--subcarriers_space` | サブキャリアの間引き幅 (既定は規格ごとの推奨値) |
| `--delta_t_refined` | 精細グリッドの時間刻み [s] |

### 段階ごとに実行する方法

SHARP オリジナルと同じ引数体系を保っている (末尾に本実装固有のオプションを追加)。

```bash
cd Python_code

# 手順0: SHARP 形式へ変換
python3 usrp_to_sharp.py ../../CSI/xxx_CSI.mat ./input_files/exp01.mat

# 手順1: 前処理  <dir> <all_dir> <name> <nss> <ncore> <start_idx>
python3 CSI_phase_sanitization_signal_preprocessing.py ./input_files/ 0 exp01 1 1 0

# 手順2: 多重波推定と位相の基準化  ... <start_r> <end_r>
python3 CSI_phase_sanitization_H_estimation.py ./input_files/ 0 exp01 1 1 0 -1

# 手順3: 再構成  <dir> <dir_save> <nss> <ncore> <start_idx> <end_idx>
python3 CSI_phase_sanitization_signal_reconstruction.py ./phase_processing/ ./processed_phase/ 1 1 0 0
```

SISO・アンテナ1本なので `nss=1`, `ncore=1` を指定する
(オリジナルの SHARP は Nexmon の 4 コア構成で `ncore=4` だった)。

### MATLAB から変換する場合

Python を使わず MATLAB 内で手順0 を済ませたい場合:

```matlab
addpath('SHARP-USRP/matlab');
usrpCSItoSHARP('CSI/202608191054_OpenWrt-A_CSI.mat', ...
               'SHARP-USRP/Python_code/input_files/exp01.mat');
```

その後、手順1 以降を Python で実行する。MATLAB 版と Python 版の出力が
ビット単位で一致することはテストで確認している (後述)。

> **注意**: MATLAB 側は必ず `-v7` 形式で保存している。Python の
> `scipy.io.loadmat` は v7.3 (HDF5) を読めないため、ここを `-v7.3` にすると
> 手順1 で失敗する。

## 規格ごとの構成

```bash
python3 Python_code/wifi_config.py   # 一覧を表示
```

| | VHT20 (Wi-Fi 5) | HE20 (Wi-Fi 6) | 参考: SHARP オリジナル (80MHz VHT) |
|---|---|---|---|
| FFT 長 | 64 | 256 | 256 |
| サブキャリア間隔 Δf | 312.5 kHz | 78.125 kHz | 312.5 kHz |
| 使用サブキャリア | 56 本 (k=±1..28) | 242 本 (k=±2..122) | 242 本 |
| ヌル (ガード+DC) | 8 本 | 14 本 | 14 本 |
| 再構成の出力幅 | 57 | 245 | 245 |
| 遅延分解能 (=1/帯域) | 50 ns | 50 ns | 12.5 ns |
| 最大非曖昧遅延 (=1/Δf) | 3.2 µs | 12.8 µs | 3.2 µs |

**HE20 は FFT 長・ヌル配置がオリジナル SHARP の 80MHz VHT と完全に同一**
(どちらも 256 点、ヌルは `[0..5, 127,128,129, 251..255]`)。つまり Wi-Fi 6 20MHz は
オリジナルのパラメータがほぼそのまま通用する、都合の良いケースである。

## SHARP オリジナルからの変更点

| 変更 | 理由 |
|---|---|
| FFT 長・Δf・ヌル配置を `wifi_config.py` に分離 | オリジナルは 80MHz 決め打ち (`F_frequency = 256`, `delete_idxs` 直書き) |
| 再構成の切り出しを `[6:-5]` 固定から一般化 | 帯域によって使用サブキャリアの範囲が変わるため |
| ストリーム数を固定 4 から `n_ss * n_core` へ | オリジナルは `for stream in range(0, 4)` と Nexmon の 4 コア前提。SISO では 1 |
| Nexmon 固有の符号反転を既定で無効化 | `signal_stream[:, 64:] = -signal_stream[:, 64:]` は Nexmon の CSI 規約に対する補正。MATLAB WLAN Toolbox のチャネル推定値には不要 (`--nexmon_sign_flip` で有効化可) |
| `subcarriers_space` / `delta_t_refined` を規格ごとの既定値 + CLI 指定に | 下記「20MHz での注意」を参照 |
| 出力先サブディレクトリを `--label` で指定 | オリジナルはファイル名先頭3文字を活動ラベルとして使う実装。USRP のファイル名は日時始まりのため不適合 |
| `np.linalg.lstsq` に `rcond=None` を明示 | 新しい NumPy での警告回避 |

### 20MHz での注意

位相サニタイゼーションの最適化は「観測 = 使用サブキャリア数」に対して
「未知数 = 遅延グリッドの列数」を解く問題である。80MHz (242 本) 前提の既定値を
20MHz にそのまま持ち込むと、VHT20 では観測が 56 本しかないため劣決定が過度になる。
そこで規格ごとに既定値を変えている:

| | VHT20 | HE20 |
|---|---|---|
| `subcarriers_space` (間引き) | **1** (全 56 本を使用) | 2 (オリジナル同様) |
| `delta_t_refined` (精細グリッド刻み) | **10 ns** | 5 ns (オリジナル同様) |
| 結果の観測数 vs 精細グリッド列数 | 56 vs 45 | 121 vs 90 |

VHT20 で刻みを 10 ns にしているのは、20MHz の遅延分解能が 50 ns である以上、
5 ns 刻みは分解能を超えて細かすぎるという理由もある。いずれも
`--subcarriers_space` / `--delta_t_refined` で上書きできる。

**帯域による本質的な制約**: 20MHz の遅延分解能は 50 ns (= 距離にして約 15 m)
であり、80MHz の 12.5 ns に比べて 4 倍粗い。位相オフセットの除去自体は問題なく
機能するが、近接した多重波の分離能力は原理的に劣る。これは実験条件が 20MHz で
ある以上避けられない。

## 検証

### 1. 変換の可逆性

`fftshift` で中心寄せに戻したとき、使用サブキャリアの値が元の CSI と
**完全一致** (最大絶対差 0)、ヌル位置は厳密に 0。

### 2. 実データでの位相サニタイゼーション効果

`CSI/202608191054_OpenWrt-A_CSI.mat` (Wi-Fi 5, 20MHz, Ch36, 850 パケット, 5 秒):

| | パケット間の位相差のばらつき |
|---|---|
| 生 CSI | 1.96 rad (ほぼランダム) |
| サニタイズ後 | **0.16 rad** (約 1/12) |

平均振幅プロファイルの相関は 0.9992 で、振幅情報は保たれている。
残る 0.16 rad は実際のチャネル変動と雑音によるもの。

処理時間は 850 パケットで約 13 秒。

### 3. 合成データでの厳密な確認 (VHT20 / HE20 両方)

既知の静的多重波にパケットごとのランダム位相・時刻オフセットを載せた合成データでは、
真のチャネルが静的なので理想的にはばらつきが 0 になるはず:

| | 位相ばらつきの改善 | 振幅相関 |
|---|---|---|
| VHT20 | 105 倍 (1.758 → 0.017 rad) | 0.9999 |
| HE20 | 125 倍 (1.765 → 0.014 rad) | 1.0000 |

```bash
python3 Python_code/tests/test_pipeline.py
```

### 4. MATLAB 版と Python 版の一致

同じ入力に対して両者の `csi_buff` がビット単位で一致することを確認
(実データ・合成データとも最大絶対差 0)。GNU Octave で実行するため MATLAB 本体は不要。

```bash
python3 Python_code/tests/test_matlab_parity.py
```

## 今後の課題

### パケット間隔が等間隔でない問題

これは位相サニタイゼーションの**後段**、Doppler 解析に進む際の課題である。

SHARP の Doppler 計算は、Nexmon が一定レートで CSI を取得することを前提に
**パケット列が等間隔にサンプリングされている**ものとして扱う。一方 USRP は
AP が実際に送信したパケットを受動的に捉えるため、間隔が一定にならない。

実測例 (`202608191054`): 平均 170 pkt/s だが、間隔は 0.60 ms 〜 12.06 ms、
変動係数 0.15。

対処の方向:

1. AP 側から一定レートでトラフィックを生成する (ping の間隔固定など)
2. 変換時に記録している `time_sec` を使い、Doppler 計算の前に等間隔へ再標本化する
3. 非等間隔のまま扱える時間周波数解析に置き換える

手順0 は `time_sec` を出力ファイルに保存し、変動係数が 0.2 を超える場合は警告を
出すようにしてある。位相サニタイゼーション自体はパケット単位の処理なので、
この問題の影響を受けない。

### Wi-Fi 6 の実データでの確認

HE20 の経路は合成データでは検証済みだが、実際に USRP で取得した Wi-Fi 6 の
CSI ではまだ確認していない (現時点の `CSI/` にあるのは Wi-Fi 5 のデータのみ)。
`decodeIQ_HE.m` は復号できたパケットからサブキャリア番号を実測して
`subcarrierIndicesHE20` を上書きする実装になっているため、実データを取得したら
手順0 が構成の不一致を検出しないか (エラーメッセージが出ないか) を確認すること。

## ファイル構成

```
SHARP-USRP/
├── README.md
├── LICENSE                     GPL v3 (SHARP の派生のため)
├── matlab/
│   └── usrpCSItoSHARP.m        手順0 の MATLAB 版
└── Python_code/
    ├── wifi_config.py          規格ごとの OFDM 構成定義
    ├── usrp_to_sharp.py        手順0: 形式変換
    ├── CSI_phase_sanitization_signal_preprocessing.py    手順1
    ├── CSI_phase_sanitization_H_estimation.py            手順2
    ├── CSI_phase_sanitization_signal_reconstruction.py   手順3
    ├── run_phase_sanitization.py   手順0〜3の一括実行
    ├── optimization_utility.py     SHARP 由来 (無変更)
    ├── pipeline_meta.py            段間の設定引き継ぎ
    └── tests/
        ├── make_synthetic_usrp_mat.py  合成データ生成
        ├── test_pipeline.py            パイプライン全体のテスト
        └── test_matlab_parity.py       MATLAB/Python 一致テスト
```

## ライセンス

本リポジトリは SHARP (Francesca Meneghello, GNU GPL v3) の派生物であり、
同じく **GNU GPL v3** で配布する。`optimization_utility.py` はオリジナルの
コードをそのまま利用している。

参考文献:

- F. Meneghello et al., "SHARP: Environment and Person Independent Activity
  Recognition with Commodity IEEE 802.11 Access Points," IEEE Transactions on
  Mobile Computing, 2022.
- F. Meneghello et al., "Toward Integrated Sensing and Communications in
  IEEE 802.11bf Wi-Fi Networks," IEEE Communications Magazine, 2023.
- オリジナル実装: https://github.com/francescamen/SHARP ,
  https://github.com/francescamen/SHARPax

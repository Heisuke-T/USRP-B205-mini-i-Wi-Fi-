# SHARP / SHARPax からの変更点

SHARP-USRP が、オリジナルの [SHARP](https://github.com/francescamen/SHARP) および
[SHARPax](https://github.com/francescamen/SHARPax) から何をどう変えたかの一覧。

対象は位相サニタイゼーション (手順1〜3) とドップラーマップ生成 (手順4〜5)。

---

## 0. 前提: なぜ変更が必要だったか

オリジナルは次の測定系を前提に書かれている。

| | SHARP | SHARPax | **SHARP-USRP (本実装)** |
|---|---|---|---|
| 受信機 | Nexmon CSI (ASUS RT-AC86U) | AX-CSI (ASUS RT-AX86U) | **USRP B205mini-i + MATLAB** |
| 規格 | 802.11ac (VHT) | 802.11ax (HE) | **VHT と HE の両方** |
| 帯域 | 80 MHz | 80 MHz | **20 MHz** |
| FFT 長 | 256 | 1024 | **64 (VHT20) / 256 (HE20)** |
| 受信コア | 4 | 4 | **1 (SISO)** |
| 取得間隔 | 一定 (6 ms) | 一定 (7.5 ms) | **不定 (受動受信)** |

変更は大きく 4 つの理由に分類できる。

- **(A) 帯域の一般化** — 80MHz 決め打ちだったものを 20MHz でも動くようにする
- **(B) SISO 対応** — 4 コア前提だったものをアンテナ 1 本で動くようにする
- **(C) USRP 固有** — Nexmon 固有の補正の除去、取得間隔が不定であることへの対応
- **(D) 実装上の改善** — 新しい NumPy での警告回避など

---

## 1. ファイル別の変更点一覧

| ファイル | 由来 | 変更 |
|---|---|---|
| `optimization_utility.py` | SHARP | **無変更** (そのまま利用) |
| `wifi_config.py` | 新規 | 規格ごとの OFDM 構成を集約 (A) |
| `pipeline_meta.py` | 新規 | 段間で規格設定を引き継ぐ (A) |
| `usrp_to_sharp.py` / `usrpCSItoSHARP.m` | 新規 | デコーダ出力 → SHARP 入力形式の変換 (C) |
| `CSI_phase_sanitization_signal_preprocessing.py` | SHARP | (A)(B)(C) |
| `CSI_phase_sanitization_H_estimation.py` | SHARP | (A)(B) |
| `CSI_phase_sanitization_signal_reconstruction.py` | SHARP | (A)(C)(D) |
| `CSI_doppler_computation.py` | SHARP | (A)(C) |
| `CSI_doppler_plot.py` | SHARP (plots_utility) | 軸を実測値から決める (C) |

---

## 2. 変更していない部分（アルゴリズムの核）

以下は**意図的に一切変えていない**。位相サニタイゼーションとドップラー計算の
アルゴリズムそのものはオリジナルと同一である。

- `optimization_utility.py` の `build_T_matrix` と `lasso_regression_osqp_fast`
  (スパース多重波推定の OSQP 定式化、正則化係数 `lambd = 1E-1` を含む)
- 2 段階の遅延推定 (粗いグリッド → 最強経路の周辺を精細グリッドで解き直す)
- **最強経路の複素共役を掛けて位相基準を揃える処理** — サニタイゼーションの本体
  ```python
  Trr = np.multiply(Tr, np.conj(Tr[:, position_max_r_refined:position_max_r_refined + 1]))
  ```
- 位相のアンウラップと 2π 跳びの補正、最小二乗による残留位相傾斜の除去
- 探索する遅延範囲 `t_min = -3E-7`, `t_max = 5E-7`、粗グリッド `delta_t = 1E-7`、
  精細化の範囲 `range_refined_down = 2E-7`, `range_refined_up = 2.5E-7`
- ドップラー計算の手順: ハン窓 → FFT (n=100) → fftshift → パワー →
  サブキャリア方向に加算 → 窓ごとに最大値で正規化 → 雑音床でクリップ

---

## 3. 位相サニタイゼーションの変更点

### 3.1 サブキャリア構成のパラメータ化 (A)

オリジナルは FFT 長・ヌルサブキャリア番号・切り出し範囲が各スクリプトに
直書きされていた。これを `wifi_config.py` に集約した。

**手順1 (前処理)**

```python
# SHARP オリジナル (80MHz 決め打ち)
delete_idxs = np.asarray([0, 1, 2, 3, 4, 5, 127, 128, 129,
                          251, 252, 253, 254, 255], dtype=int)
```
```python
# SHARP-USRP (規格から導出)
signal_stream = np.delete(signal_stream, cfg.delete_idxs, axis=1)
```

**手順2 (H 推定)**

```python
# SHARP オリジナル
F_frequency = 256
delta_f = 312.5E3
delete_idxs = np.asarray([0, 1, ..., 255], dtype=int)
```
```python
# SHARP-USRP
frequency_vector_complete = cfg.frequency_vector_complete()
frequency_vector = cfg.frequency_vector()
```

**手順3 (再構成)**

```python
# SHARP オリジナル
F_frequency = 256
csi_matrix_processed[:, 6:-5, 0] = np.abs(H_est[6:-5, :]).T
```
```python
# SHARP-USRP (使用帯域の下端〜上端を一般化)
H_crop = H_est[cfg.crop_slice, :]
csi_matrix_processed[:, :, 0] = np.abs(H_crop).T
```

`[6:-5]` は「最低位の使用サブキャリアから最高位まで (間の DC ヌルは 0 のまま残す)」
の意味であり、`crop_slice` はこれを一般化したもの。

導出される値:

| | VHT20 | HE20 | 参考: SHARP 80MHz |
|---|---|---|---|
| FFT 長 | 64 | 256 | 256 |
| Δf | 312.5 kHz | 78.125 kHz | 312.5 kHz |
| 使用サブキャリア | 56 (k=±1..28) | 242 (k=±2..122) | 242 |
| `delete_idxs` | `[0,1,2,3, 32, 61,62,63]` | `[0..5, 127,128,129, 251..255]` | 同左 |
| 切り出し範囲 | `[4:61]` → 57 | `[6:251]` → 245 | `[6:-5]` → 245 |

> **HE20 のヌル配置と切り出し範囲は SHARP の 80MHz VHT と完全に同一**。
> どちらも 256 点 FFT のため。Wi-Fi 6 20MHz はオリジナルの構成がそのまま通用する。

### 3.2 ストリーム数の固定 4 を解除 (B)

手順1 は元から `n_ss * n_core` で書かれていたが、**手順2 だけが 4 に固定**されていた。

```python
# SHARP オリジナル / SHARPax 共通 (どちらも 4 固定)
for stream in range(0, 4):
```
```python
# SHARP-USRP
for stream in range(n_tot):     # n_tot = n_ss * n_core、SISO では 1
```

固定のままだと SISO (`n_tot=1`) で `signal_complete[:, :, 1]` にアクセスして
IndexError になる。

### 3.3 Nexmon 固有の符号反転を既定で無効化 (C)

```python
# SHARP オリジナル (常に実行)
signal_stream[:, 64:] = - signal_stream[:, 64:]
```
```python
# SHARP-USRP (既定では実行しない)
if nexmon_sign_flip:
    signal_stream[:, cfg.half:] = -signal_stream[:, cfg.half:]
```

これは Nexmon CSI の符号規約に対する補正であり、MATLAB WLAN Toolbox の
`wlanVHTLTFChannelEstimate` などが返すチャネル推定値には不要。
`--nexmon_sign_flip` で有効化できる。

### 3.4 最適化パラメータを 20MHz 向けに調整 (A)

サブキャリア数が減ると、最適化の「観測数」に対して「未知数 (遅延グリッドの列数)」
が相対的に多くなり、劣決定が過度になる。

```python
# SHARP オリジナル (80MHz / 242 本 前提)
subcarriers_space = 2
delta_t_refined = 5E-9
```

| | 観測数 | 精細グリッド列数 | 判定 |
|---|---|---|---|
| SHARP 80MHz (242本, 間引き2) | 121 | 90 | 妥当 |
| VHT20 に既定値を適用 (56本, 間引き2) | 28 | 90 | **劣決定が過度** |
| **VHT20 (本実装: 間引き1, 刻み10ns)** | **56** | **45** | 妥当 |
| **HE20 (本実装: 既定値のまま)** | **121** | **90** | 妥当 |

VHT20 で `delta_t_refined` を 10 ns にしたのは、20MHz の遅延分解能が 50 ns
である以上 5 ns 刻みは分解能を超えて細かすぎる、という理由もある。
いずれも `--subcarriers_space` / `--delta_t_refined` で上書きできる。

### 3.5 出力先サブディレクトリの決め方 (C)

```python
# SHARP オリジナル (ファイル名の先頭3文字を活動ラベルとみなす)
sub_dir_name = name_f[0:3]
```
```python
# SHARP-USRP (--label で明示、既定はサブディレクトリを作らない)
subdir = save_dir if label is None else path.join(save_dir, label)
```

SHARP のデータセットは `E1_`, `W2_` のような活動ラベル始まりのファイル名だが、
USRP のファイル名は `202608191054_...` と日時始まりのため、先頭3文字が `202` に
なってしまう。

### 3.6 パケット取得時刻の持ち回り (C, 新規追加)

オリジナルには無い。手順4 で実測のサンプリング間隔を使うために必要。

- 手順0: `time_sec` を SHARP 入力 .mat に保存
- 手順1: ストリームごとの間引き・切り出しと同じ操作を時刻にも適用し `time_<name>.npy` に保存
- 手順3: 手順2 の `[start_r:end_r]` と手順3 の `[start_idx:末尾-end_idx]` を
  同じ順序で時刻にも適用し、`csi_matrix_processed` と一緒に `.mat` へ保存

### 3.7 NumPy の警告回避 (D)

```python
# SHARP オリジナル
temp2 = np.linalg.lstsq(ones_vector.T, error)[0]
```
```python
# SHARP-USRP
temp2 = np.linalg.lstsq(ones_vector.T, error, rcond=None)[0]
```

また、手順2 で未使用の `from plots_utility import *` (matplotlib に依存) を外し、
必要な関数だけを import するようにした。

---

## 4. ドップラーマップの変更点

### 4.1 `--bandwidth` / `--sub_band` の廃止 (A) ← 最も重要

**オリジナルの `--bandwidth 20` は「80MHz のデータから 20MHz を切り出す」オプション**
であり、245 列 (80MHz 分) の配列に対する固定インデックスである。

```python
# SHARP オリジナル
elif bandwidth == 20:
    if sub_band == 1:
        selected_subcarriers_idxs = np.arange(0, 57, 1)
    elif sub_band == 2:
        selected_subcarriers_idxs = np.arange(60, 117, 1)
    ...
    csi_matrix_complete = csi_matrix_complete[:, selected_subcarriers_idxs]
```

すでに 20MHz である本システムの出力に適用すると:

| 入力 | `--bandwidth 20 --sub_band 1` の結果 |
|---|---|
| VHT20 (57 列) | `arange(0,57)` がたまたま全域と一致して動くが**偶然**。`--sub_band 2` 以降は範囲外エラー |
| HE20 (245 列) | **20MHz のさらに 1/4 だけを切り出す誤り。列数が SHARP の 80MHz と同じ 245 のためエラーが出ず気付けない** |

正しく使うには「切り出しをスキップする」意味の `--bandwidth 80` を指定する必要が
あり、直感に反する。そこで本実装ではこのオプション自体を廃止し、既定で全サブキャリアを
使う。部分帯域を見たい場合は `--subcarrier_range LO HI` で明示する。

### 4.2 Tc / fc のハードコードをやめる (C)

```python
# SHARP オリジナル
Tc = 6e-3
fc = 5e9
```
```python
# SHARPax オリジナル
Tc = 7.5e-3
fc = 5785e6
```
```python
# SHARP-USRP (実測から決める。--Tc / --fc で上書き可)
dt = np.diff(np.sort(time_sec))
Tc = float(np.median(dt))          # 手順3 が保存した time_sec から
# fc は --fc (既定 5.18e9 = Ch36)
```

USRP は受動受信のため取得間隔が測定ごとに異なる。固定値のままだと速度軸の
目盛りがずれる (実測 Tc≈5.997 ms vs 6 ms、fc=5.18 GHz vs 5 GHz)。

### 4.3 速度軸の計算を明示的にした (D)

オリジナルは速度軸の目盛りを `delta_v = c / (Tc · fc · feature_length)` で
与えていた。本実装では、ゼロ詰めを含む FFT のビン数から決まる**ビン間隔**と、
窓長から決まる**分解能**を区別して両方記録する。

```python
delta_v_bin = V_LIGHT / (Tc * fc * n_fft)          # 軸の目盛り (ビン間隔)
delta_v_res = V_LIGHT / (Tc * fc * sample_length)  # 分解能 (ゼロ詰めでは向上しない)
v_max       = V_LIGHT / (Tc * fc) / 2              # 一意に測れる上限
```

これらは `_meta.json` に保存され、描画時にそのまま使われる。

### 4.4 非等間隔サンプリングへの対応 (C)

STFT は等間隔サンプリングを前提とするが、USRP は AP が実際に送信したパケットしか
捉えられないため間隔が揺れる (実測: 中央値 6.00 ms、0.60〜12.06 ms、変動係数 0.15)。

`--resample {none, nudft, interp}` を追加した。**既定は `none`** (オリジナルと同じ扱い)。

実測の時刻列を模した合成データでの比較 (真の速度 ±4 ビンへのパワー集中度、高いほど良い):

| ドップラー速度 | 理想(等間隔) | `none` (既定) | `nudft` | `interp` (線形補間) |
|---|---|---|---|---|
| 0.5 m/s | 0.595 | **0.592** | 0.547 | 0.592 |
| 1.0 m/s | 0.330 | **0.323** | 0.303 | 0.309 |
| 2.0 m/s | 0.330 | 0.302 | 0.302 | 0.247 |
| 3.0 m/s | 0.330 | 0.277 | **0.306** | 0.165 |

- 2 m/s 程度まで (人の歩行) は `none` で理想とほぼ同等 → **既定を `none` にした根拠**
- 3 m/s 付近では `none` が理想の 84% まで落ち、`nudft` (非等間隔DFT) が取り戻す
- `interp` (線形補間で等間隔化) は補間が低域通過フィルタとして働くため
  高速側で明確に悪化する → **既定にしてはいけない**

`nudft` は実測時刻をそのまま指数に入れる非等間隔 DFT。周波数グリッドは等間隔 FFT と
同じに取るため速度軸の意味は変わらず、等間隔サンプリングなら通常の FFT と一致する。

```python
f_grid = (np.arange(n_fft) - n_fft // 2) / (Tc * n_fft)
w = 0.5 * (1 - np.cos(2 * np.pi * t / span))   # ハン窓を時間位置で与える
w = w * np.gradient(t) / np.gradient(t).mean() # 標本密度の偏りを補正
prof = np.exp(-1j * 2 * np.pi * np.outer(f_grid, t)) @ (x * w[:, None])
```

実データでは `none` と `nudft` の結果は相関 0.9996 で一致した。

---

## 5. 新規に追加したもの (オリジナルに対応物が無い)

| ファイル | 役割 |
|---|---|
| `usrp_to_sharp.py` / `matlab/usrpCSItoSHARP.m` | デコーダ出力 (使用サブキャリアのみ) を SHARP が前提とする FFT 長ぶんの `csi_buff` (FFT 順) へ変換。両者の出力はビット単位で一致することを検証済み |
| `wifi_config.py` | 規格ごとの OFDM 構成の一元管理 |
| `pipeline_meta.py` | 段間で規格・切り出し位置を引き継ぐ |
| `run_phase_sanitization.py` | 手順0〜3 の一括実行 |
| `CSI_doppler_plot.py` | ドップラーマップの描画 (軸は `_meta.json` から) |
| `tests/test_pipeline.py` | 変換の可逆性・位相安定化の効果を VHT20/HE20 で検証 |
| `tests/test_doppler.py` | 非等間隔サンプリングの 3 手法を比較検証 |
| `tests/test_matlab_parity.py` | MATLAB 版と Python 版の一致を Octave で検証 |

---

## 6. 検証結果まとめ

| 検証 | 結果 |
|---|---|
| 変換の可逆性 | 使用サブキャリアが元の CSI と完全一致 (最大絶対差 0) |
| 実データの位相サニタイゼーション (850 パケット) | パケット間の位相ばらつき 1.96 rad → 0.16 rad、振幅相関 0.9992 |
| 合成データ (VHT20 / HE20) | 位相安定化 105 倍 / 125 倍、振幅相関 0.9999 / 1.0000 |
| MATLAB 版 vs Python 版 | `csi_buff` がビット単位で一致 |
| ドップラー: `none` vs `nudft` (実データ) | マップ相関 0.9996 |
| 実データのドップラーマップ | 0 m/s に静止成分、0.9/1.65/2.45 秒付近に ±2 m/s の拡がり |

---

## 7. まだ確認していないこと

- **Wi-Fi 6 の実データ** — HE20 の経路は合成データでのみ検証。
  `decodeIQ_HE.m` は復号できたパケットからサブキャリア番号を実測して
  `subcarrierIndicesHE20` を上書きするため、実データ取得後に手順0 が
  構成の不一致を報告しないか確認すること
- **活動認識ネットワークへの接続** — SHARP の `CSI_doppler_create_dataset_*.py`
  以降は未検証。入力サイズを引数で受け取る作りなので 20MHz のサブキャリア数に
  合わせれば流用できる見込み

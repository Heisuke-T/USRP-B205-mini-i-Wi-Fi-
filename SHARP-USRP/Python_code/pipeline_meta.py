"""
    パイプライン各段の間で規格設定を受け渡すための補助モジュール。

    SHARP オリジナルは 80MHz 決め打ちだったため各スクリプトが独立に
    パラメータを持てたが、本実装は規格 (VHT20 / HE20) によって FFT 長や
    ヌル配置が変わる。前処理の段で判定した設定を副次ファイルに残し、
    後段がそれを引き継ぐ。

    Copyright (C) 2026 Heisuke Takeda
    Released under the GNU GPL v3 (see LICENSE).
"""

import json
from os import path


def meta_path(out_dir, name):
    return path.join(out_dir, 'meta_' + name + '.json')


def save_meta(out_dir, name, **fields):
    with open(meta_path(out_dir, name), 'w', encoding='utf-8') as fp:
        json.dump(fields, fp, ensure_ascii=False, indent=2)


def load_meta(out_dir, name):
    """副次ファイルを読む。無ければ空 dict。"""
    p = meta_path(out_dir, name)
    if not path.exists(p):
        return {}
    with open(p, encoding='utf-8') as fp:
        return json.load(fp)

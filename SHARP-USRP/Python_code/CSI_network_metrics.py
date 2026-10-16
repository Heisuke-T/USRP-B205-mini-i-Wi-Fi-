"""
    保存した評価結果の表示

    CSI_network.py / CSI_network_test.py が outputs/ に保存した指標を読み、
    正解率・F 値・混同行列を表示する。SHARP は 5 クラス・4 アンテナ決め打ちの
    表示だったため、任意のクラス数・アンテナ数に一般化した。

    例:
      python CSI_network_metrics.py complete_different_antenna1,antenna2_E,W,R_S2,S3 E,W,R

    Copyright (C) 2026 Heisuke Takeda
    Based on SHARP (C) 2022 Francesca Meneghello, GNU GPL v3.
    Released under the GNU GPL v3 (see LICENSE).
"""

import argparse
import os
import pickle

from metrics_utility import print_metrics


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('name_file', help='outputs/ 内の結果ファイル名 (.txt を除く)')
    parser.add_argument('activities', help='活動ラベル (カンマ区切り)')
    args = parser.parse_args()

    with open(os.path.join('./outputs', args.name_file + '.txt'), 'rb') as fp:
        metrics = pickle.load(fp)

    by_num_ant = None
    name_ant = os.path.join('./outputs', 'change_number_antennas_' + args.name_file + '.txt')
    if os.path.exists(name_ant):
        with open(name_ant, 'rb') as fp:
            by_num_ant = pickle.load(fp)

    print_metrics(metrics, args.activities.split(','), by_num_ant)


if __name__ == '__main__':
    main()

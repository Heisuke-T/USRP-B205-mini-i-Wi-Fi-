"""
    学習済みネットワークのテスト (学習に使っていない測定で評価する)

    CSI_doppler_create_dataset_test.py で作ったデータセットを、CSI_network.py
    で学習したモデルに通して評価する。

    例 (S1 で学習したモデルを S2, S3 で評価。アンテナ 2 本):
      python CSI_network_test.py ./doppler_traces/ S2,S3 100 340 1 32 antenna1,antenna2 usrp E,W,R

    S2,S3 のように並べるとまとめて 1 つの評価になる。別々に見たいときは
    S2 と S3 で 1 回ずつ実行する。

    モデルは既定で <name_base>_<アンテナ>_<活動>_network.keras を使う。
    ネットワークはアンテナ 1 本ぶんの入力を受け取るので、アンテナ 2 本で学習した
    モデルを antenna1 だけで評価する (またはその逆) こともできる。その場合は
    --model でモデルを指定する。

    Copyright (C) 2026 Heisuke Takeda
    Based on SHARP (C) 2022 Francesca Meneghello, GNU GPL v3.
    Released under the GNU GPL v3 (see LICENSE).
"""

import argparse
import os
import pickle

import numpy as np
import tensorflow as tf

from dataset_utility import (check_window_shape, create_dataset_single,
                             expand_antennas, parse_antennas, read_set,
                             remove_files, set_name)
from metrics_utility import (evaluate, print_metrics,
                             scores_by_number_of_antennas)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dir', help='データセットのディレクトリ (ドップラーデータと同じ)')
    parser.add_argument('subdirs', help='テストに使うサブディレクトリ (カンマ区切り。例: S2,S3)')
    parser.add_argument('feature_length', help='特徴量方向の長さ (学習時と同じ)', type=int)
    parser.add_argument('sample_length', help='時間方向の長さ (学習時と同じ)', type=int)
    parser.add_argument('channels', help='チャネル数 (1)', type=int)
    parser.add_argument('batch_size', help='バッチサイズ', type=int)
    parser.add_argument('antennas', help='使うアンテナ (例: antenna1,antenna2 / antenna1)。本数でも可')
    parser.add_argument('name_base', help='モデルなどのファイル名の先頭 (学習時と同じ)')
    parser.add_argument('activities', help='活動ラベル (学習時と同じもの)')
    parser.add_argument('--model', default=None,
                        help='使うモデル (既定: <name_base>_<アンテナ>_<活動>_network.keras)')
    args = parser.parse_args()

    gpus = tf.config.list_physical_devices('GPU')
    print(gpus)

    csi_act = args.activities
    activities = np.asarray(csi_act.split(','))
    antennas = parse_antennas(args.antennas)
    num_antennas = len(antennas)
    prefix = f'{args.name_base}_{",".join(antennas)}_{csi_act}'
    remove_files(prefix + '_cache_complete*')

    labels_complete, files_complete = [], []
    for sdir in args.subdirs.split(','):
        lab, fil = read_set(os.path.join(args.dir, sdir), set_name('complete', antennas, csi_act))
        labels_complete.extend(lab)
        files_complete.extend(fil)
    if not files_complete:
        raise ValueError('テストデータが 0 件です。データセット作成時の出力を確認してください。')

    sample_length = args.sample_length
    feature_length = args.feature_length
    input_network = (sample_length, feature_length, args.channels)
    output_shape = activities.shape[0]
    labels_considered = np.arange(output_shape)
    check_window_shape(files_complete[0], num_antennas, feature_length, sample_length)

    lab, count = np.unique(labels_complete, return_counts=True)
    print(f'complete: {len(files_complete)} 窓 x {num_antennas} アンテナ  ' +
          ', '.join(f'{activities[la]} {c}' for la, c in zip(lab, count)))

    files_exp, labels_exp, stream_ant = expand_antennas(files_complete, labels_complete, num_antennas)
    dataset_csi_complete = create_dataset_single(files_exp, labels_exp, stream_ant, input_network,
                                                 args.batch_size, shuffle=False,
                                                 cache_file=prefix + '_cache_complete', repeat=False)

    name_model = args.model or prefix + '_network.keras'
    csi_model = tf.keras.models.load_model(name_model)
    print(f'モデル: {name_model}')

    complete_prediction_list = csi_model.predict(dataset_csi_complete)
    metrics_matrix_dict = evaluate(complete_prediction_list, labels_complete, num_antennas, labels_considered)
    by_num_ant = scores_by_number_of_antennas(complete_prediction_list, labels_complete, num_antennas,
                                              labels_considered)
    print_metrics(metrics_matrix_dict, activities, by_num_ant)

    os.makedirs('./outputs', exist_ok=True)
    name_file = f'./outputs/complete_different_{",".join(antennas)}_{csi_act}_{args.subdirs}.txt'
    with open(name_file, "wb") as fp:  # Pickling
        pickle.dump(metrics_matrix_dict, fp)
    name_file_ant = (f'./outputs/change_number_antennas_complete_different_'
                     f'{",".join(antennas)}_{csi_act}_{args.subdirs}.txt')
    with open(name_file_ant, "wb") as fp:  # Pickling
        pickle.dump(by_num_ant, fp)
    print(f'\n指標: {name_file}')


if __name__ == '__main__':
    main()

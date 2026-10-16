"""
    活動認識ネットワークの学習

    CSI_doppler_create_dataset_train.py で作った学習・検証用データセットで
    SHARP のネットワークを学習し、同じ測定から取り分けたテスト区間で評価する。
    ネットワークの構造・学習の設定 (Adam 1e-4, 早期終了など) は SHARP と同じ。

    ネットワークはアンテナ 1 本ぶんの入力を受け取る。アンテナが複数あるときは
    各アンテナの窓を別々のサンプルとして学習し、評価のときにアンテナごとの
    予測を統合する (SHARP と同じ)。

    例 (S1 で学習。アンテナ 2 本):
      python CSI_network.py ./doppler_traces/ S1 100 340 1 32 antenna1,antenna2 usrp E,W,R

    出力:
      <name_base>_<アンテナ>_<活動>_network.keras  学習したモデル
      outputs/test_<アンテナ>_<活動>_<subdirs>.txt   テスト区間での指標

    SHARP からの変更点:
      * 引数 num_tot (アンテナ数) を、使うアンテナの指定に置き換えた
      * --bandwidth / --sub_band は出力ファイル名にしか使われていないため廃止
      * 保存形式を .h5 から .keras に変えた (Keras 3 では .h5 は旧形式として警告が出る)
      * 学習後に最終エポックのモデルで上書き保存していたのをやめ、検証正解率が
        最良だったエポックのモデルで評価する
      * アンテナ本数ごとの評価を、4 本決め打ちから任意の本数に一般化した

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
from network_utility import csi_network_inc_res


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('dir', help='データセットのディレクトリ (ドップラーデータと同じ)')
    parser.add_argument('subdirs', help='学習に使うサブディレクトリ (カンマ区切り。例: S1)')
    parser.add_argument('feature_length', help='特徴量方向の長さ (ドップラービン数、既定の n_fft なら 100)', type=int)
    parser.add_argument('sample_length', help='時間方向の長さ (データセット作成時の windows_length)', type=int)
    parser.add_argument('channels', help='チャネル数 (1)', type=int)
    parser.add_argument('batch_size', help='バッチサイズ', type=int)
    parser.add_argument('antennas', help='使うアンテナ (例: antenna1,antenna2 / antenna1)。本数でも可')
    parser.add_argument('name_base', help='モデルなどのファイル名の先頭')
    parser.add_argument('activities', help='活動ラベル (カンマ区切り。データセット作成時と同じもの)')
    parser.add_argument('--epochs', type=int, default=25,
                        help='最大エポック数 (既定 25, SHARP と同じ。検証損失が 3 エポック'
                             '改善しなければ早めに止まる)')
    args = parser.parse_args()

    gpus = tf.config.list_physical_devices('GPU')
    print(gpus)

    csi_act = args.activities
    activities = np.asarray(csi_act.split(','))
    antennas = parse_antennas(args.antennas)
    num_antennas = len(antennas)
    prefix = f'{args.name_base}_{",".join(antennas)}_{csi_act}'
    remove_files(prefix + '_cache_*')

    sets = {}
    for kind in ('train', 'val', 'test'):
        labels, files = [], []
        for sdir in args.subdirs.split(','):
            lab, fil = read_set(os.path.join(args.dir, sdir), set_name(kind, antennas, csi_act))
            labels.extend(lab)
            files.extend(fil)
        if not files:
            raise ValueError(f'{kind} のデータが 0 件です。データセット作成時の出力を確認してください。')
        sets[kind] = (files, labels)

    sample_length = args.sample_length
    feature_length = args.feature_length
    input_network = (sample_length, feature_length, args.channels)
    batch_size = args.batch_size
    output_shape = activities.shape[0]
    labels_considered = np.arange(output_shape)
    check_window_shape(sets['train'][0][0], num_antennas, feature_length, sample_length)

    datasets = {}
    for kind, (files, labels) in sets.items():
        files_exp, labels_exp, stream_ant = expand_antennas(files, labels, num_antennas)
        datasets[kind] = create_dataset_single(files_exp, labels_exp, stream_ant, input_network, batch_size,
                                               shuffle=(kind == 'train'), cache_file=f'{prefix}_cache_{kind}',
                                               repeat=(kind == 'train'))
        lab, count = np.unique(labels, return_counts=True)
        print(f'{kind}: {len(files)} 窓 x {num_antennas} アンテナ  ' +
              ', '.join(f'{activities[la]} {c}' for la, c in zip(lab, count)))

    csi_model = csi_network_inc_res(input_network, output_shape)
    csi_model.summary()

    optimiz = tf.keras.optimizers.Adam(learning_rate=0.0001)

    loss = tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True)
    csi_model.compile(optimizer=optimiz, loss=loss, metrics=[tf.keras.metrics.SparseCategoricalAccuracy()])

    num_samples_train = len(sets['train'][0]) * num_antennas
    train_steps_per_epoch = int(np.ceil(num_samples_train / batch_size))

    callback_stop = tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=3)

    name_model = prefix + '_network.keras'
    callback_save = tf.keras.callbacks.ModelCheckpoint(name_model, save_freq='epoch', save_best_only=True,
                                                       monitor='val_sparse_categorical_accuracy')

    csi_model.fit(datasets['train'], epochs=args.epochs, steps_per_epoch=train_steps_per_epoch,
                  validation_data=datasets['val'], callbacks=[callback_save, callback_stop])

    csi_model = tf.keras.models.load_model(name_model)
    print(f'\nモデル: {name_model}')

    # 同じ測定から取り分けたテスト区間で評価
    test_prediction_list = csi_model.predict(datasets['test'])
    labels_test = sets['test'][1]
    metrics_matrix_dict = evaluate(test_prediction_list, labels_test, num_antennas, labels_considered)
    by_num_ant = scores_by_number_of_antennas(test_prediction_list, labels_test, num_antennas,
                                              labels_considered)
    print_metrics(metrics_matrix_dict, activities, by_num_ant)

    os.makedirs('./outputs', exist_ok=True)
    name_file = f'./outputs/test_{",".join(antennas)}_{csi_act}_{args.subdirs}.txt'
    with open(name_file, "wb") as fp:  # Pickling
        pickle.dump(metrics_matrix_dict, fp)
    name_file_ant = f'./outputs/change_number_antennas_test_{",".join(antennas)}_{csi_act}_{args.subdirs}.txt'
    with open(name_file_ant, "wb") as fp:  # Pickling
        pickle.dump(by_num_ant, fp)
    print(f'\n指標: {name_file}')


if __name__ == '__main__':
    main()

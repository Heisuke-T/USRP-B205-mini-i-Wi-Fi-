"""
    活動認識の評価 (アンテナごとの予測の統合と指標の計算)

    SHARP の CSI_network.py / CSI_network_test.py に同じ処理が重複して書かれて
    いたものをまとめた。規則は SHARP と同じだが、次を一般化している:

      * アンテナの組み合わせ: SHARP は 4 本決め打ち ([0],[1],[2],[3] など) で、
        2 本では存在しない 3 本目を参照して IndexError になる
      * クラス数: SHARP は labels=[0, 1, 2, 3, 4] (5 クラス) 決め打ち

    Copyright (C) 2026 Heisuke Takeda
    Based on SHARP (C) 2022 Francesca Meneghello, GNU GPL v3.
    Released under the GNU GPL v3 (see LICENSE).
"""

import itertools

import numpy as np
from sklearn.metrics import (accuracy_score, confusion_matrix,
                             precision_recall_fscore_support)


def merge_antennas(pred_prob, pred_lab, num_antennas, ants=None):
    """アンテナごとの予測を窓ごとに 1 つの判定へまとめる (SHARP と同じ規則)。

    pred_prob: [窓数*アンテナ数 x クラス]  ネットワークの出力 (窓ごとにアンテナが並ぶ)
    pred_lab:  [窓数*アンテナ数]            その argmax
    ants:      使うアンテナの番号 (既定は全部)

    多数決で 1 位が決まればそのクラス。同票、または票が (使うアンテナ数 - 2) より
    多くのクラスに割れたときは、出力の和が最大のクラスにする。
    """
    if ants is None:
        ants = list(range(num_antennas))
    n = pred_prob.shape[0] // num_antennas
    merged = np.zeros(n, dtype=int)
    for i in range(n):
        prob = pred_prob[i * num_antennas:(i + 1) * num_antennas, :][ants, :]
        lab = pred_lab[i * num_antennas:(i + 1) * num_antennas][ants]
        lab_unique, count = np.unique(lab, return_counts=True)
        if lab_unique.shape[0] == 1:
            merged[i] = lab_unique[0]
            continue
        order = np.flip(np.argsort(count))
        count_sort = count[order]
        if count_sort[0] == count_sort[1] or lab_unique.shape[0] > len(ants) - 2:
            merged[i] = np.argmax(np.sum(prob, axis=0))
        else:
            merged[i] = lab_unique[order[0]]
    return merged


def _scores(labels_true, labels_pred, labels_considered):
    precision, recall, fscore, _ = precision_recall_fscore_support(
        labels_true, labels_pred, labels=labels_considered, zero_division=0)
    return (confusion_matrix(labels_true, labels_pred, labels=labels_considered),
            accuracy_score(labels_true, labels_pred), precision, recall, fscore)


def evaluate(pred_prob, labels_true, num_antennas, labels_considered):
    """アンテナ単体と、アンテナ統合後の指標を求める。

    labels_true: 窓ごとの正解 (アンテナで展開する前のもの)
    戻り値は SHARP と同じキーの辞書。
    """
    pred_lab = np.argmax(pred_prob, axis=1)
    labels_true = np.asarray(labels_true)
    labels_true_expanded = np.repeat(labels_true, num_antennas)

    conf, acc, prec, rec, fs = _scores(labels_true_expanded, pred_lab, labels_considered)
    merged = merge_antennas(pred_prob, pred_lab, num_antennas)
    conf_m, acc_m, prec_m, rec_m, fs_m = _scores(labels_true, merged, labels_considered)

    return {'conf_matrix': conf,
            'accuracy_single': acc,
            'precision_single': prec,
            'recall_single': rec,
            'fscore_single': fs,
            'conf_matrix_max_merge': conf_m,
            'accuracy_max_merge': acc_m,
            'precision_max_merge': prec_m,
            'recall_max_merge': rec_m,
            'fscore_max_merge': fs_m}


def scores_by_number_of_antennas(pred_prob, labels_true, num_antennas, labels_considered):
    """使うアンテナの本数ごとに、全組み合わせで平均した正解率と F 値。"""
    pred_lab = np.argmax(pred_prob, axis=1)
    acc = np.zeros((num_antennas,))
    fs = np.zeros((num_antennas,))
    for k in range(1, num_antennas + 1):
        combos = list(itertools.combinations(range(num_antennas), k))
        for ants in combos:
            merged = merge_antennas(pred_prob, pred_lab, num_antennas, list(ants))
            _, a, _, _, f = _scores(labels_true, merged, labels_considered)
            acc[k - 1] += a
            fs[k - 1] += np.mean(f)
        acc[k - 1] /= len(combos)
        fs[k - 1] /= len(combos)
    return {'average_accuracy_change_num_ant': acc,
            'average_fscore_change_num_ant': fs}


def print_metrics(metrics, activities, by_num_ant=None):
    for title, key in (('アンテナ単体', 'single'), ('アンテナ統合 (最終判定)', 'max_merge')):
        conf = metrics['conf_matrix' if key == 'single' else 'conf_matrix_max_merge']
        fscore = metrics['fscore_' + key]
        print(f'\n-- {title} --')
        print(f'正解率 {metrics["accuracy_" + key]:.4f}, '
              f'平均適合率 {np.mean(metrics["precision_" + key]):.4f}, '
              f'平均再現率 {np.mean(metrics["recall_" + key]):.4f}, '
              f'平均 F 値 {np.mean(fscore):.4f}')
        print('F 値: ' + ', '.join(f'{a} {f:.3f}' for a, f in zip(activities, fscore)))
        print('混同行列 (行: 正解, 列: 予測)  ' + ' '.join(activities))
        print(conf)
    if by_num_ant is not None:
        acc = by_num_ant['average_accuracy_change_num_ant']
        fs = by_num_ant['average_fscore_change_num_ant']
        print('\n-- 使うアンテナの本数ごと (全組み合わせの平均) --')
        for k in range(len(acc)):
            print(f'{k + 1} 本: 正解率 {acc[k]:.4f}, 平均 F 値 {fs[k]:.4f}')

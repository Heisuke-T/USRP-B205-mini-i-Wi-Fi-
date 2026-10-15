%% alignCSI.m
% =========================================================================
%  2 台の受信機 (antenna1 / antenna2) の CSI の時間軸を揃え、
%  両方がそろっている区間から 80 秒 (既定) を切り出して保存する
% -------------------------------------------------------------------------
%  使う場面:
%    2 台の USRP をそれぞれ別の PC で captureIQ.m などで回すと、開始時刻が
%    数秒ずれる。また各 PC の時計も合っていないので、記録された時刻からは
%    揃えられない。
%
%  揃え方 (パケットの到着時刻の突き合わせ):
%    2 台は同じ AP・同じクライアントの同じパケットを受信している。各パケットの
%    到着時刻の並び方は、送信のたびの待ち時間 (バックオフ) などで不規則なので、
%    指紋のように一致する場所が 1 か所しかない。これを使って、
%      1. 粗い位置合わせ   antenna1 の中ほど 5 秒分の並びを、antenna2 の中で
%                          ±maxOffsetSec の範囲で探す (相互相関, 50 us 単位)
%      2. パケットの対応づけ 同じ種類のパケット同士 (HE の下り同士、RTS 同士、
%                          Block Ack 同士 …) を 1 対 1 で対応づける
%      3. ずれの追跡      2 秒ごとにずれを求め直す。これでクロックのわずかな
%                          差 (数 ppm) や、オーバーランによる時刻の段差にも
%                          追従できる
%    の順に揃える。精度は数 us 程度 (パケット間隔は数 ms)。
%
%    すでに復号済みの *_CSI.mat だけを使うので、復号し直す必要はない。
%
%  時間軸:
%    antenna1 (refTag) の時計を基準にし、antenna2 (otherTag) の各パケットの
%    時刻をそれに変換する。出力の timeSec は、2 台そろって 0 から始まる。
%    同じ時刻のパケットは、2 台が同じ瞬間に受けたもの。
%
%  入力 (csiPath 内のペアを自動で探す):
%    S2_C_antenna1_WAX202_CSI.mat  と  S2_C_antenna2_WAX202_CSI.mat
%    (ファイル名の refTag を otherTag に置き換えた名前を相手とみなす)
%
%  出力 (受信機ごとに 1 つ):
%    S2_C_antenna1_WAX202_aligned_CSI.mat
%    S2_C_antenna2_WAX202_aligned_CSI.mat
%      中身の変数名は decodeIQ_HE.m の出力と同じ (csiHE / timeSecHE / ...)。
%      ResultCSI.m や SHARP-USRP の変換スクリプトにそのまま渡せる。
%      変わるのは次の点だけ。
%        - 両方がそろっている区間から outputDuration 秒だけを残す
%        - timeSec* は揃えた共通の時刻 (切り出し区間の先頭が 0)
%        - timeSecLocal* に元の (各受信機の) 時刻を残す
%        - csiMeta.alignment に、揃えたときの情報 (ずれ・品質など) を残す
%    ログ: align_<日時>.log と align_<日時>_summary.csv
%
%  注意:
%    - 2 台が共通のパケットをある程度受けている必要がある (目安 3 割以上)。
%      片方がほとんど受けていない場合は失敗として報告し、出力は作らない。
%    - 開始時刻のずれが maxOffsetSec (既定 20 秒) を超える場合は見つけられない。
%      そのときは maxOffsetSec を大きくする。
% =========================================================================

clear; clc;

%% ------------------------------------------------------------------------
%  1. ユーザ設定パラメータ
%  ------------------------------------------------------------------------
cfg = struct();

% --- CSI ファイルのあるフォルダ (decodeIQ_HE.m の hddSavePath) ------------
cfg.csiPath = 'D:\IQ_csi';

% --- ペアの見分け方 ------------------------------------------------------
%     ファイル名の refTag を otherTag に置き換えたものを相手とみなす。
%     時間軸は refTag 側の時計を基準にする。
cfg.refTag   = 'antenna1';
cfg.otherTag = 'antenna2';

% --- 切り出す長さ [s] ----------------------------------------------------
cfg.outputDuration = 80;

% --- 切り出す位置 --------------------------------------------------------
%     'center' … 2 台がそろっている区間の真ん中から切り出す (前後を均等に捨てる)
%     'start'  … 2 台がそろった瞬間から切り出す
cfg.windowPosition = 'center';

% --- 2 台の開始時刻のずれとして想定する最大値 [s] ------------------------
cfg.maxOffsetSec = 20;

% --- すでに出力があるペアを飛ばすか --------------------------------------
cfg.skipExisting = true;

%% ------------------------------------------------------------------------
%  2. 実行
%  ------------------------------------------------------------------------
runAlign(cfg);

%% ------------------------------------------------------------------------
%  ローカル関数
%  ------------------------------------------------------------------------
function runAlign(cfg)
    if ~exist(cfg.csiPath, 'dir')
        error('alignCSI:noFolder', 'フォルダがありません: %s', cfg.csiPath);
    end

    runTag  = datestr(now, 'yyyymmdd_HHMMSS');
    logFile = fullfile(cfg.csiPath, ['align_' runTag '.log']);
    csvFile = fullfile(cfg.csiPath, ['align_' runTag '_summary.csv']);
    diary(logFile);
    diaryCleanup = onCleanup(@() diary('off')); %#ok<NASGU>

    pairs = findPairs(cfg);
    if isempty(pairs)
        fprintf('ペアが見つかりませんでした (%s 内の *%s*_CSI.mat と *%s*_CSI.mat)。\n', ...
            cfg.csiPath, cfg.refTag, cfg.otherTag);
        return;
    end

    fprintf('時間軸を揃えるペア (基準: %s):\n', cfg.refTag);
    for k = 1:numel(pairs)
        fprintf('  %s  %s\n', ternary(pairs(k).skip, '済', '  '), pairs(k).label);
    end

    results = newResult(pairs(1));
    results(:) = [];
    for k = 1:numel(pairs)
        if pairs(k).skip
            res = newResult(pairs(k));
            res.status = 'SKIP';
        else
            try
                res = alignPair(pairs(k), cfg);
            catch ME
                res = newResult(pairs(k));
                res.status = 'FAIL';
                res.message = ME.message;
                fprintf(2, '\n  失敗しました: %s\n', ME.message);
            end
        end
        results(end+1) = res; %#ok<AGROW>
        writeAlignCsv(csvFile, results);
    end

    printAlignSummary(results);
    fprintf('\nログ       : %s\n', logFile);
    fprintf('結果の一覧 : %s\n', csvFile);
end

function pairs = findPairs(cfg)
    lst   = dir(fullfile(cfg.csiPath, '*_CSI.mat'));
    names = {lst.name};
    names = names(~contains(names, '_aligned_CSI.mat'));   % 出力は入力にしない
    names = sort(names);

    pairs = struct('refFile', {}, 'otherFile', {}, 'refOut', {}, ...
        'otherOut', {}, 'label', {}, 'skip', {});
    for k = 1:numel(names)
        if ~contains(names{k}, cfg.refTag)
            continue;
        end
        partner = strrep(names{k}, cfg.refTag, cfg.otherTag);
        if ~any(strcmp(names, partner))
            fprintf('  相手が見つからないため飛ばします: %s (探した名前: %s)\n', ...
                names{k}, partner);
            continue;
        end
        refOut   = fullfile(cfg.csiPath, regexprep(names{k}, '_CSI\.mat$', '_aligned_CSI.mat'));
        otherOut = fullfile(cfg.csiPath, regexprep(partner,  '_CSI\.mat$', '_aligned_CSI.mat'));
        pr = struct( ...
            'refFile',   fullfile(cfg.csiPath, names{k}), ...
            'otherFile', fullfile(cfg.csiPath, partner), ...
            'refOut',    refOut, ...
            'otherOut',  otherOut, ...
            'label',     sprintf('%s <-> %s', names{k}, partner), ...
            'skip',      cfg.skipExisting && isfile(refOut) && isfile(otherOut));
        pairs(end+1) = pr; %#ok<AGROW>
    end
end

function res = newResult(pr)
    res = struct('label', pr.label, 'status', '', 'message', '', ...
        'startDelaySec', NaN, 'confidence', NaN, 'nPairs', 0, 'matchRatio', NaN, ...
        'residualUs', NaN, 'driftPpm', NaN, 'nSteps', 0, 'overlapSec', NaN, ...
        'outputSec', NaN, 'nHERef', 0, 'nHEOther', 0);
end

function res = alignPair(pr, cfg)
    res = newResult(pr);
    fprintf('\n=========================================================================\n');
    fprintf(' %s\n', pr.label);
    fprintf('=========================================================================\n');

    A = load(pr.refFile);
    B = load(pr.otherFile);
    evA = collectEvents(A);
    evB = collectEvents(B);
    D1  = max([captureLength(A); evA.t]);
    D2  = max([captureLength(B); evB.t]);
    fprintf('  パケット数 (対象SSIDの全形式): %s %d 件 / %s %d 件\n', ...
        cfg.refTag, numel(evA.t), cfg.otherTag, numel(evB.t));
    if numel(evA.t) < 200 || numel(evB.t) < 200
        res = failResult(res, sprintf( ...
            'パケットが少なすぎて揃えられません (%d 件 / %d 件)。', numel(evA.t), numel(evB.t)));
        return;
    end

    % --- 1. 粗い位置合わせ ---
    [L0, conf] = coarseOffset(evA, evB, D1, cfg.maxOffsetSec);
    res.confidence = conf;
    fprintf('  粗い位置合わせ: ずれ %.4f s (確信度 %.1f)\n', L0, conf);
    if conf < 1.5
        res = failResult(res, sprintf( ...
            ['パケットの並びが一致する位置をはっきり見つけられませんでした (確信度 %.1f)。', ...
             '開始時刻のずれが %g 秒を超えている、または 2 台で共通に受けた', ...
             'パケットが少ない可能性があります。'], conf, cfg.maxOffsetSec));
        return;
    end

    % --- 2, 3. パケットの対応づけと、ずれの追跡 ---
    fm = fineMatch(evA, evB, L0);
    res.nPairs     = numel(fm.t1);
    res.residualUs = fm.residualUs;
    res.driftPpm   = fm.driftPpm;
    res.nSteps     = numel(fm.stepTimes);
    if numel(fm.t1) < 100
        res = failResult(res, sprintf( ...
            '対応づけられたパケットが %d 組しかなく、揃えられません。', numel(fm.t1)));
        return;
    end

    offP = fm.t2 - fm.t1;          % (other の時刻) - (ref の時刻)
    mapB = @(t2) t2 - offsetAt(t2, fm.t2, offP);

    startDelay = mapB(0);          % other の開始時刻を ref の時計で見たもの
    res.startDelaySec = startDelay;
    if startDelay >= 0
        fprintf('  %s は %s より %.3f 秒遅れて開始\n', cfg.otherTag, cfg.refTag, startDelay);
    else
        fprintf('  %s は %s より %.3f 秒早く開始\n', cfg.otherTag, cfg.refTag, -startDelay);
    end

    % --- 2 台がそろっている区間と、切り出し区間 ---
    a = max(0, mapB(0));
    b = min(D1, mapB(D2));
    overlap = b - a;
    res.overlapSec = overlap;
    inOverlap = evA.t >= a & evA.t < b;
    res.matchRatio = numel(fm.t1) / max(nnz(inOverlap), 1);

    fprintf('  対応づけたパケット: %d 組 (%s の %.0f%%)、残差 %.1f us\n', ...
        numel(fm.t1), cfg.refTag, 100 * res.matchRatio, fm.residualUs);
    fprintf('  ずれの変化: 開始時 %.3f ms -> 終了時 %.3f ms (クロック差 %.1f ppm)\n', ...
        1e3 * offP(1), 1e3 * offP(end), fm.driftPpm);
    if ~isempty(fm.stepTimes)
        for s = 1:numel(fm.stepTimes)
            fprintf('  時刻の段差: %.1f s 付近で %.2f ms (どちらかのオーバーランと思われる。追従済み)\n', ...
                fm.stepTimes(s), 1e3 * fm.stepSizes(s));
        end
    end
    if res.matchRatio < 0.3
        warning('alignCSI:lowMatch', ...
            ['対応づけられたパケットが %.0f%% と少なめです。揃え方の精度は保たれますが、\n', ...
             '2 台の受信状況がかなり違う可能性があります。'], 100 * res.matchRatio);
    end

    dur = cfg.outputDuration;
    if overlap < dur
        warning('alignCSI:shortOverlap', ...
            '2 台がそろっている区間が %.1f 秒しかないため、%.1f 秒だけ出力します。', overlap, overlap);
        dur = overlap;
        winStart = a;
    elseif strcmpi(cfg.windowPosition, 'start')
        winStart = a;
    else
        winStart = a + (overlap - dur) / 2;
    end
    res.outputSec = dur;
    fprintf('  2 台がそろっている区間: %s の時計で %.2f〜%.2f s (%.1f 秒)\n', ...
        cfg.refTag, a, b, overlap);
    fprintf('  切り出し区間          : %s の時計で %.2f〜%.2f s (%.1f 秒) -> 出力では 0〜%.1f s\n', ...
        cfg.refTag, winStart, winStart + dur, dur, dur);

    % --- 揃えたときの情報 (出力の csiMeta.alignment に残す) ---
    info = struct();
    info.method            = 'packet arrival matching (same AP traffic)';
    info.referenceFile     = pr.refFile;
    info.otherFile         = pr.otherFile;
    info.windowStartRefSec = winStart;     % 切り出し区間の先頭 (ref の元の時刻)
    info.outputDuration    = dur;
    info.otherStartDelaySec = startDelay;  % other の開始が ref より何秒遅いか
    info.coarseOffsetSec   = L0;
    info.confidence        = conf;
    info.nMatchedPackets   = numel(fm.t1);
    info.matchRatio        = res.matchRatio;
    info.residualUs        = fm.residualUs;
    info.clockDiffPpm      = fm.driftPpm;
    info.stepTimesRefSec   = fm.stepTimes;
    info.stepSizesSec      = fm.stepSizes;
    info.matchedTimesRef   = fm.t1;        % 対応づけたパケットの元の時刻 (検証用)
    info.matchedTimesOther = fm.t2;

    infoA = info; infoA.role = 'reference';
    infoB = info; infoB.role = 'aligned to reference';
    Aout = cropAligned(A, @(t) t, winStart, dur, infoA);
    Bout = cropAligned(B, mapB,    winStart, dur, infoB);
    res.nHERef   = numel(fieldOr(Aout, 'timeSecHE', []));
    res.nHEOther = numel(fieldOr(Bout, 'timeSecHE', []));

    save(pr.refOut,   '-struct', 'Aout', '-v7.3');
    save(pr.otherOut, '-struct', 'Bout', '-v7.3');
    fprintf('  保存しました (HE: %d 件 / %d 件):\n    %s\n    %s\n', ...
        res.nHERef, res.nHEOther, pr.refOut, pr.otherOut);
    res.status = 'OK';
end

function res = failResult(res, msg)
    % 想定内の失敗。途中まで求めた値 (確信度など) は結果の一覧に残す。
    res.status  = 'FAIL';
    res.message = msg;
    fprintf(2, '\n  失敗しました: %s\n', msg);
end

function ev = collectEvents(S)
    % 対象 SSID の全パケットの (時刻, 種類) を時刻順に並べる。
    % 種類を分けておくのは、対応づけで取り違えを防ぐため (RTS の直後に HE の
    % データが来るなど、別のパケットが 0.1 ms 程度の近さで並ぶことがある)。
    t = zeros(0, 1);
    kind = cell(0, 1);
    legacy = {'NonHT', 'Non-HT'; 'HT', 'HT'; 'VHT', 'VHT'};
    for f = 1:size(legacy, 1)
        tt = double(fieldOr(S, ['timeSec' legacy{f, 1}], []));
        tt = tt(:);
        if isempty(tt)
            continue;
        end
        ft = fieldOr(S, ['frameType' legacy{f, 1}], {});
        if iscell(ft) && numel(ft) == numel(tt)
            kk = cellfun(@(x) [legacy{f, 2} ':' char(x)], ft(:), 'UniformOutput', false);
        else
            kk = repmat(legacy(f, 2), numel(tt), 1);
        end
        t = [t; tt]; kind = [kind; kk]; %#ok<AGROW>
    end

    tt = double(fieldOr(S, 'timeSecHE', []));
    tt = tt(:);
    if ~isempty(tt)
        ul = fieldOr(S, 'uplinkHE', []);
        if numel(ul) == numel(tt)
            kk = repmat({'HE-DL'}, numel(tt), 1);
            kk(logical(ul(:))) = {'HE-UL'};
        else
            kk = repmat({'HE'}, numel(tt), 1);
        end
        t = [t; tt]; kind = [kind; kk];
    end

    [t, order] = sort(t);
    ev = struct('t', t, 'kind', {kind(order)});
end

function D = captureLength(S)
    % キャプチャの実際の長さ [s]
    D = NaN;
    try
        D = double(S.csiMeta.captureMeta.totalSamples) / double(S.csiMeta.sampleRate);
    catch
    end
    if ~isfinite(D) || D <= 0
        D = double(S.csiMeta.captureDuration);
    end
end

function [L0, conf] = coarseOffset(evA, evB, D1, M)
    % ref の中ほど 5 秒分のパケットの並びを、other の中で ±M 秒の範囲で探す。
    % 種類ごとに 50 us 単位の相互相関を取って足し合わせる。
    binW   = 50e-6;
    segLen = 5;
    s0     = max(0, D1 / 2 - segLen / 2);
    inSeg  = evA.t >= s0 & evA.t < s0 + segLen;
    if nnz(inSeg) < 100
        segLen = 15;
        s0     = max(0, D1 / 2 - segLen / 2);
        inSeg  = evA.t >= s0 & evA.t < s0 + segLen;
    end

    nR   = ceil(segLen / binW);
    nO   = ceil((segLen + 2 * M) / binW);
    nfft = 2^nextpow2(nR + nO);
    C    = zeros(nfft, 1);
    kinds = unique(evA.kind(inSeg));
    for q = 1:numel(kinds)
        r = evA.t(inSeg & strcmp(evA.kind, kinds{q})) - s0;
        o = evB.t(strcmp(evB.kind, kinds{q})) - (s0 - M);
        o = o(o >= 0 & o < segLen + 2 * M);
        if isempty(r) || isempty(o)
            continue;
        end
        R = accumarray(min(floor(r / binW) + 1, nR), 1, [nR 1]);
        O = accumarray(min(floor(o / binW) + 1, nO), 1, [nO 1]);
        C = C + real(ifft(conj(fft(R, nfft)) .* fft(O, nfft)));
    end

    K  = nO - nR + 1;              % 調べるずれの数
    Cs = movsum(C(1:K), 5);        % ±100 us をまとめて数える
    [pk, kIdx] = max(Cs);
    L0 = (kIdx - 1) * binW - M;

    % 確信度 = 一番の山 / それ以外 (±1 ms の外) で一番の山
    guard = 20;
    Cs2 = Cs;
    Cs2(max(1, kIdx - guard):min(K, kIdx + guard)) = 0;
    conf = pk / max(max(Cs2), 1);
end

function fm = fineMatch(evA, evB, L0)
    % 同じ種類のパケット同士を 1 対 1 で対応づけ、2 秒ごとのずれを追跡する。
    W      = 10e-3;     % 粗い位置合わせからのずれとして探す範囲
    winLen = 2;         % ずれを求め直す間隔 [s]
    tolAcc = 20e-6;     % その区間のずれからの許容幅

    [cRef, cOth, cD] = candidatePairs(evA, evB, L0, W);
    if isempty(cRef)
        fm = struct('t1', [], 't2', [], 'residualUs', NaN, 'driftPpm', NaN, ...
            'stepTimes', [], 'stepSizes', []);
        return;
    end

    cT1 = evA.t(cRef);
    wIdx = floor(cT1 / winLen) + 1;
    nW   = max(max(wIdx), floor(max(evA.t) / winLen) + 1);
    nRefWin = accumarray(floor(evA.t / winLen) + 1, 1, [nW 1]);

    edges = -W:10e-6:W;
    offW  = NaN(nW, 1);       % 区間ごとの主なずれ (一番大きい山)
    peaksW = cell(nW, 1);     % 区間ごとのずれの山 (段差を含む区間では 2 つ)
    for w = 1:nW
        sel = (wIdx == w);
        if nnz(sel) < 5
            continue;
        end
        cnt = movsum(histcounts(cD(sel), edges), 3);
        % 2 台がそろっていない区間では、たまたまの一致しかないので採用しない
        thr = max(5, 0.1 * nRefWin(w));
        pk = [];
        while true
            [cmax, m] = max(cnt);
            if cmax < thr
                break;
            end
            center = (edges(m) + edges(m + 1)) / 2;
            near = sel & abs(cD - center) < tolAcc;
            if nnz(near) >= thr
                pk(end+1) = median(cD(near)); %#ok<AGROW>
            end
            cnt(max(1, m - 4):min(numel(cnt), m + 4)) = 0;
        end
        peaksW{w} = pk;
        if ~isempty(pk)
            offW(w) = pk(1);
        end
    end

    % 各候補を、自分の区間の山と、隣の区間の主なずれに照らして採否を決める。
    % オーバーランで時刻に段差がある区間では、段差の手前は前の区間と、
    % 後ろは次の区間と同じずれになる。少数派の側も取りこぼさないため。
    resid = inf(size(cD));
    for w = 1:nW
        cand = peaksW{w};
        if w > 1 && ~isnan(offW(w - 1))
            cand(end+1) = offW(w - 1); %#ok<AGROW>
        end
        if w < nW && ~isnan(offW(w + 1))
            cand(end+1) = offW(w + 1); %#ok<AGROW>
        end
        if isempty(cand)
            continue;
        end
        sel = find(wIdx == w);
        for c = cand
            resid(sel) = min(resid(sel), abs(cD(sel) - c));
        end
    end
    idx = find(resid < tolAcc);
    % 1 つのパケットに複数の相手が残った場合は、ずれに一番近いものを採る
    [~, ord] = sort(resid(idx));
    idx = idx(ord);
    [~, first] = unique(cRef(idx), 'stable');  idx = idx(first);
    [~, first] = unique(cOth(idx), 'stable');  idx = idx(first);

    t1 = evA.t(cRef(idx));
    t2 = evB.t(cOth(idx));
    [t2, ord] = sort(t2);
    t1  = t1(ord);
    idx = idx(ord);
    keep = [true; diff(t2) > 0 & diff(t1) > 0];
    t1 = t1(keep); t2 = t2(keep); idx = idx(keep);

    % 前後 7 組の中央値から外れた対応を除く。孤立した取り違えだけが消え、
    % オーバーランによる本物の段差は (多数派が切り替わるだけなので) 残る。
    off  = t2 - t1;
    keep = abs(off - movmedian(off, 15)) < tolAcc;
    t1 = t1(keep); t2 = t2(keep); idx = idx(keep);

    % --- 品質の指標 ---
    fm = struct();
    fm.t1 = t1;
    fm.t2 = t2;
    fm.residualUs = 1e6 * sqrt(mean(resid(idx).^2));

    valid = find(~isnan(offW));
    tw = (valid - 0.5) * winLen;
    ow = offW(valid);
    dO = diff(ow);
    isStep = abs(dO) > 200e-6;
    fm.stepTimes = tw([false; isStep]) - winLen / 2;
    fm.stepSizes = dO(isStep);
    dT = diff(tw(:));
    slopes = dO(~isStep) ./ dT(~isStep);     % 段差を除いた区間ごとの傾き
    fm.driftPpm = 1e6 * median(slopes, 'omitnan');
end

function [cRef, cOth, cD] = candidatePairs(evA, evB, L0, W)
    % ref の各パケットについて、同じ種類の other のパケットのうち
    % 粗いずれ L0 から ±W に入るものを候補として全部挙げる。
    kinds = unique(evA.kind);
    parts = cell(numel(kinds), 3);
    for q = 1:numel(kinds)
        ia = find(strcmp(evA.kind, kinds{q}));
        ib = find(strcmp(evB.kind, kinds{q}));
        if isempty(ia) || isempty(ib)
            continue;
        end
        r = evA.t(ia);
        o = evB.t(ib);
        nA = numel(r);
        nB = numel(o);

        jLo = zeros(nA, 1);
        jHi = zeros(nA, 1);
        j = 1;
        for a = 1:nA
            lo = r(a) + L0 - W;
            while j <= nB && o(j) < lo
                j = j + 1;
            end
            jLo(a) = j;
        end
        j = 1;
        for a = 1:nA
            hi = r(a) + L0 + W;
            while j <= nB && o(j) <= hi
                j = j + 1;
            end
            jHi(a) = j - 1;
        end

        cnt = max(jHi - jLo + 1, 0);
        pr = zeros(sum(cnt), 1);
        po = zeros(sum(cnt), 1);
        p = 0;
        for a = find(cnt > 0).'
            n = cnt(a);
            pr(p + (1:n)) = ia(a);
            po(p + (1:n)) = ib(jLo(a):jHi(a));
            p = p + n;
        end
        parts(q, :) = {pr, po, evB.t(po) - evA.t(pr) - L0};
    end
    cRef = vertcat(parts{:, 1});
    cOth = vertcat(parts{:, 2});
    cD   = vertcat(parts{:, 3});
end

function off = offsetAt(t2, t2p, offP)
    % other の時刻 t2 における (other - ref) のずれ。対応づけたパケットの
    % 間は線形に補間し、範囲の外は端の値をそのまま使う。
    off = interp1(t2p, offP, t2, 'linear');
    off(t2 < t2p(1))   = offP(1);
    off(t2 > t2p(end)) = offP(end);
end

function S = cropAligned(S, mapFun, winStart, dur, info)
    % 1 台分の CSI を共通の時刻に直し、切り出し区間だけを残す。
    fmts = {'NonHT', 'HT', 'VHT', 'HE'};
    for f = 1:numel(fmts)
        fn = fmts{f};
        tf = ['timeSec' fn];
        if ~isfield(S, tf) || isempty(S.(tf))
            S.(['timeSecLocal' fn]) = zeros(0, 1);
            continue;
        end
        tLocal = double(S.(tf)(:));
        n      = numel(tLocal);
        tCommon = mapFun(tLocal) - winStart;
        keep    = tCommon >= 0 & tCommon < dur;

        S.(['timeSecLocal' fn]) = tLocal(keep);
        S.(tf) = tCommon(keep);

        perPacket = {['csi' fn], ['frameType' fn], ['fcs' fn]};
        if strcmp(fn, 'HE')
            perPacket = [perPacket, {'uplinkHE', 'bssColorHE'}]; %#ok<AGROW>
        end
        for v = perPacket
            name = v{1};
            if ~isfield(S, name) || isempty(S.(name))
                continue;
            end
            if size(S.(name), 1) == n
                S.(name) = S.(name)(keep, :);
            else
                warning('alignCSI:rowMismatch', ...
                    '%s の行数 (%d) が %s (%d) と合わないため空にします。', ...
                    name, size(S.(name), 1), tf, n);
                S.(name) = S.(name)([], :);
            end
        end
    end

    % ResultCSI.m 互換の「主」データを、主フォーマットで作り直す
    pf  = S.csiMeta.primaryFormat;
    key = strrep(pf, '-', '');
    if isfield(S, ['csi' key])
        S.csi         = S.(['csi' key]);
        S.timeSec     = S.(['timeSec' key]);
        S.frameType   = fieldOr(S, ['frameType' key], {});
        S.fcsVerified = fieldOr(S, ['fcs' key], []);
        S.phyFormat   = repmat({pf}, numel(S.timeSec), 1);
        S.packetStartIndex = round(S.timeSec(:) * S.csiMeta.sampleRate);
    end

    S.csiMeta.captureDuration = dur;
    S.csiMeta.alignment       = info;
end

function v = fieldOr(S, name, default)
    if isfield(S, name)
        v = S.(name);
    else
        v = default;
    end
end

function printAlignSummary(results)
    fprintf('\n=========================================================================\n');
    fprintf(' 時間軸の揃え方の結果\n');
    fprintf('=========================================================================\n');
    for k = 1:numel(results)
        r = results(k);
        switch r.status
            case 'OK'
                fprintf('  OK    %s\n', r.label);
                fprintf('          開始のずれ %+.3f s, 対応 %d 組 (%.0f%%), 残差 %.1f us, 段差 %d, 出力 %.1f s (HE %d / %d 件)\n', ...
                    r.startDelaySec, r.nPairs, 100 * r.matchRatio, r.residualUs, ...
                    r.nSteps, r.outputSec, r.nHERef, r.nHEOther);
            case 'SKIP'
                fprintf('  SKIP  %s (出力済み)\n', r.label);
            otherwise
                fprintf('  FAIL  %s\n          %s\n', r.label, r.message);
        end
    end
end

function writeAlignCsv(csvFile, results)
    fid = fopen(csvFile, 'w', 'n', 'UTF-8');
    if fid == -1
        return;
    end
    fprintf(fid, '%s', char(65279));   % Excel 向けの BOM
    fprintf(fid, ['pair,status,start_delay_s,confidence,matched_pairs,match_ratio,', ...
                  'residual_us,clock_diff_ppm,steps,overlap_s,output_s,HE_ref,HE_other,message\n']);
    for k = 1:numel(results)
        r = results(k);
        fprintf(fid, '"%s",%s,%.4f,%.2f,%d,%.3f,%.2f,%.2f,%d,%.2f,%.2f,%d,%d,"%s"\n', ...
            r.label, r.status, r.startDelaySec, r.confidence, r.nPairs, r.matchRatio, ...
            r.residualUs, r.driftPpm, r.nSteps, r.overlapSec, r.outputSec, ...
            r.nHERef, r.nHEOther, strrep(regexprep(r.message, '\s+', ' '), '"', '""'));
    end
    fclose(fid);
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end

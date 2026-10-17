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
%    2 台は同じ AP・同じクライアントの同じパケットを受信している。同じパケット
%    なら、2 台での到着時刻の差 (= 2 台の時計のずれ) は 1 us 以下の精度で
%    そろう。一方、iperf3 で飽和した通信は数 ms 周期でほぼ同じ形が繰り返される
%    ので、粗い時間分解能で比べると「1 周期ずれた位置」とも区別がつかない。
%    違うパケット同士はバックオフ (9 us 単位のランダムな待ち時間) の分だけ
%    ずれるので、10 us の細かさで比べれば本当の位置だけが際立つ。これを使って、
%      1. 粗い位置合わせ   antenna1 の 5 か所 (各 1 秒) のパケットについて、
%                          antenna2 の同じ種類のパケットとの時刻差を
%                          ±maxOffsetSec の範囲で全部求め、10 us の幅に
%                          一番多く集まる時刻差を採る
%      2. ずれの追跡      1 秒ごとに、前の区間のずれの近く (±20 ms、見つから
%                          なければ ±2 s) で同じように求め直す。これでクロック
%                          のわずかな差 (数 ppm) や、オーバーランによる時刻の
%                          段差にも追従できる
%      3. パケットの対応づけ 同じ種類のパケット同士 (HE の下り同士、RTS 同士、
%                          Block Ack 同士 …) を、その区間のずれ ±20 us で
%                          1 対 1 で対応づけ、その間を直線で補間する
%    の順に揃える。精度は 1 us 程度 (パケット間隔は数 ms)。
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
                if ~isempty(ME.stack)
                    % 想定外のエラーは、どこで起きたかも残す
                    res.message = sprintf('%s (%s の %d 行目)', ME.message, ...
                        ME.stack(1).name, ME.stack(1).line);
                end
                fprintf(2, '\n  失敗しました: %s\n', res.message);
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
    [evA, evB] = indexKinds(evA, evB);
    D1  = max([captureLength(A); evA.t]);
    D2  = max([captureLength(B); evB.t]);
    fprintf('  パケット数 (対象SSIDの全形式): %s %d 件 / %s %d 件\n', ...
        cfg.refTag, numel(evA.t), cfg.otherTag, numel(evB.t));
    fprintf('  キャプチャ時のオーバーラン  : %s %g 回 / %s %g 回\n', ...
        cfg.refTag, fieldOr(A.csiMeta, 'overrunCount', NaN), ...
        cfg.otherTag, fieldOr(B.csiMeta, 'overrunCount', NaN));
    if numel(evA.t) < 200 || numel(evB.t) < 200
        res = failResult(res, sprintf( ...
            'パケットが少なすぎて揃えられません (%d 件 / %d 件)。', numel(evA.t), numel(evB.t)));
        return;
    end

    % --- 1. 粗い位置合わせ ---
    co = coarseOffset(evA, evB, D1, cfg.maxOffsetSec);
    if isempty(co.conf)
        res = failResult(res, '位置合わせに使える区間がありません (パケットが少なすぎます)。');
        return;
    end
    fprintf('  粗い位置合わせ (%s の 1 秒区間ごとに、±%g 秒の範囲を 10 us の細かさで探索):\n', ...
        cfg.refTag, cfg.maxOffsetSec);
    for s = 1:numel(co.conf)
        fprintf('    %5.1f s 付近: ずれ %+.6f s, 一致 %4d / %4d 件, 確信度 %5.1f\n', ...
            co.segCenter(s), co.offset(s), co.count(s), co.nRef(s), co.conf(s));
    end
    [conf, best] = max(co.conf);
    L0 = co.offset(best);
    res.confidence = conf;
    if conf < 3
        res = failResult(res, sprintf( ...
            ['パケットの並びが一致する位置をはっきり見つけられませんでした (確信度 %.1f)。', ...
             '開始時刻のずれが %g 秒を超えている、または 2 台で共通に受けた', ...
             'パケットが少ない可能性があります。'], conf, cfg.maxOffsetSec));
        return;
    end

    % --- 2, 3. ずれの追跡と、パケットの対応づけ ---
    fm = fineMatch(evA, evB, D1, co.segCenter(best), L0);
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
    fprintf('  ずれ (%s - %s): 最初 %+.6f s -> 最後 %+.6f s (クロック差 %.1f ppm)\n', ...
        cfg.otherTag, cfg.refTag, offP(1), offP(end), fm.driftPpm);
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

function [evA, evB] = indexKinds(evA, evB)
    % 種類を番号にし、other は種類ごとのパケット番号の一覧を作っておく
    % (対応づけで何度も使うので、文字列の比較を毎回しないため)。
    [~, ~, id] = unique([evA.kind; evB.kind]);
    nA = numel(evA.t);
    evA.kid = id(1:nA);
    evB.kid = id(nA+1:end);
    nK = max([id; 0]);
    evB.byKind = cell(nK, 1);
    for q = 1:nK
        evB.byKind{q} = find(evB.kid == q);
    end
end

function co = coarseOffset(evA, evB, D1, M)
    % ref の 5 か所 (各 1 秒) について、other とのずれを ±M 秒の範囲で探す。
    % 同じ種類のパケット同士の時刻差を全部求め、幅 10 us に一番多く集まる
    % 時刻差を、その区間のずれとする。
    % 確信度 = その山の件数 / ±1 ms より外で 2 番目に大きい山の件数
    segLen = 1;
    fracs  = [0.3 0.4 0.5 0.6 0.7];
    bw     = 10e-6;
    co = struct('segCenter', [], 'offset', [], 'count', [], 'nRef', [], 'conf', []);
    for f = fracs
        s0  = max(0, D1 * f - segLen / 2);
        sel = find(evA.t >= s0 & evA.t < s0 + segLen);
        if numel(sel) < 50
            continue;
        end
        d = pairDiffs(evA, sel, evB, -M, M);
        if isempty(d)
            continue;
        end
        [loc, cnt] = diffPeaks(d, bw, 1e-3, 2);
        co.segCenter(end+1) = s0 + segLen / 2;
        co.offset(end+1)    = loc(1);
        co.count(end+1)     = cnt(1);
        co.nRef(end+1)      = numel(sel);
        co.conf(end+1)      = cnt(1) / max(cnt(2), 1);
    end
end

function fm = fineMatch(evA, evB, D1, anchorT, L0)
    % 1 秒ごとにずれを追跡し、同じ種類のパケット同士を 1 対 1 で対応づける。
    winLen = 1;         % ずれを求め直す間隔 [s]
    tolAcc = 20e-6;     % その区間のずれからの許容幅

    nW   = max(1, ceil(max([D1; evA.t]) / winLen));
    wOf  = min(floor(evA.t / winLen) + 1, nW);   % ref の各パケットの区間番号
    offW = trackOffset(evA, evB, wOf, nW, winLen, anchorT, L0);
    [t1, t2, resid] = matchPackets(evA, evB, offW, wOf, tolAcc);

    % --- 品質の指標 ---
    fm = struct();
    fm.t1 = t1;
    fm.t2 = t2;
    fm.residualUs = 1e6 * sqrt(mean(resid .^ 2));

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

function offW = trackOffset(evA, evB, wOf, nW, winLen, anchorT, L0)
    % 粗い位置合わせをした区間から前後へ 1 区間ずつ進み、前の区間のずれの
    % 近くで、その区間のずれを求める。2 台がそろっていない区間は NaN。
    offW = NaN(nW, 1);
    a = min(max(floor(anchorT / winLen) + 1, 1), nW);
    orders = {a:nW, a-1:-1:1};
    for k = 1:2
        p = L0;
        for w = orders{k}
            offW(w) = windowOffset(evA, find(wOf == w), evB, p);
            if ~isnan(offW(w))
                p = offW(w);
            end
        end
    end
end

function off = windowOffset(evA, sel, evB, p)
    % 1 区間分のずれ。まず前の区間のずれ p の ±20 ms で探し、見つからなければ
    % (大きなオーバーランの段差など) ±2 s に広げる。
    off = NaN;
    if numel(sel) < 5
        return;
    end
    thr = max(5, 0.1 * numel(sel));   % 2 台がそろっていない区間はここで落ちる
    for W = [20e-3, 2]
        d = pairDiffs(evA, sel, evB, p - W, p + W);
        if isempty(d)
            continue;
        end
        [loc, cnt] = diffPeaks(d, 10e-6, 1e-3, 2);
        if cnt(1) >= thr && cnt(1) >= 2 * cnt(2)
            off = loc(1);
            return;
        end
    end
end

function [t1, t2, resid] = matchPackets(evA, evB, offW, wOf, tol)
    % 各区間のパケットを、その区間と前後の区間のずれ ±tol で対応づける。
    % オーバーランで段差がある区間では、段差の手前は前の区間と、後ろは
    % 次の区間と同じずれになるので、前後の区間のずれも候補に入れる。
    nW = numel(offW);
    parts = cell(nW, 3);
    for w = 1:nW
        nb   = [w, w - 1, w + 1];
        nb   = nb(nb >= 1 & nb <= nW);
        cand = offW(nb);
        cand = cand(~isnan(cand));
        sel  = find(wOf == w);
        if isempty(cand) || isempty(sel)
            continue;
        end
        IA = []; IB = []; R = [];
        for c = cand(:).'
            [d, ia, ib] = pairDiffs(evA, sel, evB, c - tol, c + tol);
            IA = [IA; ia]; IB = [IB; ib]; R = [R; abs(d - c)]; %#ok<AGROW>
        end
        parts(w, :) = {IA, IB, R};
    end
    ia = vertcat(parts{:, 1});
    ib = vertcat(parts{:, 2});
    r  = vertcat(parts{:, 3});
    if isempty(ia)
        t1 = zeros(0, 1); t2 = zeros(0, 1); resid = zeros(0, 1);
        return;
    end

    % 1 つのパケットに複数の相手が残った場合は、ずれに一番近いものを採る
    [r, ord] = sort(r);
    ia = ia(ord); ib = ib(ord);
    [~, first] = unique(ia, 'stable');  ia = ia(first); ib = ib(first); r = r(first);
    [~, first] = unique(ib, 'stable');  ia = ia(first); ib = ib(first); r = r(first);

    t1 = evA.t(ia);
    t2 = evB.t(ib);
    [t2, ord] = sort(t2);
    t1 = t1(ord);
    r  = r(ord);
    keep = [true; diff(t2) > 0 & diff(t1) > 0];
    t1 = t1(keep); t2 = t2(keep); r = r(keep);

    % 前後 7 組の中央値から外れた対応を除く。孤立した取り違えだけが消え、
    % オーバーランによる本物の段差は (多数派が切り替わるだけなので) 残る。
    off  = t2 - t1;
    keep = abs(off - movmedian(off, 15)) < tol;
    t1 = t1(keep); t2 = t2(keep); resid = r(keep);
end

function [d, ia, ib] = pairDiffs(evA, selA, evB, lo, hi)
    % ref のパケット selA のそれぞれについて、同じ種類の other のパケットの
    % うち (other の時刻) - (ref の時刻) が [lo, hi] に入るものを全部挙げる。
    % d = その時刻差、ia / ib = ref / other のパケット番号
    d = zeros(0, 1); ia = zeros(0, 1); ib = zeros(0, 1);
    selA = selA(:);
    if isempty(selA)
        return;
    end
    wantIdx = (nargout > 1);
    kA = evA.kid(selA);
    parts = cell(0, 3);
    for q = unique(kA).'
        a = selA(kA == q);
        b = evB.byKind{q};
        if isempty(b)
            continue;
        end
        r = evA.t(a);
        % other は関係する範囲 [r(1)+lo, r(end)+hi] だけに先に絞る
        o  = evB.t(b);
        i0 = bisect(o, r(1) + lo, false);
        i1 = bisect(o, r(end) + hi, true) - 1;
        if i1 < i0
            continue;
        end
        b = b(i0:i1);
        o = o(i0:i1);
        jLo = firstIndex(o, r + lo, false);       % o >= r+lo となる最初
        jHi = firstIndex(o, r + hi, true) - 1;    % o <= r+hi となる最後
        n   = max(jHi - jLo + 1, 0);
        N   = sum(n);
        if N == 0
            continue;
        end
        % repelem は対象が 1 個 (スカラー) だと行ベクトルを返すので、
        % 必ず列ベクトルにそろえる (その種類が区間に 1 回だけのとき)
        rep  = reshape(repelem((1:numel(a)).', n), [], 1);
        base = reshape(repelem(cumsum(n) - n, n), [], 1);
        jj   = jLo(rep) + (1:N).' - 1 - base;
        if wantIdx
            parts(end+1, :) = {o(jj) - r(rep), a(rep), b(jj)}; %#ok<AGROW>
        else
            parts(end+1, :) = {o(jj) - r(rep), [], []}; %#ok<AGROW>
        end
    end
    if ~isempty(parts)
        d  = vertcat(parts{:, 1});
        ia = vertcat(parts{:, 2});
        ib = vertcat(parts{:, 3});
    end
end

function k = bisect(o, x, orEqual)
    % 昇順の o で、o(k) >= x (orEqual なら o(k) > x) となる最初の k
    % (なければ numel(o)+1)。x はスカラー。
    lo = 1; hi = numel(o) + 1;
    while lo < hi
        mid = floor((lo + hi) / 2);
        if o(mid) < x || (orEqual && o(mid) == x)
            lo = mid + 1;
        else
            hi = mid;
        end
    end
    k = lo;
end

function j = firstIndex(o, x, orEqual)
    % o と x は昇順。各 x(i) について、o(j) >= x(i) (orEqual なら o(j) > x(i))
    % となる最初の j (なければ numel(o)+1)。
    % o と x をまとめて並べ替え、各 x(i) より前に来た o の数を数える。
    % sort は同じ値の順序を保つので、同じ値のときに o と x のどちらを先に
    % 置くかで「>=」と「>」を切り替えられる。
    n = numel(o);
    m = numel(x);
    if orEqual
        v   = [o(:); x(:)];                 % 同じ値なら o が前 -> o <= x を数える
        isX = [false(n, 1); true(m, 1)];
    else
        v   = [x(:); o(:)];                 % 同じ値なら x が前 -> o <  x を数える
        isX = [true(m, 1); false(n, 1)];
    end
    [~, ord] = sort(v);
    posX = find(isX(ord));                  % x は昇順なので元の順に並ぶ
    j = reshape(posX - (1:m).' + 1, size(x));
end

function [loc, cnt] = diffPeaks(d, bw, guard, nMax)
    % 時刻差 d の中で、幅 bw に一番多く集まる位置を大きい順に nMax 個。
    % 2 つ目以降は、それまでの山から ±guard 以上離れたところから探す。
    loc = NaN(nMax, 1);
    cnt = zeros(nMax, 1);
    if isempty(d)
        return;
    end
    d = sort(d(:));
    c = firstIndex(d, d + bw, true) - (1:numel(d)).';   % [d(i), d(i)+bw] の件数
    for p = 1:nMax
        [cp, i1] = max(c);
        if cp <= 0
            break;
        end
        cnt(p) = cp;
        loc(p) = median(d(i1 : i1 + cp - 1));
        c(abs(d - loc(p)) < guard) = 0;
    end
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

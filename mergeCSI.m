%% mergeCSI.m
% =========================================================================
%  セグメント分割された CSI を1本の連続した時系列へ結合する
% -------------------------------------------------------------------------
%  用途:
%    captureIQ_single.m は長時間キャプチャを *_segNN_raw.mat に分割して
%    出力する。それらを decodeIQ_*.m で個別に復号すると、CSI も
%    *_segNN_..._CSI.mat と分かれてしまう。本スクリプトはそれらを読み込み、
%    キャプチャ開始を原点とする1本の時系列に繋ぎ直す。
%
%  受信自体は途切れていない:
%    captureIQ_single.m は 120 秒間を1本の .bin に連続して書き込んでおり、
%    分割は受信後にその .bin を切り出しているだけ。サンプルは別セグメントに
%    分かれても電波上は連続している。
%
%  本スクリプトが行う2つの処理:
%    [1] 時刻のオフセット加算
%        各セグメントの timeSec はそのセグメントの先頭を 0 とするため、
%        meta.segmentStartTimeSec を足してキャプチャ開始基準に直す。
%
%    [2] 重なり部分の重複除去
%        captureIQ_single.m は境界にまたがるパケットを取りこぼさないよう、
%        各セグメントの末尾に segmentOverlap 秒の重なりを付けている。
%        重なりで始まるパケットは次のセグメントにも現れるので、
%        「担当区間 (segmentCoreDuration) 内で始まったパケット」だけを
%        採用する。これで重複も欠落も無く繋がる。
%
%        旧形式 (重なり無しで分割されたもの) の場合はこの情報が無いので、
%        時刻が単調増加になるように前から順に採用する。
%
%  入力:
%    <csiSearchPath>/<共通プレフィクス>_seg*_*_CSI.mat
%
%  出力:
%    <csiSearchPath>/<共通プレフィクス>_merged_CSI.mat
%      変数構成は decodeIQ_*.m の出力と同じなので、ResultCSI.m がそのまま
%      読める (csiNonHT / csiHT / csiVHT / csiHE と各 timeSec など)。
%      mergeMeta に結合の内訳を記録する。
%
%  必要環境:
%    - MATLAB (基本機能のみ)
% =========================================================================

clear; clc;

%% ------------------------------------------------------------------------
%  1. ユーザ設定
%  ------------------------------------------------------------------------
% CSI ファイルを探すフォルダ (decodeIQ_*.m の hddSavePath と揃える)
csiSearchPath = 'D:\IQ_csi';

% 結合対象を選ぶファイル名パターン。
%   '' にすると csiSearchPath 内で最も新しい *_seg*_*_CSI.mat を見つけ、
%   その共通プレフィクス (例 '09171430') を持つものを全て集める。
%   特定の回を指定したい場合はプレフィクスを書く (例 '09171430')。
filePrefix = '';

%% ------------------------------------------------------------------------
%  2. 結合対象のファイルを集める
%  ------------------------------------------------------------------------
if isempty(filePrefix)
    d = dir(fullfile(csiSearchPath, '*_seg*_CSI.mat'));
    if isempty(d)
        error('mergeCSI:noInput', ...
            ['セグメントの CSI ファイルが見つかりません: %s\\*_seg*_CSI.mat\n', ...
             '先に captureIQ_single.m と decodeIQ_*.m を実行してください。'], ...
            csiSearchPath);
    end
    [~, iLatest] = max([d.datenum]);
    % 'プレフィクス_segNN_...' の先頭部分を取り出す
    tok = regexp(d(iLatest).name, '^(.+?)_seg\d+', 'tokens', 'once');
    if isempty(tok)
        error('mergeCSI:cannotInferPrefix', ...
            'ファイル名からプレフィクスを判定できませんでした: %s', d(iLatest).name);
    end
    filePrefix = tok{1};
    fprintf('最新のセグメントから対象を判定しました: プレフィクス "%s"\n', filePrefix);
end

listing = dir(fullfile(csiSearchPath, [filePrefix '_seg*_CSI.mat']));
if isempty(listing)
    error('mergeCSI:noMatchingFiles', ...
        '対象のファイルがありません: %s', ...
        fullfile(csiSearchPath, [filePrefix '_seg*_CSI.mat']));
end

% セグメント番号順に並べる (ファイル名の辞書順ではなく番号で並べる)
segNums = zeros(numel(listing), 1);
for k = 1:numel(listing)
    tok = regexp(listing(k).name, '_seg(\d+)', 'tokens', 'once');
    if isempty(tok)
        segNums(k) = NaN;
    else
        segNums(k) = str2double(tok{1});
    end
end
[segNums, ord] = sort(segNums);
listing = listing(ord);

fprintf('\n結合対象: %d ファイル\n', numel(listing));

%% ------------------------------------------------------------------------
%  3. 各セグメントを読み込み、時刻を補正して連結する
%  ------------------------------------------------------------------------
% フォーマットごとに CSI・時刻・フレーム種別・FCS検証結果を集める
formats = {'NonHT', 'HT', 'VHT', 'HE'};
acc = struct();
for f = 1:numel(formats)
    acc.(formats{f}) = struct('csi', {{}}, 'timeSec', {{}}, ...
        'frameType', {{}}, 'fcs', {{}}, 'subc', []);
end

mergeLog = struct('file', {}, 'segment', {}, 'startTimeSec', {}, ...
    'coreDuration', {}, 'kept', {}, 'dropped', {});
prevEndTime = -inf;   % 旧形式 (重なり情報なし) 用の単調性チェック

for k = 1:numel(listing)
    fpath = fullfile(listing(k).folder, listing(k).name);
    S = load(fpath);

    % --- このセグメントの開始時刻と担当区間を得る ---
    startTimeSec = 0;
    coreDuration = inf;   % 情報が無ければ全採用
    hasCoreInfo  = false;
    if isfield(S, 'csiMeta') && isfield(S.csiMeta, 'captureMeta')
        cm = S.csiMeta.captureMeta;
        if isfield(cm, 'segmentStartTimeSec')
            startTimeSec = cm.segmentStartTimeSec;
        end
        if isfield(cm, 'segmentCoreDuration')
            coreDuration = cm.segmentCoreDuration;
            hasCoreInfo  = true;
        end
    end

    nKept = 0; nDropped = 0;

    for f = 1:numel(formats)
        fmt     = formats{f};
        csiVar  = ['csi' fmt];
        timeVar = ['timeSec' fmt];
        frmVar  = ['frameType' fmt];
        fcsVar  = ['fcs' fmt];
        switch fmt
            case 'NonHT', subcVar = 'subcarrierIndicesNonHT';
            case 'HT',    subcVar = 'subcarrierIndicesHT20';
            case 'VHT',   subcVar = 'subcarrierIndicesVHT20';
            case 'HE',    subcVar = 'subcarrierIndicesHE20';
        end

        if ~isfield(S, csiVar) || isempty(S.(csiVar))
            continue;
        end
        csiSeg = S.(csiVar);
        nPkt   = size(csiSeg, 1);

        if isfield(S, timeVar) && numel(S.(timeVar)) == nPkt
            tSeg = S.(timeVar)(:);
        else
            % 時刻が無ければ結合できない (重複除去も順序付けも不可能)
            warning('mergeCSI:noTimeSec', ...
                '%s に %s がありません。このフォーマットは読み飛ばします。', ...
                listing(k).name, timeVar);
            continue;
        end

        % --- 担当区間で始まったパケットだけを採用 ---
        if hasCoreInfo
            keep = (tSeg < coreDuration);
        else
            % 旧形式: 重なり情報が無いので単調増加で判定する
            keep = true(nPkt, 1);
        end

        nDropped = nDropped + sum(~keep);

        % 時刻をキャプチャ開始基準へ直す
        tAbs = tSeg(keep) + startTimeSec;

        if ~hasCoreInfo
            % 前のセグメントの終端より前のものは重複とみなして落とす
            mono = (tAbs > prevEndTime);
            nDropped = nDropped + sum(~mono);
            tAbs = tAbs(mono);
            keepIdx = find(keep);
            keepIdx = keepIdx(mono);
        else
            keepIdx = find(keep);
        end

        if isempty(keepIdx)
            continue;
        end

        acc.(fmt).csi{end+1}     = csiSeg(keepIdx, :);
        acc.(fmt).timeSec{end+1} = tAbs;

        if isfield(S, frmVar) && numel(S.(frmVar)) == nPkt
            ft = S.(frmVar);
            acc.(fmt).frameType{end+1} = ft(keepIdx);
        else
            acc.(fmt).frameType{end+1} = repmat({''}, numel(keepIdx), 1);
        end

        if isfield(S, fcsVar) && numel(S.(fcsVar)) == nPkt
            fc = S.(fcsVar);
            acc.(fmt).fcs{end+1} = logical(fc(keepIdx));
        else
            acc.(fmt).fcs{end+1} = false(numel(keepIdx), 1);
        end

        if isempty(acc.(fmt).subc) && isfield(S, subcVar)
            acc.(fmt).subc = S.(subcVar);
        end

        nKept = nKept + numel(keepIdx);
        prevEndTime = max(prevEndTime, max(tAbs));
    end

    mergeLog(end+1) = struct('file', listing(k).name, ...
        'segment', segNums(k), 'startTimeSec', startTimeSec, ...
        'coreDuration', coreDuration, 'kept', nKept, ...
        'dropped', nDropped); %#ok<SAGROW>

    fprintf('  seg%02d: 開始 %7.2f s  採用 %5d 件  除外 %4d 件  (%s)\n', ...
        segNums(k), startTimeSec, nKept, nDropped, listing(k).name);

    clear S;
end

%% ------------------------------------------------------------------------
%  4. フォーマットごとに1本の行列へまとめる
%  ------------------------------------------------------------------------
csiNonHT = cat(1, acc.NonHT.csi{:});
csiHT    = cat(1, acc.HT.csi{:});
csiVHT   = cat(1, acc.VHT.csi{:});
csiHE    = cat(1, acc.HE.csi{:});

timeSecNonHT = cat(1, acc.NonHT.timeSec{:});
timeSecHT    = cat(1, acc.HT.timeSec{:});
timeSecVHT   = cat(1, acc.VHT.timeSec{:});
timeSecHE    = cat(1, acc.HE.timeSec{:});

frameTypeNonHT = cat(1, acc.NonHT.frameType{:});
frameTypeHT    = cat(1, acc.HT.frameType{:});
frameTypeVHT   = cat(1, acc.VHT.frameType{:});
frameTypeHE    = cat(1, acc.HE.frameType{:});

fcsNonHT = cat(1, acc.NonHT.fcs{:});
fcsHT    = cat(1, acc.HT.fcs{:});
fcsVHT   = cat(1, acc.VHT.fcs{:});
fcsHE    = cat(1, acc.HE.fcs{:});

subcarrierIndicesNonHT = acc.NonHT.subc;
subcarrierIndicesHT20  = acc.HT.subc;
subcarrierIndicesVHT20 = acc.VHT.subc;
subcarrierIndicesHE20  = acc.HE.subc;

fprintf('\n結合結果:\n');
counts = [size(csiNonHT,1), size(csiHT,1), size(csiVHT,1), size(csiHE,1)];
names  = {'Non-HT', 'HT', 'VHT', 'HE'};
times  = {timeSecNonHT, timeSecHT, timeSecVHT, timeSecHE};
for f = 1:4
    if counts(f) > 0
        fprintf('  %-6s: %6d パケット  (%.2f 〜 %.2f s)\n', ...
            names{f}, counts(f), min(times{f}), max(times{f}));
    end
end
if all(counts == 0)
    error('mergeCSI:empty', ...
        'パケットが1件も集まりませんでした。入力ファイルを確認してください。');
end

%% ------------------------------------------------------------------------
%  5. ResultCSI.m 互換の「主」データを決めて保存
%  ------------------------------------------------------------------------
[~, bestIdx] = max(counts);
csiSets   = {csiNonHT, csiHT, csiVHT, csiHE};
subcSets  = {subcarrierIndicesNonHT, subcarrierIndicesHT20, ...
             subcarrierIndicesVHT20, subcarrierIndicesHE20};
frameSets = {frameTypeNonHT, frameTypeHT, frameTypeVHT, frameTypeHE};
fcsSets   = {fcsNonHT, fcsHT, fcsVHT, fcsHE};

primaryFormat     = names{bestIdx};
csi               = csiSets{bestIdx};
subcarrierIndices = subcSets{bestIdx};
timeSec           = times{bestIdx};
frameType         = frameSets{bestIdx};
fcsVerified       = fcsSets{bestIdx};
phyFormat         = repmat({primaryFormat}, size(csi, 1), 1); %#ok<NASGU>

% 最後に読んだセグメントのメタデータを引き継ぎ、結合の情報を足す
Slast = load(fullfile(listing(end).folder, listing(end).name), 'csiMeta');
csiMeta = Slast.csiMeta;
csiMeta.merged          = true;
csiMeta.mergedFrom      = {listing.name};
csiMeta.mergedSegments  = numel(listing);
csiMeta.primaryFormat   = primaryFormat;
csiMeta.mergeLog        = mergeLog;
csiMeta.mergeDatetime   = datestr(now, 'yyyy-mm-dd HH:MM:SS');

sampleRate = 20e6;
if isfield(csiMeta, 'sampleRate')
    sampleRate = csiMeta.sampleRate;
end
packetStartIndex = round(timeSec(:) * sampleRate); %#ok<NASGU>

outFile = fullfile(csiSearchPath, [filePrefix '_merged_CSI.mat']);
save(outFile, ...
    'csi', 'subcarrierIndices', 'packetStartIndex', 'csiMeta', ...
    'phyFormat', 'fcsVerified', 'timeSec', 'frameType', ...
    'csiNonHT', 'timeSecNonHT', 'frameTypeNonHT', 'fcsNonHT', ...
    'csiHT', 'timeSecHT', 'frameTypeHT', 'fcsHT', ...
    'csiVHT', 'timeSecVHT', 'frameTypeVHT', 'fcsVHT', ...
    'csiHE', 'timeSecHE', 'frameTypeHE', 'fcsHE', ...
    'subcarrierIndicesNonHT', 'subcarrierIndicesHT20', ...
    'subcarrierIndicesVHT20', 'subcarrierIndicesHE20', ...
    '-v7.3');

fprintf('\n結合結果を保存しました: %s\n', outFile);
fprintf('主データ(変数 csi): %s フォーマット, %d パケット x %d サブキャリア\n', ...
    primaryFormat, size(csi, 1), size(csi, 2));
fprintf(['\nResultCSI.m の inputCsiFile にこのファイルを指定すれば、\n', ...
         '120 秒通しの振幅・位相マップが得られます。\n']);

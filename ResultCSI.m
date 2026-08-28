%% ResultCSI.m
% =========================================================================
%  CSI (.mat) から振幅 (Amplitude) マップと位相 (Phase) マップを
%  フォーマット (Non-HT / HT / VHT / HE) ごとに作成・保存するスクリプト
% -------------------------------------------------------------------------
%  概要:
%    decodeIQ_VHT.m / decode_VHT_v2.m / decodeIQ_VHT_Pico.m / decodeIQ_HE.m
%    の出力 (<入力名>_CSI.mat) を読み込み、
%    Non-HT / HT / VHT / HE のうちファイルに含まれているものそれぞれについて、
%      振幅マップ : Amplitude(packet, subcarrier) = |H(k)|        [dB]
%      位相マップ : Phase(packet, subcarrier)     = angle(H(k))  [rad]
%    を [パケット数 x サブキャリア数] の行列として計算し、フォーマットごとに
%    別の figure としてヒートマップ表示、まとめて .mat ファイルに保存します。
%    (Non-HT/HT は 52 本、VHT(20MHz) は 56 本、HE(20MHz) は 242 本と
%     サブキャリア数が異なるため、1 つのマップに混在させると軸がずれて
%     意味を成しません。必ず別マップに分けて出力します。
%     さらに HE は 20MHz でも 256点FFT (78.125kHz間隔) なので、
%     サブキャリア番号 k の刻み幅そのものが他フォーマットと違います。)
%
%    "csiNonHT/csiHT/csiVHT/csiHE" 系の変数が無い古い形式の .mat の場合は、
%    単一の "csi" 変数から1つだけマップを作成します (フォーマット名は
%    csiMeta.primaryFormat があればそれを使い、無ければ 'CSI' とする)。
%
%  出力ファイル名:
%    <入力(_CSI.mat)のベース名>_result.mat
%    例: 202607221405_CSI.mat から読み込んだ場合
%        -> 202607221405_CSI_result.mat
%
%  出力 .mat の内容:
%    results  … struct配列。要素ごとに以下を持つ
%                 .format            'Non-HT' / 'HT' / 'VHT' / 'HE' など
%                 .amplitude         [numPackets x numSubcarrier] dB
%                 .phase             [numPackets x numSubcarrier] rad (wrap)
%                 .phaseUnwrap       [numPackets x numSubcarrier] rad
%                 .subcarrierIndices [1 x numSubcarrier]
%                 .timeAxis          [numPackets x 1] 相対時刻 [s]
%                 .numPackets
%    resultMeta … 処理条件・元メタデータ
%
%  必要環境:
%    - MATLAB (基本機能のみ)
% =========================================================================

clear; clc;

%% ------------------------------------------------------------------------
%  1. ユーザ設定
%  ------------------------------------------------------------------------
% 入力 CSI ファイル (decodeIQ_*.m / decode_VHT_v2.m の出力)。
%   '' の場合は csiSearchPath 内の最新の *_CSI.mat を自動選択。
inputCsiFile = '';

% CSI ファイルを探すフォルダ。inputCsiFile 指定時は無視。
%   decodeIQ_VHT.m の hddSavePath と揃えること。現在の環境: HDPC-UT (D:)
csiSearchPath = 'D:\IQ_csi';

%% ------------------------------------------------------------------------
%  2. CSI データの読み込み
%  ------------------------------------------------------------------------
if isempty(inputCsiFile)
    d = dir(fullfile(csiSearchPath, '*_CSI.mat'));
    if isempty(d)
        error('ResultCSI:noInput', ...
            'CSI の .mat (*_CSI.mat) が見つかりません: %s', csiSearchPath);
    end
    [~, iLatest] = max([d.datenum]);
    inputCsiFile = fullfile(d(iLatest).folder, d(iLatest).name);
end

if ~isfile(inputCsiFile)
    error('ResultCSI:fileNotFound', '入力ファイルがありません: %s', inputCsiFile);
end

fprintf('入力 CSI ファイル: %s\n', inputCsiFile);

% load が失敗する主因は「decodeIQ_VHT.m の保存が完了していない (実行中・中断)」
% ため。原因が分かるようファイルサイズを添えて報告する。
try
    S = load(inputCsiFile);
catch loadErr
    dInfo = dir(inputCsiFile);
    error('ResultCSI:loadFailed', ...
        ['CSI ファイルを読み込めませんでした。\n', ...
         '  ファイル  : %s\n', ...
         '  サイズ    : %d バイト\n', ...
         '  更新日時  : %s\n', ...
         '  エラー    : %s\n', ...
         'decodeIQ_VHT.m が「すべての処理が完了しました。」まで到達しているか\n', ...
         '確認してください。実行中・中断されたファイルは読み込めません。'], ...
        inputCsiFile, dInfo.bytes, dInfo.date, loadErr.message);
end

sampleRate = NaN;
if isfield(S, 'csiMeta') && isfield(S.csiMeta, 'sampleRate')
    sampleRate = S.csiMeta.sampleRate;
end

%% ------------------------------------------------------------------------
%  3. フォーマットごとのデータセットを集める
%  ------------------------------------------------------------------------
% フォーマット別形式 (csiNonHT/csiHT/csiVHT/csiHE ...) を優先的に探す。
% HE は decodeIQ_HE.m の出力にのみ含まれる (無ければ黙って読み飛ばす)。
candidates = { ...
    'Non-HT', 'csiNonHT', 'subcarrierIndicesNonHT', 'timeSecNonHT'; ...
    'HT',     'csiHT',    'subcarrierIndicesHT20',  'timeSecHT';    ...
    'VHT',    'csiVHT',   'subcarrierIndicesVHT20', 'timeSecVHT';   ...
    'HE',     'csiHE',    'subcarrierIndicesHE20',  'timeSecHE'     ...
    };

datasets = struct('format', {}, 'csi', {}, 'subcarrierIndices', {}, 'timeSec', {});
for r = 1:size(candidates, 1)
    [fmtName, csiVar, subcVar, timeVar] = candidates{r, :};
    if ~isfield(S, csiVar) || ~isfield(S, subcVar)
        continue;
    end
    csiData = normalizeToMatrix(S.(csiVar));
    if isempty(csiData)
        continue;   % このフォーマットのパケットは0件
    end
    if isfield(S, timeVar) && numel(S.(timeVar)) == size(csiData, 1)
        tSec = S.(timeVar)(:);
    else
        tSec = (0:size(csiData,1)-1).';   % フォールバック: パケット番号
    end
    datasets(end+1) = struct('format', fmtName, 'csi', csiData, ...
        'subcarrierIndices', S.(subcVar), 'timeSec', tSec); %#ok<SAGROW>
end

% フォーマット別の変数が見つからない場合は、単一の csi 変数にフォールバック
% (フォーマット別変数を持たない古い形式の .mat を読むため)
if isempty(datasets)
    if ~isfield(S, 'csi') || ~isfield(S, 'subcarrierIndices')
        error('ResultCSI:invalidInput', ...
            ['変数 csi / subcarrierIndices、または csiNonHT 等のフォーマット別\n', ...
             '変数が見つかりません: %s\nファイル内の変数: %s'], ...
            inputCsiFile, strjoin(fieldnames(S).', ', '));
    end

    csiData = normalizeToMatrix(S.csi);
    if isempty(csiData)
        error('ResultCSI:emptyCSI', ...
            'CSI が空です (パケットが検出できていません)。検出条件を見直してください。');
    end

    numPackets = size(csiData, 1);
    if isfield(S, 'timeSec') && numel(S.timeSec) == numPackets
        tSec = S.timeSec(:);
    elseif isfield(S, 'packetStartIndex') && ~isnan(sampleRate) && sampleRate > 0 ...
            && numel(S.packetStartIndex) == numPackets
        tSec = S.packetStartIndex(:) / sampleRate;
    else
        tSec = (0:numPackets-1).';
    end

    fmtName = 'CSI';
    if isfield(S, 'csiMeta') && isfield(S.csiMeta, 'primaryFormat')
        fmtName = S.csiMeta.primaryFormat;
    end

    datasets(end+1) = struct('format', fmtName, 'csi', csiData, ...
        'subcarrierIndices', S.subcarrierIndices, 'timeSec', tSec);
end

fprintf('\n検出されたフォーマット: %s\n', strjoin({datasets.format}, ', '));
for k = 1:numel(datasets)
    fprintf('  %-6s: %d パケット x %d サブキャリア\n', datasets(k).format, ...
        size(datasets(k).csi, 1), size(datasets(k).csi, 2));
end

% 複数フォーマットがある場合、時間軸は捕捉開始からの相対時間で揃える
% (フォーマットごとに別々の原点にすると比較できないため)
t0 = min(cellfun(@(t) min(t), {datasets.timeSec}));

%% ------------------------------------------------------------------------
%  4. フォーマットごとに振幅・位相マップを計算して表示
%  ------------------------------------------------------------------------
results = struct('format', {}, 'amplitude', {}, 'phase', {}, 'phaseUnwrap', {}, ...
    'subcarrierIndices', {}, 'timeAxis', {}, 'numPackets', {});

for k = 1:numel(datasets)
    ds = datasets(k);
    amplitude   = 20 * log10(abs(ds.csi) + eps);   % [numPackets x numSubcarrier] dB
    phase       = angle(ds.csi);                    % [numPackets x numSubcarrier] rad
    phaseUnwrap = unwrap(phase, [], 2);              % サブキャリア方向にアンラップ
    timeAxis    = ds.timeSec - t0;

    figure('Name', sprintf('CSI Amplitude / Phase Map (%s)', ds.format));

    subplot(1,2,1);
    imagesc(timeAxis, ds.subcarrierIndices, amplitude.');
    axis xy;
    colormap(gca, 'jet');
    cb1 = colorbar;
    ylabel(cb1, 'Amplitude [dB]');
    xlabel('Time [s]');
    ylabel('Subcarrier index k');
    title(sprintf('%s Amplitude Map (%d packets)', ds.format, size(ds.csi,1)));

    subplot(1,2,2);
    imagesc(timeAxis, ds.subcarrierIndices, phaseUnwrap.');
    axis xy;
    colormap(gca, 'jet');
    cb2 = colorbar;
    ylabel(cb2, 'Phase [rad]');
    xlabel('Time [s]');
    ylabel('Subcarrier index k');
    title(sprintf('%s Phase Map (unwrapped)', ds.format));

    results(end+1) = struct('format', ds.format, 'amplitude', amplitude, ...
        'phase', phase, 'phaseUnwrap', phaseUnwrap, ...
        'subcarrierIndices', ds.subcarrierIndices, 'timeAxis', timeAxis, ...
        'numPackets', size(ds.csi, 1)); %#ok<SAGROW>
end

%% ------------------------------------------------------------------------
%  5. 保存 (.mat) — ファイル名末尾に "_result" を付加
%  ------------------------------------------------------------------------
[inDir, inBase, ~] = fileparts(inputCsiFile);
outMatFile = fullfile(inDir, [inBase '_result.mat']);

resultMeta = struct();
resultMeta.description       = 'Amplitude/Phase maps computed from CSI, split by PHY format';
resultMeta.sourceFile        = inputCsiFile;
resultMeta.formats           = {results.format};
resultMeta.amplitudeUnit     = 'dB (20*log10|H(k)|)';
resultMeta.phaseUnit         = 'rad';
resultMeta.processedDatetime = datestr(now, 'yyyymmddHHMMSS');
if isfield(S, 'csiMeta')
    resultMeta.sourceCsiMeta = S.csiMeta;        % 元スクリプトの処理条件を継承
end

save(outMatFile, 'results', 'resultMeta', '-v7.3');

fprintf('\nAmplitude/Phase マップ (%d フォーマット分) を保存しました: %s\n', ...
    numel(results), outMatFile);
fprintf('すべての処理が完了しました。\n');

%% ------------------------------------------------------------------------
%  ローカル関数
%  ------------------------------------------------------------------------
function M = normalizeToMatrix(csiRaw)
    % パケットごとの CSI をセル配列で受け取った場合 (古い形式) は
    % [パケット数 x サブキャリア数] の行列に変換する。既に行列ならそのまま。
    % サブキャリア数が揃わないものが混ざっていた場合は最頻の長さに合わせる。
    if isempty(csiRaw)
        M = [];
        return;
    end
    if iscell(csiRaw)
        lens = cellfun(@numel, csiRaw);
        keep = (lens == mode(lens));
        M = cat(1, csiRaw{keep});
    else
        M = csiRaw;
    end
end

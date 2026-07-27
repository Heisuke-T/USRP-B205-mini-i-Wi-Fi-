%% ResultCSI.m
% =========================================================================
%  calculateCSI.m で計算した CSI (.mat) から
%  振幅 (Amplitude) マップと位相 (Phase) マップを作成・保存するスクリプト
% -------------------------------------------------------------------------
%  概要:
%    calculateCSI.m の出力 (<入力名>_CSI.mat, 変数 csi / subcarrierIndices /
%    packetStartIndex / csiMeta) を読み込み、
%      振幅マップ : Amplitude(packet, subcarrier) = |H(k)|        [dB]
%      位相マップ : Phase(packet, subcarrier)     = angle(H(k))  [rad]
%    をそれぞれ [パケット数 x サブキャリア数] の行列として計算し、
%    ヒートマップとして表示、さらに .mat ファイルとして保存します。
%
%  出力ファイル名:
%    <入力(_CSI.mat)のベース名>_result.mat
%    例: 202607221405_CSI.mat から読み込んだ場合
%        -> 202607221405_CSI_result.mat
%
%  出力 .mat の内容:
%    amplitude   … [numPackets x numSubcarrier] 振幅 [dB] (20*log10|H(k)|)
%    phase       … [numPackets x numSubcarrier] 位相 [rad] (-pi..pi, wrap)
%    phaseUnwrap … [numPackets x numSubcarrier] 位相 [rad]
%                  (サブキャリア方向にアンラップ済み)
%    subcarrierIndices … [1 x numSubcarrier] 使用サブキャリア番号 k
%    timeAxis    … [numPackets x 1] 各パケットの取得時刻 [s] (先頭からの相対時間)
%    resultMeta  … 処理条件・元メタデータ
%
%  必要環境:
%    - MATLAB (基本機能のみ)
% =========================================================================

clear; clc;

%% ------------------------------------------------------------------------
%  1. ユーザ設定
%  ------------------------------------------------------------------------
% 入力 CSI ファイル (calculateCSI.m の出力, "*_CSI.mat")。
%   '' の場合は usbSavePath 内の最新の *_CSI.mat を自動選択。
inputCsiFile = '';

% CSI ファイルを探すフォルダ (USB 外部ストレージ)。inputCsiFile 指定時は無視。
usbSavePath  = 'D:\IQ';

%% ------------------------------------------------------------------------
%  2. CSI データの読み込み
%  ------------------------------------------------------------------------
if isempty(inputCsiFile)
    d = dir(fullfile(usbSavePath, '*_CSI.mat'));
    if isempty(d)
        error('ResultCSI:noInput', ...
            'CSI の .mat (*_CSI.mat) が見つかりません: %s', usbSavePath);
    end
    [~, iLatest] = max([d.datenum]);
    inputCsiFile = fullfile(d(iLatest).folder, d(iLatest).name);
end

if ~isfile(inputCsiFile)
    error('ResultCSI:fileNotFound', '入力ファイルがありません: %s', inputCsiFile);
end

fprintf('入力 CSI ファイル: %s\n', inputCsiFile);
S = load(inputCsiFile);

if ~isfield(S, 'csi') || ~isfield(S, 'subcarrierIndices')
    error('ResultCSI:invalidInput', ...
        ['変数 csi / subcarrierIndices が見つかりません。calculateCSI.m または ', ...
         'captureIQ_v2.m の出力を指定してください: %s\n', ...
         'ファイル内の変数: %s'], ...
        inputCsiFile, strjoin(fieldnames(S).', ', '));
end

csi               = S.csi;                     % [numPackets x numSubcarrier] complex
subcarrierIndices = S.subcarrierIndices;        % [1 x numSubcarrier]

% captureIQ_v2.m は互換性のため行列で保存するが、古い形式 (パケットごとの
% セル配列) を読み込んだ場合はここで行列に変換する。
if iscell(csi)
    lens = cellfun(@numel, csi);
    keep = (lens == mode(lens));
    csi  = cat(1, csi{keep});
end

[numPackets, numSubcarrier] = size(csi);
fprintf('  パケット数: %d, サブキャリア数: %d\n', numPackets, numSubcarrier);

if numPackets == 0
    error('ResultCSI:emptyCSI', ...
        'CSI が空です (パケットが検出できていません)。検出条件を見直してください。');
end

% 時間軸 (取得開始からの相対時間 [s])
sampleRate = NaN;
if isfield(S, 'csiMeta') && isfield(S.csiMeta, 'sampleRate')
    sampleRate = S.csiMeta.sampleRate;
end

if isfield(S, 'timeSec') && numel(S.timeSec) == numPackets
    % captureIQ_v2.m は秒単位の時刻を直接保存している
    timeAxis = S.timeSec(:) - S.timeSec(1);
elseif isfield(S, 'packetStartIndex') && ~isnan(sampleRate) && sampleRate > 0 ...
        && numel(S.packetStartIndex) == numPackets
    timeAxis = (S.packetStartIndex(:) - S.packetStartIndex(1)) / sampleRate;  % [s]
else
    timeAxis = (0:numPackets-1).';   % フォールバック: パケット番号を使用
end

%% ------------------------------------------------------------------------
%  3. 振幅マップ・位相マップの計算
%  ------------------------------------------------------------------------
amplitude   = 20 * log10(abs(csi) + eps);       % [numPackets x numSubcarrier] dB
phase       = angle(csi);                        % [numPackets x numSubcarrier] rad (-pi..pi)
phaseUnwrap = unwrap(phase, [], 2);              % サブキャリア方向 (周波数方向) にアンラップ

%% ------------------------------------------------------------------------
%  4. マップの表示
%  ------------------------------------------------------------------------
figure('Name', 'CSI Amplitude / Phase Map');

subplot(1,2,1);
imagesc(timeAxis, subcarrierIndices, amplitude.');
axis xy;
colormap(gca, 'jet');
cb1 = colorbar;
ylabel(cb1, 'Amplitude [dB]');
xlabel('Time [s]');
ylabel('Subcarrier index k');
title('CSI Amplitude Map');

subplot(1,2,2);
imagesc(timeAxis, subcarrierIndices, phaseUnwrap.');
axis xy;
colormap(gca, 'jet');
cb2 = colorbar;
ylabel(cb2, 'Phase [rad]');
xlabel('Time [s]');
ylabel('Subcarrier index k');
title('CSI Phase Map (unwrapped)');

%% ------------------------------------------------------------------------
%  5. 保存 (.mat) — ファイル名末尾に "_result" を付加
%  ------------------------------------------------------------------------
[inDir, inBase, ~] = fileparts(inputCsiFile);
outMatFile = fullfile(inDir, [inBase '_result.mat']);

resultMeta = struct();
resultMeta.description       = 'Amplitude/Phase map computed from CSI';
resultMeta.sourceFile        = inputCsiFile;
resultMeta.numPackets        = numPackets;
resultMeta.numSubcarrier     = numSubcarrier;
resultMeta.amplitudeUnit     = 'dB (20*log10|H(k)|)';
resultMeta.phaseUnit         = 'rad';
resultMeta.processedDatetime = datestr(now, 'yyyymmddHHMMSS');
if isfield(S, 'csiMeta')
    resultMeta.sourceCsiMeta = S.csiMeta;        % calculateCSI.m の処理条件を継承
end

save(outMatFile, 'amplitude', 'phase', 'phaseUnwrap', 'subcarrierIndices', ...
    'timeAxis', 'resultMeta', '-v7.3');

fprintf('\nAmplitude/Phase マップを保存しました: %s\n', outMatFile);
fprintf('すべての処理が完了しました。\n');

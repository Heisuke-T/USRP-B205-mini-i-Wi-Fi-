%% calculateCSI.m
% =========================================================================
%  captureIQ.m で保存した Wi-Fi IQ 信号から CSI を計算して保存するスクリプト
% -------------------------------------------------------------------------
%  概要:
%    captureIQ.m が USB に保存した .mat ファイル (変数 iq / meta) を読み込み、
%    IEEE 802.11a/g/n のレガシープリアンブルに含まれる
%    L-LTF (Legacy Long Training Field) を用いて、パケットごとの
%    サブキャリア別チャネル応答 = CSI (Channel State Information) を推定し、
%    別の .mat ファイルに保存します。
%
%    5GHz 帯 20MHz チャネル (例: ch36) を想定。FFT サイズ 64、
%    使用サブキャリア -26..-1, 1..26 (計 52 本、DC/ガードは null)。
%
%  処理の流れ:
%    1. IQ (.mat) の読み込み
%    2. L-STF の遅延相関によるパケット検出 (粗タイミング)
%    3. L-LTF の相互相関による精タイミング同期
%    4. L-LTF 2 シンボルから周波数オフセット補正 + チャネル推定
%    5. パケットごとの CSI を行列にまとめて .mat 保存
%
%  必要環境:
%    - MATLAB (基本機能のみ。WLAN Toolbox は不要)
%    ※ WLAN Toolbox がある場合は wlanPacketDetect / wlanLLTFChannelEstimate
%      等を使うとより高精度に推定できます (本スクリプトは自己完結実装)。
%
%  出力ファイル:
%    <入力名>_CSI.mat
%      csi                … [numPackets x 52] complex, パケット別 CSI (H(k))
%      subcarrierIndices  … [1 x 52] 使用サブキャリア番号 k (-26..-1,1..26)
%      packetStartIndex   … [numPackets x 1] 各パケットの L-LTF 開始サンプル位置
%      csiMeta            … 処理条件・元メタデータ
% =========================================================================

clear; clc;

%% ------------------------------------------------------------------------
%  1. ユーザ設定
%  ------------------------------------------------------------------------
% 入力 IQ ファイル (captureIQ.m の出力)。
%   '' の場合は usbSavePath 内の最新の yyyymmddHHMM.mat を自動選択。
inputMatFile = '';

% IQ ファイルを探すフォルダ (USB 外部ストレージ)。inputMatFile 指定時は無視。
usbSavePath  = fullfile('E:', 'iq_capture');   % ★環境に合わせて変更

% --- 検出パラメータ ------------------------------------------------------
detThreshold   = 0.6;    % L-STF 遅延相関のしきい値 (0〜1)
minPlateauLen  = 32;     % プラトーと見なす最小連続長 [samples]
minPacketGap   = 400;    % パケット間の最小間隔 [samples] (重複検出防止)
ltfSearchSpan  = 80;     % L-LTF 精同期の探索窓 [samples]

%% ------------------------------------------------------------------------
%  2. IQ データの読み込み
%  ------------------------------------------------------------------------
if isempty(inputMatFile)
    % usbSavePath 内で _CSI を除く最新の .mat を選択
    d = dir(fullfile(usbSavePath, '*.mat'));
    d = d(~contains({d.name}, '_CSI'));
    if isempty(d)
        error('calculateCSI:noInput', ...
            'IQ の .mat が見つかりません: %s', usbSavePath);
    end
    [~, iLatest] = max([d.datenum]);
    inputMatFile = fullfile(d(iLatest).folder, d(iLatest).name);
end

if ~isfile(inputMatFile)
    error('calculateCSI:fileNotFound', '入力ファイルがありません: %s', inputMatFile);
end

fprintf('入力 IQ ファイル: %s\n', inputMatFile);
S = load(inputMatFile);
if ~isfield(S, 'iq')
    error('calculateCSI:noIQ', '変数 iq が見つかりません: %s', inputMatFile);
end
iq = double(S.iq(:));            % complex 列ベクトルへ

if isfield(S, 'meta') && isfield(S.meta, 'sampleRate')
    fs = S.meta.sampleRate;
else
    fs = 20e6;                    % 既定 20 MHz
    warning('calculateCSI:noMeta', 'meta.sampleRate 不明のため %g を使用', fs);
end

if abs(fs - 20e6) > 1
    warning('calculateCSI:sampleRate', ...
        ['サンプルレートが 20 MHz ではありません (%.3f MHz)。本スクリプトは ', ...
         '20MHz/64FFT を前提とするため、必要に応じ 20MHz へリサンプルしてください。'], ...
         fs/1e6);
end

fprintf('  サンプル数: %d, サンプルレート: %.3f MHz\n', numel(iq), fs/1e6);

%% ------------------------------------------------------------------------
%  3. L-LTF 参照系列 (周波数領域) と時間領域波形
%  ------------------------------------------------------------------------
% 802.11a/g L-LTF 周波数領域系列 (サブキャリア k = -26..26, DC=0)
Lseq = [ 1  1 -1 -1  1  1 -1  1 -1  1  1  1  1  1  1 -1 -1  1  1 -1  1 -1  1  1  1  1 ...
         0 ...
         1 -1 -1  1  1 -1  1 -1  1 -1 -1 -1 -1 -1  1  1 -1 -1  1 -1  1 -1  1  1  1  1 ].';

% FFT=64 の周波数グリッド (fftshift 順: index 1..64 -> k = -32..31)
N       = 64;
Lgrid   = zeros(N, 1);
kAll    = (-26:26).';            % L-LTF が占めるサブキャリア
Lgrid(kAll + 33) = Lseq;        % k=-26 -> idx7 ... k=26 -> idx59, DC(idx33)=0

% 使用サブキャリア (DC とガードを除く 52 本)
usedIdx           = [7:32, 34:59].';      % fftshift 順のインデックス
subcarrierIndices = [-26:-1, 1:26];       % 対応する k

% 時間領域 L-LTF (1 シンボル 64 サンプル)。精同期の相互相関に使用。
tLTF = ifft(ifftshift(Lgrid));

%% ------------------------------------------------------------------------
%  4. パケット検出 (L-STF 遅延相関)
%  ------------------------------------------------------------------------
D = 16;                          % L-STF の周期 [samples]
L = 32;                          % 相関窓長 [samples]

r = iq;
if numel(r) < 320
    error('calculateCSI:tooShort', 'IQ が短すぎます (最低数百サンプル必要)。');
end

% 遅延相関 c(n) と受信エネルギー p(n) の移動和
prodv  = r(1:end-D) .* conj(r(1+D:end));     % r(n) * conj(r(n+D))
energy = abs(r(1+D:end)).^2;
csum   = movsum(prodv,  [0 L-1]);            % 前方 L サンプルの和
esum   = movsum(energy, [0 L-1]);
metric = abs(csum).^2 ./ (esum.^2 + eps);    % 0〜1 の検出メトリック

% しきい値を超える連続区間 (プラトー) を抽出
above  = metric > detThreshold;
dAbove = diff([0; above; 0]);
runStart = find(dAbove == 1);
runEnd   = find(dAbove == -1) - 1;
plateaus = runStart(runEnd - runStart + 1 >= minPlateauLen);

% パケット間隔でデデュープ
pktStf = [];
lastPos = -inf;
for i = 1:numel(plateaus)
    if plateaus(i) - lastPos >= minPacketGap
        pktStf(end+1, 1) = plateaus(i); %#ok<SAGROW>
        lastPos = plateaus(i);
    end
end

fprintf('検出パケット候補数: %d\n', numel(pktStf));

%% ------------------------------------------------------------------------
%  5. 各パケットの精同期 + CSI 推定
%  ------------------------------------------------------------------------
csi              = [];
packetStartIndex = [];
tLTFn            = tLTF / norm(tLTF);         % 相互相関用に正規化

for i = 1:numel(pktStf)
    s = pktStf(i);

    % L-LTF は STF(160) の後。粗位置 s から探索窓を設定。
    winStart = s + 128;
    winEnd   = s + 128 + ltfSearchSpan;
    if winEnd + 2*N - 1 > numel(r)
        continue;                             % 末尾が足りない
    end

    % 時間領域 L-LTF との正規化相互相関でシンボル先頭を探索
    xc = zeros(winEnd - winStart + 1, 1);
    for n = winStart:winEnd
        seg = r(n:n+N-1);
        xc(n - winStart + 1) = abs(seg' * tLTFn) / (norm(seg) + eps);
    end
    [xcPeak, iPk] = max(xc);
    if xcPeak < 0.3
        continue;                             % LTF らしいピーク無し
    end
    p = winStart + iPk - 1;                    % 第 1 L-LTF シンボルの先頭

    if p + 2*N - 1 > numel(r)
        continue;
    end

    % --- 周波数オフセット (CFO) 補正: 2 シンボル間の位相回転から推定 ---
    ltfRegion = r(p : p + 2*N - 1);
    sym1 = ltfRegion(1:N);
    sym2 = ltfRegion(N+1:2*N);
    phi  = angle(sum(conj(sym1) .* sym2));     % 64 サンプルあたりの位相回転
    cfo  = phi / N;                            % [rad/sample]
    nvec = (0:2*N-1).';
    ltfCorr = ltfRegion .* exp(-1j * cfo * nvec);
    sym1 = ltfCorr(1:N);
    sym2 = ltfCorr(N+1:2*N);

    % --- チャネル推定: 各シンボルを FFT し既知系列で除算、2 本を平均 ---
    Y1 = fftshift(fft(sym1));
    Y2 = fftshift(fft(sym2));
    H1 = Y1(usedIdx) ./ Lgrid(usedIdx);
    H2 = Y2(usedIdx) ./ Lgrid(usedIdx);
    H  = (H1 + H2) / 2;                         % [52 x 1] CSI

    csi(end+1, :)          = H.';               %#ok<SAGROW> [pkt x 52]
    packetStartIndex(end+1, 1) = p;             %#ok<SAGROW>
end

numPackets = size(csi, 1);
fprintf('CSI を推定できたパケット数: %d\n', numPackets);
if numPackets == 0
    warning('calculateCSI:noPacket', ...
        ['Wi-Fi パケットを検出できませんでした。ゲイン/中心周波数/しきい値 ', ...
         '(detThreshold) を見直すか、実際にトラフィックがある状態で取得してください。']);
end

%% ------------------------------------------------------------------------
%  6. 保存 (.mat)
%  ------------------------------------------------------------------------
[inDir, inBase, ~] = fileparts(inputMatFile);
outMatFile = fullfile(inDir, [inBase '_CSI.mat']);

csiMeta = struct();
csiMeta.description       = 'CSI estimated from L-LTF of captured Wi-Fi IQ';
csiMeta.sourceFile        = inputMatFile;
csiMeta.sampleRate        = fs;
csiMeta.fftSize           = N;
csiMeta.numUsedSubcarrier = numel(subcarrierIndices);
csiMeta.detThreshold      = detThreshold;
csiMeta.numPackets        = numPackets;
csiMeta.processedDatetime = datestr(now, 'yyyymmddHHMMSS');
if isfield(S, 'meta')
    csiMeta.sourceMeta = S.meta;               % 元の取得条件を継承
end

save(outMatFile, 'csi', 'subcarrierIndices', 'packetStartIndex', 'csiMeta', '-v7.3');
fprintf('\nCSI を保存しました: %s\n', outMatFile);

%% ------------------------------------------------------------------------
%  7. (任意) 簡易プロット
%  ------------------------------------------------------------------------
if numPackets > 0
    figure('Name', 'CSI');
    subplot(2,1,1);
    plot(subcarrierIndices, 20*log10(abs(csi(1,:))), '-o');
    grid on; xlabel('Subcarrier index k'); ylabel('|H(k)| [dB]');
    title(sprintf('CSI amplitude (packet 1 / %d)', numPackets));
    subplot(2,1,2);
    plot(subcarrierIndices, unwrap(angle(csi(1,:))), '-o');
    grid on; xlabel('Subcarrier index k'); ylabel('\angle H(k) [rad]');
    title('CSI phase (packet 1)');
end

fprintf('すべての処理が完了しました。\n');

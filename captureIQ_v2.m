%% captureIQ_v2.m
% =========================================================================
%  USRP B205 mini-i でWi-Fiをリアルタイム受信し、
%  WLAN Toolbox でパケットを復号して「指定SSIDのAPだけ」のCSIを記録する
% -------------------------------------------------------------------------
%  概要:
%    captureIQ.m (生IQをそのまま保存) とは異なり、本スクリプトは受信しながら
%    その場で WLAN Toolbox を使ってパケットを検出・復号し、
%      1) Beacon/Probe Response フレームから SSID と 送信元アドレス(BSSID) の
%         対応関係を学習
%      2) 復号できた全パケットについて、プリアンブル (Non-HTはL-LTF、VHTは
%         VHT-LTF) から CSI (チャネル応答 H(k)) と送信元アドレス(Address2)
%         を記録
%      3) キャプチャ終了後、学習した BSSID の中から targetSSID に一致する
%         ものを探し、そのBSSIDのパケットのCSIだけを抽出して .mat 保存
%    という処理を行います。
%
%  対応フォーマット:
%    Non-HT (802.11a/g レガシー) と VHT (802.11ac / Wi-Fi 5, SU-VHTのみ) に
%    対応しています。HT (802.11n / Wi-Fi 4) と HE (802.11ax / Wi-Fi 6) は
%    フォーマット検出のみ行い、CSI 記録の対象からは除外されます。
%
%    VHT対応の制約 (いずれもUSRP B205 mini-iが受信アンテナ1本=SISOのため):
%      - NSTS(空間ストリーム数) = 1 のパケットのみ復号します。
%        マルチストリーム(NSTS>=2)は1本アンテナでは原理的に復号できません。
%      - 20MHz動作 (VHT-SIG-A の BW フィールドが 20MHz) のパケットのみ
%        対象です。40/80/160MHzで送信されたVHTパケットは、本機の受信帯域
%        (20MHz) では正しく復号できません。
%      - SU-VHT (GroupID = 0 または 63) のみ対応。MU-MIMO(GroupID 1-62)は
%        非対応です。
%      - STBC を使用しているパケットは非対応としてスキップします。
%    上記に該当しないVHTパケットは検出はされますが、復号エラーとして
%    読み捨てられ、CSI記録の対象外となります(キャプチャ自体は継続します)。
%
%    VHT-SIG-A/VHT-SIG-B のビットフィールド解釈は WLAN Toolbox が自動で
%    行ってくれる関数が存在しないため、IEEE Std 802.11ac-2013 の仕様
%    (Table 22-12 等) に基づき本スクリプト内で実装しています。実機での
%    検証を行っていないため、特に VHT-SIG-B の Length フィールドの解釈
%    (20MHzでは 17bit, 4byte単位と想定) は動作しない場合に最初に疑うべき
%    箇所です。うまく復号できない場合はコメントを参照し調整してください。
%
%  必要環境:
%    - MATLAB
%    - Communications Toolbox
%    - Communications Toolbox Support Package for USRP Radio
%    - WLAN Toolbox
%    - USB 3.0 接続された USRP B205 mini-i
%
%  出力ファイル:
%    <保存先>/<yyyymmddHHMM>_<SSID>_CSI.mat
%      csi          … {numPackets x 1} セル配列。各セルは1パケット分の
%                      CSI (H(k)) の行ベクトル (Non-HTなら52本、VHT-20なら
%                      56本と、フォーマットにより長さが異なるためセル配列)
%      phyFormat    … {numPackets x 1} cellstr, 各パケットの物理層フォーマット
%                      ('Non-HT' または 'VHT')
%      subcarrierIndicesNonHT … [1 x 52] Non-HT の使用サブキャリア番号
%      subcarrierIndicesVHT20 … [1 x 56] VHT-20MHz の使用サブキャリア番号
%      timeSec      … [numPackets x 1] キャプチャ開始からの相対時刻 [s]
%      frameType    … [numPackets x 1] cellstr, 各パケットのフレーム種別
%      targetSSIDOut  … 指定した SSID 文字列
%      targetBSSIDOut … 学習された対象 AP の MAC アドレス文字列
%      seenNetworks … 受信中に検出できた全 BSSID/SSID の一覧 (診断用)
%      resultMeta   … 処理条件
%
%  注意:
%    電波の受信・記録は、利用地域の電波法および関連法令を遵守し、
%    自身が管理する機器・許可された環境でのみ実施してください。
%    本スクリプトは MATLAB 上での実機テストを前提に作成しています。
%    WLAN Toolbox のバージョンによっては関数の挙動に差異がある場合が
%    あるため、まずは短時間 (1秒程度) のキャプチャで動作確認してください。
% =========================================================================

clear; clc;

%% ------------------------------------------------------------------------
%  1. ユーザ設定パラメータ
%  ------------------------------------------------------------------------

% --- 保存先 (USB 外部ストレージ) -----------------------------------------
usbSavePath = 'D:\IQ';

% --- 抽出したい Wi-Fi の SSID --------------------------------------------
targetSSID = 'OpenWrt-A';

% --- Wi-Fi 受信パラメータ (captureIQ.m と同じ ch36 / 20MHz) --------------
centerFrequency = 5.180e9;      % [Hz] 5GHz 帯 ch36 (20MHz 帯域幅)
chanBW          = 'CBW20';      % WLAN Toolbox のチャネル帯域幅指定
sampleRate      = 20e6;         % [Sps] 20 MHz 帯域
gain            = 40;           % [dB]
captureDuration = 2.0;          % [s] Beacon 間隔は通常 100ms なので
                                 %     確実に捕捉したい場合は長めに設定
samplesPerFrame = 20000;        % [samples/frame]
usrpPlatform    = 'B200';
usrpSerialNum   = '3240497';

% --- パケット検出パラメータ ----------------------------------------------
pktDetThreshold = 0.5;          % wlanPacketDetect のしきい値 (0〜1)
maxBufferMargin = 4000;         % バッファを間引く前に保持する最大サンプル数
trimKeepSamples = 2000;         % 間引き後に残すサンプル数 (パケット境界保護用)

%% ------------------------------------------------------------------------
%  2. 保存先の準備
%  ------------------------------------------------------------------------
if ~exist(usbSavePath, 'dir')
    fprintf('保存先フォルダが存在しないため作成します: %s\n', usbSavePath);
    [ok, msg] = mkdir(usbSavePath);
    if ~ok
        error('captureIQ_v2:mkdirFailed', ...
            'USB 保存先フォルダを作成できませんでした (%s): %s', usbSavePath, msg);
    end
end

testFile = fullfile(usbSavePath, '.write_test.tmp');
fidTest  = fopen(testFile, 'w');
if fidTest == -1
    error('captureIQ_v2:usbNotWritable', ...
        'USB 保存先に書き込みできません: %s', usbSavePath);
end
fclose(fidTest);
delete(testFile);

timestamp = datestr(now, 'yyyymmddHHMM');
ssidSafe  = regexprep(targetSSID, '[^A-Za-z0-9_-]', '_');
outMatFile = fullfile(usbSavePath, [timestamp '_' ssidSafe '_CSI.mat']);

%% ------------------------------------------------------------------------
%  3. USRP B205 mini-i 受信機オブジェクトの生成
%  ------------------------------------------------------------------------
radioInfo = [];
try
    radioInfo = findsdru();
    if isempty(radioInfo)
        warning('captureIQ_v2:noRadio', ...
            'USRP 機器が検出されませんでした。USB 接続と電源を確認してください。');
    else
        fprintf('検出された USRP 機器:\n');
        for k = 1:numel(radioInfo)
            fprintf('  Platform=%s, SerialNum=%s, Status=%s\n', ...
                radioInfo(k).Platform, radioInfo(k).SerialNum, radioInfo(k).Status);
        end
    end
catch ME
    warning('captureIQ_v2:findsdruFailed', 'findsdru の実行に失敗しました: %s', ME.message);
end

if isempty(usrpSerialNum)
    error('captureIQ_v2:noSerialNum', ...
        'usrpSerialNum を指定してください (findsdru の表示を参照)。');
end

rx = comm.SDRuReceiver( ...
    'Platform',            usrpPlatform, ...
    'SerialNum',           usrpSerialNum, ...
    'CenterFrequency',     centerFrequency, ...
    'Gain',                gain, ...
    'MasterClockRate',     sampleRate * 2, ...
    'DecimationFactor',    2, ...
    'OutputDataType',      'double', ...   % WLAN Toolbox 関数は double 前提
    'SamplesPerFrame',     samplesPerFrame);

fprintf('\n受信設定:\n');
fprintf('  中心周波数      : %.4f GHz\n', centerFrequency / 1e9);
fprintf('  サンプルレート  : %.3f MSps (帯域幅)\n', sampleRate / 1e6);
fprintf('  ゲイン          : %d dB\n', gain);
fprintf('  キャプチャ時間  : %.2f s\n', captureDuration);
fprintf('  対象 SSID       : %s\n', targetSSID);
fprintf('  保存先(.mat)    : %s\n', outMatFile);

%% ------------------------------------------------------------------------
%  4. リアルタイム パケット検出・復号ループ
%  ------------------------------------------------------------------------
subcarrierIndicesNonHT = [-26:-1, 1:26];    % Non-HT CBW20 (52本)
subcarrierIndicesVHT20 = [-28:-1, 1:28];    % VHT CBW20 (56本)

pktLog = struct('timeSec', {}, 'bssid', {}, 'frameType', {}, 'ssid', {}, ...
    'phyFormat', {}, 'csi', {});
bssidToSSID = containers.Map('KeyType', 'char', 'ValueType', 'char');

buffer            = zeros(0, 1);
bufferGlobalOffset = 0;           % buffer(1) がキャプチャ全体の何サンプル目か
totalSamplesTarget = round(captureDuration * sampleRate);
totalSamplesCaptured = 0;
overrunCount = 0;

fprintf('\nキャプチャ・リアルタイム復号を開始します...\n');
cleanupObj = onCleanup(@() cleanupResources(rx));
captureTic = tic;

while totalSamplesCaptured < totalSamplesTarget
    [iqData, dataLen, overrun] = rx();
    if dataLen == 0
        continue;
    end
    if overrun
        overrunCount = overrunCount + 1;
    end

    buffer = [buffer; iqData(1:dataLen)]; %#ok<AGROW>
    totalSamplesCaptured = totalSamplesCaptured + dataLen;

    % バッファ先頭から検出・復号できるだけ処理する
    while true
        if numel(buffer) < 320
            break;   % L-STF+L-LTF に満たない
        end

        startOffset = wlanPacketDetect(buffer, chanBW, 0, pktDetThreshold);

        if isempty(startOffset)
            % 候補なし。バッファが大きくなり過ぎたら末尾だけ残して間引く
            if numel(buffer) > maxBufferMargin
                nDrop = numel(buffer) - trimKeepSamples;
                buffer(1:nDrop) = [];
                bufferGlobalOffset = bufferGlobalOffset + nDrop;
            end
            break;   % 次のSDRフレームを待つ
        end

        pkt = buffer(startOffset + 1 : end);
        if numel(pkt) < 320
            break;   % タイミング/CFO推定に必要な分がまだ足りない -> 次フレーム待ち
        end

        try
            % --- 粗CFO補正 (L-STF) ---
            coarseCFO = wlanCoarseCFOEstimate(pkt(1:160), chanBW);
            pkt = applyCFO(pkt, sampleRate, -coarseCFO);

            % --- シンボルタイミング精同期 (L-STF+L-LTF) ---
            fineTimingOffset = wlanSymbolTimingEstimate(pkt(1:320), chanBW);
            pkt = pkt(1 + fineTimingOffset : end);
            if numel(pkt) < 320
                break;   % 微調整後にサンプル不足 -> 次フレーム待ち
            end

            % --- 精CFO補正 (L-LTF) ---
            lltf = pkt(161:320);
            fineCFO = wlanFineCFOEstimate(lltf, chanBW);
            pkt = applyCFO(pkt, sampleRate, -fineCFO);
            lltf = pkt(161:320);

            % --- レガシー部のチャネル推定・ノイズ推定 (L-LTF) ---
            lltfChanEst = wlanLLTFChannelEstimate(lltf, chanBW);
            noiseEst    = wlanLLTFNoiseEstimate(lltf);

            % --- フォーマット検出に必要な分 (L-SIG + 2シンボル) ---
            if numel(pkt) < 560
                break;   % 次フレーム待ち
            end
            fmt = wlanFormatDetect(pkt(321:560));

            switch true
                case strcmpi(fmt, 'Non-HT')
                    [ok, advanceBy, entry] = processNonHT(pkt, startOffset, ...
                        lltfChanEst, noiseEst, chanBW, sampleRate, bufferGlobalOffset);
                    if ok < 0
                        break;   % サンプル不足 -> 次フレーム待ち (バッファ変更なし)
                    end

                case strcmpi(fmt, 'VHT')
                    [ok, advanceBy, entry] = processVHT(pkt, startOffset, ...
                        lltfChanEst, noiseEst, chanBW, sampleRate, bufferGlobalOffset);
                    if ok < 0
                        break;   % サンプル不足 -> 次フレーム待ち (バッファ変更なし)
                    end

                otherwise
                    % HT(Wi-Fi4) / HE(Wi-Fi6) は非対応。読み捨てて次を探す。
                    ok = 0;
                    advanceBy = startOffset + 320;
                    entry = [];
            end

            if ok > 0 && ~isempty(entry)
                pktLog(end+1) = entry; %#ok<SAGROW>
                if any(strcmpi(entry.frameType, {'Beacon', 'Probe Response'})) ...
                        && ~isempty(entry.ssid)
                    bssidToSSID(entry.bssid) = entry.ssid;
                end
                fprintf('  [%.3fs] %-6s %-14s BSSID=%s%s\n', entry.timeSec, ...
                    entry.phyFormat, entry.frameType, entry.bssid, ...
                    ternary(~isempty(entry.ssid), sprintf('  SSID="%s"', entry.ssid), ''));
            end

            buffer(1:advanceBy) = [];
            bufferGlobalOffset = bufferGlobalOffset + advanceBy;

        catch ME
            % 復号エラーは検出誤り(ノイズ等)や非対応パケットとしてスキップし、
            % キャプチャを継続する
            warning('captureIQ_v2:decodeError', 'パケット処理中にエラー: %s', ME.message);
            advanceBy = startOffset + 1;
            buffer(1:advanceBy) = [];
            bufferGlobalOffset = bufferGlobalOffset + advanceBy;
        end
    end
end

elapsed = toc(captureTic);
release(rx);
clear cleanupObj;

fprintf('\nキャプチャ完了。\n');
fprintf('  経過時間        : %.2f s\n', elapsed);
fprintf('  オーバーラン回数: %d\n', overrunCount);
fprintf('  復号できたパケット数: %d\n', numel(pktLog));

%% ------------------------------------------------------------------------
%  5. 対象 SSID の BSSID を特定し、CSI を抽出
%  ------------------------------------------------------------------------
seenNetworks = struct('bssid', {}, 'ssid', {});
bssidKeys = keys(bssidToSSID);
for k = 1:numel(bssidKeys)
    seenNetworks(end+1) = struct('bssid', bssidKeys{k}, 'ssid', bssidToSSID(bssidKeys{k})); %#ok<SAGROW>
end

fprintf('\n検出できたネットワーク一覧:\n');
for k = 1:numel(seenNetworks)
    fprintf('  BSSID=%s  SSID="%s"\n', seenNetworks(k).bssid, seenNetworks(k).ssid);
end

targetBSSID = '';
for k = 1:numel(seenNetworks)
    if strcmp(seenNetworks(k).ssid, targetSSID)
        targetBSSID = seenNetworks(k).bssid;
        break;
    end
end

csi = {};
phyFormat = {};
timeSec = [];
frameType = {};

if isempty(targetBSSID)
    warning('captureIQ_v2:ssidNotFound', ...
        ['指定した SSID "%s" の Beacon/Probe Response を検出できませんでした。\n', ...
         'captureDuration を長くする、SSID の綴りを確認する、電波状況を確認する' ...
         'などをお試しください。'], targetSSID);
else
    isMatch = strcmp({pktLog.bssid}, targetBSSID);
    matched = pktLog(isMatch);
    if ~isempty(matched)
        csi       = {matched.csi}.';
        phyFormat = {matched.phyFormat}.';
        timeSec   = [matched.timeSec].';
        frameType = {matched.frameType}.';
    end
    fprintf('\n対象 SSID "%s" (BSSID=%s) のパケット数: %d\n', ...
        targetSSID, targetBSSID, numel(matched));
    if ~isempty(matched)
        fprintf('  内訳: Non-HT=%d, VHT=%d\n', ...
            sum(strcmpi(phyFormat, 'Non-HT')), sum(strcmpi(phyFormat, 'VHT')));
    end
end

%% ------------------------------------------------------------------------
%  6. 保存 (.mat)
%  ------------------------------------------------------------------------
resultMeta = struct();
resultMeta.description      = 'CSI filtered by target SSID (Non-HT and SU-VHT/NSTS=1/20MHz frames)';
resultMeta.centerFrequency  = centerFrequency;
resultMeta.sampleRate       = sampleRate;
resultMeta.gain             = gain;
resultMeta.captureDuration  = captureDuration;
resultMeta.wifiChannel      = 36;
resultMeta.platform         = usrpPlatform;
resultMeta.serialNum        = usrpSerialNum;
resultMeta.overrunCount     = overrunCount;
resultMeta.totalDecodedPackets = numel(pktLog);
resultMeta.captureDatetime  = timestamp;
resultMeta.matlabVersion    = version;

targetSSIDOut = targetSSID; %#ok<NASGU>
targetBSSIDOut = targetBSSID; %#ok<NASGU>

save(outMatFile, 'csi', 'phyFormat', 'subcarrierIndicesNonHT', 'subcarrierIndicesVHT20', ...
    'timeSec', 'frameType', 'targetSSIDOut', 'targetBSSIDOut', 'seenNetworks', ...
    'resultMeta', '-v7.3');

fprintf('\n結果を保存しました: %s\n', outMatFile);
fprintf('すべての処理が完了しました。\n');

%% ------------------------------------------------------------------------
%  ローカル関数
%  ------------------------------------------------------------------------
function cleanupResources(rx)
    try
        if ~isempty(rx) && isvalid(rx)
            release(rx);
        end
    catch
    end
end

function y = applyCFO(x, fs, cfoHz)
    % x に周波数オフセット cfoHz [Hz] を回転補正として与える
    n = (0:numel(x)-1).';
    y = x .* exp(1j * 2 * pi * cfoHz * n / fs);
end

function [status, advanceBy, entry] = processNonHT(pkt, startOffset, chanEst, noiseEst, chanBW, sampleRate, bufferGlobalOffset)
    % Non-HT (802.11a/g) パケットを復号する。
    % status: 1=成功(entryあり), 0=失敗(読み捨て,entry=[]), -1=サンプル不足(待機)
    entry = [];
    advanceBy = startOffset + 320;

    % --- L-SIG 復号 (レート・PSDU長を取得) ---
    [recLSIGBits, lsigFail] = wlanLSIGRecover(pkt(321:400), chanEst, noiseEst, chanBW);
    if lsigFail
        status = 0;
        advanceBy = startOffset + 400;
        return;
    end

    [mcs, psduLen] = decodeLSIGBits(recLSIGBits);
    if isempty(mcs) || psduLen <= 0 || psduLen > 4095
        status = 0;
        advanceBy = startOffset + 400;
        return;
    end

    ndbpsTable = [24 36 48 72 96 144 192 216];   % MCS0..7 (bits/symbol)
    numDataSym = ceil((16 + 8 * psduLen + 6) / ndbpsTable(mcs + 1));
    numDataSamples = numDataSym * 80;

    if numel(pkt) < 400 + numDataSamples
        status = -1;
        return;   % データ部がまだ届いていない -> 次フレーム待ち
    end

    cfgNonHT = wlanNonHTConfig('MCS', mcs, 'PSDULength', psduLen, ...
        'ChannelBandwidth', chanBW);
    rxData = pkt(401 : 400 + numDataSamples);
    rxPSDU = wlanNonHTDataRecover(rxData, chanEst, noiseEst, cfgNonHT);

    [cfgMAC, payload, mpduStatus] = wlanMPDUDecode(rxPSDU, cfgNonHT, 'DataFormat', 'bits');
    advanceBy = startOffset + 400 + numDataSamples;

    if ~strcmpi(string(mpduStatus), "Success")
        status = 0;
        return;
    end

    frameType = char(cfgMAC.FrameType);
    ssidStr = '';
    if any(strcmpi(frameType, {'Beacon', 'Probe Response'})) && numel(payload) >= 14
        ssidStr = parseSSIDFromMgmtFrame(payload);
    end

    entry = struct( ...
        'timeSec',   (bufferGlobalOffset + startOffset) / sampleRate, ...
        'bssid',     char(cfgMAC.Address2), ...
        'frameType', frameType, ...
        'ssid',      ssidStr, ...
        'phyFormat', 'Non-HT', ...
        'csi',       chanEst(:).');
    status = 1;
end

function [status, advanceBy, entry] = processVHT(pkt, startOffset, lltfChanEst, noiseEst, chanBW, sampleRate, bufferGlobalOffset)
    % SU-VHT (802.11ac, NSTS=1, 20MHz) パケットを復号する。
    % status: 1=成功(entryあり), 0=失敗/非対応(読み捨て,entry=[]), -1=サンプル不足(待機)
    entry = [];
    advanceBy = startOffset + 560;   % 最低限、SIG-Aまでは読み進めたとみなす

    % --- VHT-SIG-A 復号 (2シンボル, pkt(401:560)) ---
    [sigaBits, sigaFail] = wlanVHTSIGARecover(pkt(401:560), lltfChanEst, noiseEst, chanBW);
    if sigaFail
        status = 0;
        return;
    end

    vhtA = parseVHTSIGABits(sigaBits);
    if ~vhtA.isValid || ~vhtA.is20MHz || ~vhtA.isSU || vhtA.nsts ~= 1 || vhtA.stbc
        % NSTS>=2(マルチストリーム,SISOでは復号不可)/MU-MIMO/STBC/非20MHzは非対応
        status = 0;
        return;
    end

    % --- VHT-STF / VHT-LTF / VHT-SIG-B の位置を取得 (仮のPSDULengthで算出) ---
    % STF/LTF/SIG-Bの位置は BW と NSTS だけで決まり PSDULength に依存しないため、
    % 仮の値(MCS=0,PSDULength=1)で構成した cfgVHT から位置を求める。
    cfgVHT0 = wlanVHTConfig('ChannelBandwidth', chanBW, 'NumSpaceTimeStreams', 1, ...
        'MCS', 0, 'PSDULength', 1, 'ChannelCoding', 'BCC', 'GuardInterval', 'Long');
    ind0 = wlanFieldIndices(cfgVHT0);

    if numel(pkt) < ind0.VHTSIGB(2)
        status = -1;
        return;   % VHT-LTF/VHT-SIG-B がまだ届いていない -> 次フレーム待ち
    end

    % --- VHT-LTF チャネル推定 (= CSI) ---
    rxVHTLTF = pkt(ind0.VHTLTF(1):ind0.VHTLTF(2));
    demodVHTLTF = wlanVHTLTFDemodulate(rxVHTLTF, chanBW, 1);
    vhtChanEst = wlanVHTLTFChannelEstimate(demodVHTLTF, chanBW, 1);

    % --- VHT-SIG-B 復号 (PSDU長を取得) ---
    rxVHTSIGB = pkt(ind0.VHTSIGB(1):ind0.VHTSIGB(2));
    sigbBits = wlanVHTSIGBRecover(rxVHTSIGB, vhtChanEst, noiseEst, chanBW);
    psduLenBytes = decodeVHTSIGBLength(sigbBits);
    if isempty(psduLenBytes) || psduLenBytes <= 0 || psduLenBytes > 65535
        status = 0;
        return;
    end

    codingStr = ternary(vhtA.coding == 1, 'LDPC', 'BCC');
    giStr     = ternary(vhtA.gi == 1, 'Short', 'Long');

    try
        cfgVHT = wlanVHTConfig('ChannelBandwidth', chanBW, 'NumSpaceTimeStreams', 1, ...
            'MCS', vhtA.mcs, 'PSDULength', psduLenBytes, 'ChannelCoding', codingStr, ...
            'GuardInterval', giStr);
        ind = wlanFieldIndices(cfgVHT);
    catch
        % MCS/長さの組み合わせが不正 (ビット解釈ミスの可能性)
        status = 0;
        return;
    end

    advanceBy = startOffset + ind.VHTData(2);

    if numel(pkt) < ind.VHTData(2)
        status = -1;
        advanceBy = startOffset + 560;   % データ未達なので進めすぎない
        return;   % VHT Data がまだ届いていない -> 次フレーム待ち
    end

    rxVHTData = pkt(ind.VHTData(1):ind.VHTData(2));
    rxPSDU = wlanVHTDataRecover(rxVHTData, vhtChanEst, noiseEst, cfgVHT);

    [cfgMAC, payload, mpduStatus] = wlanMPDUDecode(rxPSDU, cfgVHT, 'DataFormat', 'bits');

    if ~strcmpi(string(mpduStatus), "Success")
        status = 0;
        return;
    end

    frameType = char(cfgMAC.FrameType);
    ssidStr = '';
    if any(strcmpi(frameType, {'Beacon', 'Probe Response'})) && numel(payload) >= 14
        ssidStr = parseSSIDFromMgmtFrame(payload);
    end

    entry = struct( ...
        'timeSec',   (bufferGlobalOffset + startOffset) / sampleRate, ...
        'bssid',     char(cfgMAC.Address2), ...
        'frameType', frameType, ...
        'ssid',      ssidStr, ...
        'phyFormat', 'VHT', ...
        'csi',       vhtChanEst(:).');
    status = 1;
end

function [mcs, psduLen] = decodeLSIGBits(bits)
    % 802.11a/g L-SIG (24bit: RATE(4)+Reserved(1)+LENGTH(12)+Parity(1)+Tail(6))
    % を解釈し、Non-HT MCS インデックス (0..7) と PSDU長 [byte] を返す。
    % パリティ不一致の場合は mcs=[] を返す。
    bits = double(bits(:)).';

    % パリティチェック (RATE+Reserved+LENGTHの17bit + Parityビットで偶数)
    parityOk = mod(sum(bits(1:18)), 2) == 0;
    if ~parityOk
        mcs = [];
        psduLen = 0;
        return;
    end

    rateBits = bits(1:4);
    rateMap = containers.Map( ...
        {'1101','1111','0101','0111','1001','1011','0001','0011'}, ...
        {0,      1,     2,     3,     4,     5,     6,     7});
    rateKey = sprintf('%d', rateBits);
    if ~isKey(rateMap, rateKey)
        mcs = [];
        psduLen = 0;
        return;
    end
    mcs = rateMap(rateKey);

    lenBits = bits(6:17);   % LSBが先頭 (bits(6)=LSB)
    psduLen = sum(lenBits .* 2.^(0:11));
end

function out = parseVHTSIGABits(bits)
    % VHT-SIG-A (48bit = SIG-A1(24) + SIG-A2(24)) を解釈する。
    % IEEE Std 802.11ac-2013, Table 22-12 に基づく実装。
    % 注意: MCSフィールドのビット順(LSB/MSB)は実機未検証。デコード結果の
    % MCS値が明らかにおかしい場合はここを最初に疑うこと。
    bits = double(bits(:)).';
    out = struct('isValid', false, 'is20MHz', false, 'isSU', false, ...
        'nsts', 0, 'stbc', false, 'groupId', 0, 'gi', 0, 'coding', 0, ...
        'mcs', 0, 'beamformed', false);

    if numel(bits) < 48
        return;
    end

    bw       = bits(1:2);       % 00=20MHz,01=40,10=80,11=160/80+80
    stbcBit  = bits(4);
    groupId  = bits(5:10);      % 6bit
    nstsBits = bits(11:13);     % 3bit (SU: value+1 = NSTS)
    giBit    = bits(25);        % 0=Long,1=Short
    codingBit= bits(27);        % 0=BCC,1=LDPC
    mcsBits  = bits(29:32);     % 4bit (LSB-first と仮定)

    out.is20MHz = all(bw == 0);                 % 00 = 20MHz (順序非依存)
    out.isSU    = all(groupId == 0) || all(groupId == 1);  % GroupID=0 or 63
    if all(nstsBits == 0)
        out.nsts = 1;   % 全ビット0であれば、ビット順によらずNSTS=1と確定できる
    else
        out.nsts = 1 + sum(nstsBits .* 2.^(0:2));  % 参考値 (ビット順未検証)
    end
    out.stbc    = stbcBit == 1;
    out.groupId = sum(groupId .* 2.^(0:5));
    out.gi      = giBit;
    out.coding  = codingBit;
    out.mcs     = sum(mcsBits .* 2.^(0:3));
    out.isValid = true;
end

function psduLen = decodeVHTSIGBLength(bits)
    % VHT-SIG-B (20MHz, SU) の Length フィールド(17bit, LSB先頭と仮定)を
    % 解釈し、PSDU長 [byte] を返す。20MHzでは4byte単位とされているため
    % 4倍する。この換算の正確性は実機未検証 (最も不確実性が高い箇所)。
    bits = double(bits(:)).';
    if numel(bits) < 17
        psduLen = [];
        return;
    end
    lengthVal = sum(bits(1:17) .* 2.^(0:16));
    psduLen = lengthVal * 4;
end

function ssidStr = parseSSIDFromMgmtFrame(payload)
    % Beacon/Probe Response のフレームボディ(payload, uint8)から
    % 先頭 Information Element (SSID, Element ID = 0) を抽出する。
    % 固定フィールド: Timestamp(8) + BeaconInterval(2) + CapabilityInfo(2) = 12byte
    ssidStr = '';
    payload = uint8(payload(:)).';
    if numel(payload) < 14
        return;
    end
    ieOffset = 12;   % 0-indexed: 固定フィールドの直後
    elemId  = payload(ieOffset + 1);
    elemLen = payload(ieOffset + 2);
    if elemId ~= 0
        return;   % 先頭IEがSSIDでない (仕様上は通常SSIDが最初)
    end
    if numel(payload) < ieOffset + 2 + elemLen
        return;
    end
    ssidBytes = payload(ieOffset + 3 : ieOffset + 2 + elemLen);
    ssidStr = char(ssidBytes);
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end

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
%      2) 復号できた全パケットについて、L-LTF から CSI (チャネル応答 H(k)) と
%         送信元アドレス(Address2) を記録
%      3) キャプチャ終了後、学習した BSSID の中から targetSSID に一致する
%         ものを探し、そのBSSIDのパケットのCSIだけを抽出して .mat 保存
%    という処理を行います。
%
%  対応フォーマット:
%    Non-HT (802.11a/g レガシー) のみに対応しています。
%    Beacon / Probe Response / 多くの管理・制御フレームはレガシーレートで
%    送信されるため、SSID/BSSID の学習と CSI 取得には十分機能します。
%    802.11n(HT)/ac(VHT)/ax(HE) の高スループットデータフレームは
%    フォーマット検出のみ行い、CSI 記録の対象からは除外されます
%    (対応が必要な場合は別途拡張してください)。
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
%      csi          … [numPackets x 52] complex, targetSSID の BSSID から
%                      検出された各パケットの CSI (H(k))
%      subcarrierIndices … [1 x 52] 使用サブキャリア番号 k (-26..-1,1..26)
%      timeSec      … [numPackets x 1] キャプチャ開始からの相対時刻 [s]
%      frameType    … [numPackets x 1] cellstr, 各パケットのフレーム種別
%      targetSSID   … 指定した SSID 文字列
%      targetBSSID  … 学習された対象 AP の MAC アドレス文字列
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
subcarrierIndices = [-26:-1, 1:26];   % Non-HT CBW20 の使用サブキャリア (52本)

pktLog = struct('timeSec', {}, 'bssid', {}, 'frameType', {}, 'ssid', {}, 'csi', {});
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

            % --- チャネル推定 (= CSI) とノイズ推定 ---
            chanEst  = wlanLLTFChannelEstimate(lltf, chanBW);
            noiseEst = wlanLLTFNoiseEstimate(lltf);

            % --- フォーマット検出に必要な分 (L-SIG + 2シンボル) ---
            if numel(pkt) < 560
                break;   % 次フレーム待ち
            end
            fmt = wlanFormatDetect(pkt(321:560));

            if ~strcmpi(fmt, 'Non-HT')
                % HT/VHT/HE はこのスクリプトでは非対応。読み捨てて次を探す。
                advanceBy = startOffset + 320;
                buffer(1:advanceBy) = [];
                bufferGlobalOffset = bufferGlobalOffset + advanceBy;
                continue;
            end

            % --- L-SIG 復号 (レート・PSDU長を取得) ---
            [recLSIGBits, lsigFail] = wlanLSIGRecover(pkt(321:400), chanEst, noiseEst, chanBW);
            if lsigFail
                advanceBy = startOffset + 400;
                buffer(1:advanceBy) = [];
                bufferGlobalOffset = bufferGlobalOffset + advanceBy;
                continue;
            end

            [mcs, psduLen] = decodeLSIGBits(recLSIGBits);
            if isempty(mcs) || psduLen <= 0 || psduLen > 4095
                advanceBy = startOffset + 400;
                buffer(1:advanceBy) = [];
                bufferGlobalOffset = bufferGlobalOffset + advanceBy;
                continue;
            end

            ndbpsTable = [24 36 48 72 96 144 192 216];   % MCS0..7 (bits/symbol)
            numDataSym = ceil((16 + 8 * psduLen + 6) / ndbpsTable(mcs + 1));
            numDataSamples = numDataSym * 80;

            if numel(pkt) < 400 + numDataSamples
                break;   % データ部がまだ届いていない -> 次フレーム待ち
            end

            cfgNonHT = wlanNonHTConfig('MCS', mcs, 'PSDULength', psduLen, ...
                'ChannelBandwidth', chanBW);
            rxData = pkt(401 : 400 + numDataSamples);
            rxPSDU = wlanNonHTDataRecover(rxData, chanEst, noiseEst, cfgNonHT);

            [cfgMAC, payload, status] = wlanMPDUDecode(rxPSDU, cfgNonHT, 'DataFormat', 'bits');

            advanceBy = startOffset + 400 + numDataSamples;

            if strcmpi(string(status), "Success")
                bssid = char(cfgMAC.Address2);
                frameType = char(cfgMAC.FrameType);
                timeSec = (bufferGlobalOffset + startOffset) / sampleRate;

                ssidStr = '';
                if any(strcmpi(frameType, {'Beacon', 'Probe Response'})) && numel(payload) >= 14
                    ssidStr = parseSSIDFromMgmtFrame(payload);
                    if ~isempty(ssidStr)
                        bssidToSSID(bssid) = ssidStr;
                    end
                end

                pktLog(end+1) = struct( ...           %#ok<SAGROW>
                    'timeSec',   timeSec, ...
                    'bssid',     bssid, ...
                    'frameType', frameType, ...
                    'ssid',      ssidStr, ...
                    'csi',       chanEst(:).');

                fprintf('  [%.3fs] %-10s BSSID=%s%s\n', timeSec, frameType, bssid, ...
                    ternary(~isempty(ssidStr), sprintf('  SSID="%s"', ssidStr), ''));
            end

            buffer(1:advanceBy) = [];
            bufferGlobalOffset = bufferGlobalOffset + advanceBy;

        catch ME
            % 復号エラーは検出誤り(ノイズ等)としてスキップし、キャプチャを継続する
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

csi = [];
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
        csi       = cat(1, matched.csi);
        timeSec   = [matched.timeSec].';
        frameType = {matched.frameType}.';
    end
    fprintf('\n対象 SSID "%s" (BSSID=%s) のパケット数: %d\n', ...
        targetSSID, targetBSSID, numel(matched));
end

%% ------------------------------------------------------------------------
%  6. 保存 (.mat)
%  ------------------------------------------------------------------------
resultMeta = struct();
resultMeta.description      = 'CSI filtered by target SSID (Non-HT frames only)';
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

save(outMatFile, 'csi', 'subcarrierIndices', 'timeSec', 'frameType', ...
    'targetSSIDOut', 'targetBSSIDOut', 'seenNetworks', 'resultMeta', '-v7.3');

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

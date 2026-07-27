%% captureIQ_v2.m
% =========================================================================
%  USRP B205 mini-i でWi-Fiを受信し、WLAN Toolbox でパケットを復号して
%  「指定SSIDのAPだけ」のCSIを記録する
% -------------------------------------------------------------------------
%  概要:
%    処理を2段階に分けています。
%      [第1段] 生IQのキャプチャ (captureIQ.m と同じ方式)
%              復号処理を挟まずにひたすら受信するため、オーバーランが
%              起きにくく取りこぼしが少ない。
%      [第2段] キャプチャ済みIQのオフライン復号
%              WLAN Toolbox でパケットを検出・復号し、
%                1) Beacon/Probe Response から SSID と 送信元アドレス(BSSID)
%                   の対応を学習
%                2) 復号できた全パケットについて、プリアンブル (Non-HTは
%                   L-LTF、VHTは VHT-LTF) から CSI (チャネル応答 H(k)) を算出
%                3) targetSSID に対応する BSSID のパケットのCSIだけを抽出
%              して .mat 保存する。
%
%    ※ 第2段は受信と独立しているため、条件を変えて復号だけやり直すことも
%      できます (saveRawIQ を true にすると生IQも保存されます)。
%
%  対応フォーマット:
%    Non-HT (802.11a/g レガシー) と VHT (802.11ac / Wi-Fi 5, SU-VHTのみ) に
%    対応しています。HT (802.11n / Wi-Fi 4) と HE (802.11ax / Wi-Fi 6) は
%    フォーマット検出のみ行い、CSI 記録の対象からは除外されます。
%
%    VHT対応の制約 (いずれもUSRP B205 mini-iが受信アンテナ1本=SISOのため):
%      - NSTS(空間ストリーム数) = 1 のパケットのみ復号します。
%        マルチストリーム(NSTS>=2)は1本アンテナでは原理的に復号できません。
%      - 20MHz動作のVHTパケットのみ対象です。40/80/160MHzで送信された
%        パケットは本機の受信帯域(20MHz)では正しく復号できません。
%      - SU-VHT (GroupID = 0 または 63) のみ対応。MU-MIMOは非対応です。
%      - STBC を使用しているパケットは非対応としてスキップします。
%
%    VHT-SIG-A/VHT-SIG-B のビットフィールド解釈は WLAN Toolbox に専用関数が
%    無いため、IEEE Std 802.11ac-2013 (Table 22-12 等) に基づき本スクリプト
%    内で実装しています。特に VHT-SIG-B の Length フィールドの解釈
%    (20MHzでは17bit・4オクテット単位と想定) は実機未検証のため、VHTだけ
%    復号できない場合に最初に疑うべき箇所です。
%
%  必要環境:
%    - MATLAB / Communications Toolbox / WLAN Toolbox
%    - Communications Toolbox Support Package for USRP Radio
%    - USB 3.0 接続された USRP B205 mini-i
%
%  出力ファイル:
%    <保存先>/<yyyymmddHHMM>_<SSID>_CSI.mat
%
%    [主データ] ResultCSI.m がそのまま読める形式 (calculateCSI.m と同じ変数名)
%      csi               … [パケット数 x サブキャリア数] complex の行列
%      subcarrierIndices … [1 x サブキャリア数] 使用サブキャリア番号 k
%      packetStartIndex  … [パケット数 x 1] 各パケットのサンプル位置
%      csiMeta           … 処理条件・復号統計 (sampleRate 等を含む)
%      ※ Non-HT は 52 本、VHT(20MHz) は 56 本とサブキャリア数が異なるため
%         1 つの行列には混在させられない。パケット数が多い方を主データとし、
%         どちらを採用したかは csiMeta.primaryFormat に記録する。
%
%    [フォーマット別] 両方が必要な場合はこちらを使う
%      csiNonHT / timeSecNonHT / frameTypeNonHT / fcsNonHT
%      csiVHT   / timeSecVHT   / frameTypeVHT   / fcsVHT
%      subcarrierIndicesNonHT … [1 x 52]   Non-HT の使用サブキャリア番号
%      subcarrierIndicesVHT20 … [1 x 56]   VHT-20MHz の使用サブキャリア番号
%
%    [補助]
%      timeSec      … [パケット数 x 1] キャプチャ開始からの相対時刻 [s]
%      frameType    … [パケット数 x 1] 各パケットのフレーム種別
%      phyFormat    … [パケット数 x 1] 'Non-HT' または 'VHT'
%      fcsVerified  … [パケット数 x 1] false は FCS 未検証 (MAC ヘッダのみ
%                      から送信元を判定したもの)。CSI 自体には影響しない。
%      targetSSIDOut / targetBSSIDOut … 指定SSIDと学習されたBSSID
%      seenNetworks … 検出できた全 BSSID/SSID の一覧 (診断用)
%
%  注意:
%    電波の受信・記録は、利用地域の電波法および関連法令を遵守し、
%    自身が管理する機器・許可された環境でのみ実施してください。
% =========================================================================

%% ------------------------------------------------------------------------
%  0. 直前の実行で残っている USRP オブジェクトの解放
%  ------------------------------------------------------------------------
% スクリプトの変数はベースワークスペースに残るため、前回がエラーで終了して
% いると受信機オブジェクトが USRP を掴んだままになり、次回実行時に
% "radio is busy" となる。clear より先に明示的に release する。
if exist('rx', 'var')
    try
        release(rx);
    catch
    end
end
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

% --- 復号パラメータ ------------------------------------------------------
pktDetThreshold = 0.5;          % wlanPacketDetect のしきい値 (0〜1)
                                 % 検出数が少なすぎる場合は下げる (例 0.3)
saveRawIQ       = false;        % true にすると生IQも別ファイルに保存する
                                 % (2秒/20MHz で約 320MB になるので注意)
verboseErrors   = false;        % true にすると復号エラーを毎回表示する
                                 % (通常は最後に集計のみ表示)

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

timestamp  = datestr(now, 'yyyymmddHHMM');
ssidSafe   = regexprep(targetSSID, '[^A-Za-z0-9_-]', '_');
outMatFile = fullfile(usbSavePath, [timestamp '_' ssidSafe '_CSI.mat']);
rawIQFile  = fullfile(usbSavePath, [timestamp '_raw.mat']);

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
%  4. 第1段: 生IQのキャプチャ (復号は行わない)
%  ------------------------------------------------------------------------
totalSamplesTarget = round(captureDuration * sampleRate);
iqBuffer = complex(zeros(totalSamplesTarget + samplesPerFrame, 1));
totalSamplesCaptured = 0;
overrunCount = 0;

fprintf('\n[第1段] IQ キャプチャを開始します...\n');
captureTic = tic;

% エラーや中断が起きても USRP を必ず解放する
% (スクリプトの onCleanup はベースワークスペースに残り発火しないため、
%  try/catch で明示的に解放する)
try
    while totalSamplesCaptured < totalSamplesTarget
        [iqData, dataLen, overrun] = rx();
        if dataLen == 0
            continue;
        end
        if overrun
            overrunCount = overrunCount + 1;
        end
        iqBuffer(totalSamplesCaptured + (1:dataLen)) = iqData(1:dataLen);
        totalSamplesCaptured = totalSamplesCaptured + dataLen;
    end
catch captureErr
    try
        release(rx);
    catch
    end
    rethrow(captureErr);
end

elapsedCapture = toc(captureTic);
release(rx);

iq = iqBuffer(1:totalSamplesCaptured);
clear iqBuffer;

fprintf('  キャプチャ完了: %d サンプル, %.2f s, オーバーラン %d 回\n', ...
    totalSamplesCaptured, elapsedCapture, overrunCount);

if saveRawIQ
    meta = struct('centerFrequency', centerFrequency, 'sampleRate', sampleRate, ...
        'gain', gain, 'captureDatetime', timestamp);       %#ok<NASGU>
    iqSingle = single(iq);                                  %#ok<NASGU>
    save(rawIQFile, 'iqSingle', 'meta', '-v7.3');
    fprintf('  生IQを保存しました: %s\n', rawIQFile);
    clear iqSingle;
end

%% ------------------------------------------------------------------------
%  5. 第2段: オフライン復号 (パケット検出 → CSI 推定 → MAC 復号)
%  ------------------------------------------------------------------------
subcarrierIndicesNonHT = [-26:-1, 1:26];    % Non-HT CBW20 (52本)
subcarrierIndicesVHT20 = [-28:-1, 1:28];    % VHT CBW20 (56本)

pktLog = struct('timeSec', {}, 'bssid', {}, 'frameType', {}, 'ssid', {}, ...
    'phyFormat', {}, 'fcsVerified', {}, 'csi', {});
bssidToSSID = containers.Map('KeyType', 'char', 'ValueType', 'char');

% 復号統計 (診断用)
stats = struct('detected', 0, 'timingSkip', 0, 'nonHT', 0, 'vht', 0, ...
    'ht', 0, 'other', 0, 'vhtUnsupported', 0, 'decodeOK', 0, ...
    'noAddr2', 0, 'errors', 0);
errMsgs       = containers.Map('KeyType', 'char', 'ValueType', 'double');
vhtRejects    = containers.Map('KeyType', 'char', 'ValueType', 'double');
vhtBWCounts   = containers.Map('KeyType', 'double', 'ValueType', 'double');
vhtNSTSCounts = containers.Map('KeyType', 'double', 'ValueType', 'double');
vhtReservedOK = 0;   % VHT-SIG-A の予約ビットが仕様どおりだった件数
vhtDetailShown = 0;  % 内訳を表示した VHT パケット数
vhtDetailMax   = 5;  % 内訳を表示する最大件数

minPreambleLen = 560;   % L-STF..L-SIG + 2シンボル (フォーマット検出まで)
searchOffset = 0;

fprintf('\n[第2段] オフライン復号を開始します...\n');
decodeTic = tic;

% 検出は窓に区切って行う (毎回バッファ全体を走査すると非常に遅くなるため)。
% 窓の境界にまたがるプリアンブルを取りこぼさないよう少し重ねて進める。
detectWindow  = 400000;   % 20 ms 分
detectOverlap = 512;      % プリアンブル検出に必要な重なり

while searchOffset + minPreambleLen <= numel(iq)
    % --- パケット検出 (窓内の相対位置が返る) ---
    winEnd    = min(numel(iq), searchOffset + detectWindow);
    relOffset = wlanPacketDetect(iq(searchOffset+1 : winEnd), chanBW, 0, pktDetThreshold);
    if isempty(relOffset)
        if winEnd >= numel(iq)
            break;   % 末尾まで探索済み
        end
        searchOffset = winEnd - detectOverlap;
        continue;
    end
    pktOffset = searchOffset + relOffset;   % 0-based の絶対位置

    if numel(iq) - pktOffset < minPreambleLen
        break;   % 末尾が足りない
    end

    stats.detected = stats.detected + 1;

    % 次回の検索開始位置 (最低でも L-STF 分は進める。成功時は後で上書き)
    nextSearch = pktOffset + 160;

    try
        % --- 粗CFO推定 (L-STF) ---
        coarseCFO = wlanCoarseCFOEstimate(iq(pktOffset + (1:160)), chanBW);

        % --- 精タイミング推定 (粗CFO補正した Non-HT プリアンブル) ---
        nonHTPre  = applyCFO(iq(pktOffset + (1:400)), sampleRate, -coarseCFO);
        symOffset = wlanSymbolTimingEstimate(nonHTPre, chanBW);

        % wlanSymbolTimingEstimate は負値を返し得る。真の先頭が
        % バッファ範囲外になる場合は誤検出として読み捨てる。
        pktStart = pktOffset + symOffset;    % 0-based
        if pktStart < 0 || numel(iq) - pktStart < minPreambleLen
            stats.timingSkip = stats.timingSkip + 1;
            searchOffset = nextSearch;
            continue;
        end

        % --- 精CFO推定・補正 (L-LTF) ---
        pkt     = applyCFO(iq(pktStart+1:end), sampleRate, -coarseCFO);
        fineCFO = wlanFineCFOEstimate(pkt(161:320), chanBW);
        pkt     = applyCFO(pkt, sampleRate, -fineCFO);

        % --- L-LTF を復調してからチャネル推定・ノイズ推定 ---
        %     (wlanLLTFChannelEstimate は時間領域波形ではなく復調済み
        %      シンボルを受け取る点に注意)
        lltfDemod   = wlanLLTFDemodulate(pkt(161:320), chanBW);
        lltfChanEst = wlanLLTFChannelEstimate(lltfDemod, chanBW);
        noiseEst    = wlanLLTFNoiseEstimate(lltfDemod);

        % --- フォーマット検出 (L-SIG + 後続2シンボル) ---
        %     内部の L-SIG チェック失敗警告は独自サマリで集計するため抑止する
        wState = warning('off', 'all');
        fmt = wlanFormatDetect(pkt(321:560), lltfChanEst, noiseEst, chanBW);
        warning(wState);
        fmtStr = upper(string(fmt));

        switch true
            case fmtStr == "NON-HT"
                stats.nonHT = stats.nonHT + 1;
                [st, consumed, entry] = processNonHT(pkt, lltfChanEst, noiseEst, chanBW);

            case fmtStr == "VHT"
                stats.vht = stats.vht + 1;
                [st, consumed, entry, vhtInfo] = processVHT(pkt, lltfChanEst, noiseEst, chanBW);

                if vhtInfo.bwMHz > 0
                    if isKey(vhtBWCounts, vhtInfo.bwMHz)
                        vhtBWCounts(vhtInfo.bwMHz) = vhtBWCounts(vhtInfo.bwMHz) + 1;
                    else
                        vhtBWCounts(vhtInfo.bwMHz) = 1;
                    end
                end
                if vhtInfo.nsts > 0
                    if isKey(vhtNSTSCounts, vhtInfo.nsts)
                        vhtNSTSCounts(vhtInfo.nsts) = vhtNSTSCounts(vhtInfo.nsts) + 1;
                    else
                        vhtNSTSCounts(vhtInfo.nsts) = 1;
                    end
                end
                if vhtInfo.reservedOK
                    vhtReservedOK = vhtReservedOK + 1;
                end
                % 最初の数件だけ内訳を表示 (パケット長算出が妥当か確認するため)
                if vhtDetailShown < vhtDetailMax && vhtInfo.nsts == 1 && vhtInfo.bwMHz == 20
                    vhtDetailShown = vhtDetailShown + 1;
                    fprintf(['    <VHT#%d> MCS=%d GI=%s 符号化=%s | L-SIG長=%d NSYM=%d\n', ...
                             '            APEP: L-SIG由来=%d SIG-B由来=%d 採用=%d | ', ...
                             'A-MPDU分解=%s MPDU数=%d\n'], ...
                        vhtDetailShown, vhtInfo.mcs, ternary(vhtInfo.gi==1,'Short','Long'), ...
                        ternary(vhtInfo.coding==1,'LDPC','BCC'), vhtInfo.lsigLen, vhtInfo.nsym, ...
                        vhtInfo.apepFromLSIG, vhtInfo.apepFromSIGB, vhtInfo.apepUsed, ...
                        vhtInfo.deagStatus, vhtInfo.mpduCount);
                end
                if ~isempty(vhtInfo.reason)
                    stats.vhtUnsupported = stats.vhtUnsupported + 1;
                    if isKey(vhtRejects, vhtInfo.reason)
                        vhtRejects(vhtInfo.reason) = vhtRejects(vhtInfo.reason) + 1;
                    else
                        vhtRejects(vhtInfo.reason) = 1;
                    end
                end

            case startsWith(fmtStr, "HT")
                stats.ht = stats.ht + 1;
                st = 0; consumed = 320; entry = [];

            otherwise
                stats.other = stats.other + 1;
                st = 0; consumed = 320; entry = [];
        end

        if st < 0
            % パケット後半がキャプチャ範囲外 (末尾で切れている)
            searchOffset = nextSearch;
            continue;
        end

        if st > 0
            stats.decodeOK = stats.decodeOK + 1;

            if isempty(entry)
                % ACK/CTS など Address2 を持たないフレーム (BSSID 判定に使えない)
                stats.noAddr2 = stats.noAddr2 + 1;
            else
                entry.timeSec = pktStart / sampleRate;
                pktLog(end+1) = entry; %#ok<SAGROW>

                if any(strcmpi(entry.frameType, {'Beacon', 'Probe Response'})) && ~isempty(entry.ssid)
                    bssidToSSID(entry.bssid) = entry.ssid;
                end

                fprintf('  [%7.4fs] %-6s %-16s BSSID=%s%s\n', entry.timeSec, ...
                    entry.phyFormat, entry.frameType, entry.bssid, ...
                    ternary(~isempty(entry.ssid), sprintf('  SSID="%s"', entry.ssid), ''));
            end
        end

        searchOffset = pktStart + max(consumed, 160);

    catch ME
        stats.errors = stats.errors + 1;
        key = ME.message;
        if isKey(errMsgs, key)
            errMsgs(key) = errMsgs(key) + 1;
        else
            errMsgs(key) = 1;
        end
        if verboseErrors
            warning('captureIQ_v2:decodeError', 'パケット処理中にエラー: %s', ME.message);
        end
        searchOffset = nextSearch;
    end
end

elapsedDecode = toc(decodeTic);

%% ------------------------------------------------------------------------
%  6. 復号結果のサマリ表示
%  ------------------------------------------------------------------------
fprintf('\n[復号サマリ] (所要 %.2f s)\n', elapsedDecode);
fprintf('  パケット検出数            : %d\n', stats.detected);
fprintf('    タイミング不正でスキップ: %d\n', stats.timingSkip);
fprintf('    Non-HT として検出       : %d\n', stats.nonHT);
fprintf('    VHT    として検出       : %d  (うち非対応構成 %d)\n', stats.vht, stats.vhtUnsupported);
fprintf('    HT     として検出       : %d  (非対応)\n', stats.ht);
fprintf('    その他フォーマット      : %d  (非対応)\n', stats.other);
if stats.vht > 0
    fprintf('  [VHT 詳細]\n');
    if vhtBWCounts.Count > 0
        bwKeys = cell2mat(keys(vhtBWCounts));
        fprintf('    送信帯域幅の内訳: ');
        for k = 1:numel(bwKeys)
            fprintf('%dMHz=%d回  ', bwKeys(k), vhtBWCounts(bwKeys(k)));
        end
        fprintf('\n');
    end
    if vhtNSTSCounts.Count > 0
        nKeys = cell2mat(keys(vhtNSTSCounts));
        fprintf('    空間ストリーム数: ');
        for k = 1:numel(nKeys)
            fprintf('NSTS=%d が %d回  ', nKeys(k), vhtNSTSCounts(nKeys(k)));
        end
        fprintf('\n');
    end
    if vhtRejects.Count > 0
        rKeys = keys(vhtRejects);
        fprintf('    非対応の理由    : ');
        for k = 1:numel(rKeys)
            fprintf('%s=%d回  ', rKeys{k}, vhtRejects(rKeys{k}));
        end
        fprintf('\n');
    end
    % VHT-SIG-A のビット解釈が正しいかの自己判定
    fprintf('    SIG-A予約ビット検証: %d/%d 個が仕様どおり', vhtReservedOK, stats.vht);
    if vhtReservedOK < stats.vht * 0.5
        fprintf('  <-- 大半が不一致。ビット解釈がズレている可能性あり\n');
    else
        fprintf('  (ビット解釈は妥当)\n');
    end
end
fprintf('  MAC まで復号成功          : %d\n', stats.decodeOK);
fprintf('    うち Address2 無し(ACK/CTS等、BSSID判定不可): %d\n', stats.noAddr2);
fprintf('  復号エラー                : %d\n', stats.errors);

if stats.errors > 0
    fprintf('  エラー内訳 (上位):\n');
    ks = keys(errMsgs);
    vs = cell2mat(values(errMsgs));
    [~, sortIdx] = sort(vs, 'descend');
    for k = 1:min(5, numel(ks))
        fprintf('    %4d回: %s\n', vs(sortIdx(k)), ks{sortIdx(k)});
    end
end

%% ------------------------------------------------------------------------
%  7. 対象 SSID の BSSID を特定し、CSI を抽出
%  ------------------------------------------------------------------------
seenNetworks = struct('bssid', {}, 'ssid', {});
bssidKeys = keys(bssidToSSID);
for k = 1:numel(bssidKeys)
    seenNetworks(end+1) = struct('bssid', bssidKeys{k}, ...
        'ssid', bssidToSSID(bssidKeys{k})); %#ok<SAGROW>
end

fprintf('\n検出できたネットワーク一覧:\n');
if isempty(seenNetworks)
    fprintf('  (なし)\n');
else
    for k = 1:numel(seenNetworks)
        fprintf('  BSSID=%s  SSID="%s"\n', seenNetworks(k).bssid, seenNetworks(k).ssid);
    end
end

targetBSSID = '';
for k = 1:numel(seenNetworks)
    if strcmp(seenNetworks(k).ssid, targetSSID)
        targetBSSID = seenNetworks(k).bssid;
        break;
    end
end

matched = pktLog([]);   % 空の構造体配列

if isempty(targetBSSID)
    warning('captureIQ_v2:ssidNotFound', ...
        ['指定した SSID "%s" の Beacon/Probe Response を検出できませんでした。\n', ...
         'captureDuration を長くする、pktDetThreshold を下げる (例 0.3)、', ...
         'SSID の綴り・電波状況を確認する、などをお試しください。'], targetSSID);
else
    isMatch = strcmp({pktLog.bssid}, targetBSSID);
    matched = pktLog(isMatch);
    fprintf('\n対象 SSID "%s" (BSSID=%s) のパケット数: %d\n', ...
        targetSSID, targetBSSID, numel(matched));
end

% --- フォーマット別に [パケット数 x サブキャリア数] の行列へまとめる ---
% Non-HT は 52 本、VHT(20MHz) は 56 本とサブキャリア数が異なるため、
% 1 つの行列には混ぜられない。フォーマットごとに分けて保存する。
if isempty(matched)
    allFormats   = {};
    allTimeSec   = [];
    allFrameType = {};
    allFcs       = logical([]);
else
    allFormats   = {matched.phyFormat}.';
    allTimeSec   = [matched.timeSec].';
    allFrameType = {matched.frameType}.';
    allFcs       = logical([matched.fcsVerified]).';
end

isNonHT = strcmpi(allFormats, 'Non-HT');
isVHT   = strcmpi(allFormats, 'VHT');

csiNonHT       = stackCSI(matched(isNonHT));
timeSecNonHT   = allTimeSec(isNonHT);
frameTypeNonHT = allFrameType(isNonHT);
fcsNonHT       = allFcs(isNonHT);

csiVHT       = stackCSI(matched(isVHT));
timeSecVHT   = allTimeSec(isVHT);
frameTypeVHT = allFrameType(isVHT);
fcsVHT       = allFcs(isVHT);

if ~isempty(matched)
    fprintf('  内訳: Non-HT=%d, VHT=%d\n', size(csiNonHT, 1), size(csiVHT, 1));
    nUnverified = sum(~allFcs);
    if nUnverified > 0
        fprintf(['  ※うち %d 件は FCS 未検証 (ペイロードにビット誤りがあり、\n', ...
                 '    MAC ヘッダのみから送信元を判定したもの)。CSI 自体は\n', ...
                 '    プリアンブルから算出しており影響を受けません。\n'], nUnverified);
    end
end

% --- ResultCSI.m 互換の「主」データを決める ---
% ResultCSI.m は csi (行列) / subcarrierIndices / packetStartIndex /
% csiMeta という変数名を前提とするため、パケット数が多い方を主として
% その名前でも保存する。
if size(csiVHT, 1) >= size(csiNonHT, 1) && ~isempty(csiVHT)
    csi               = csiVHT;
    subcarrierIndices = subcarrierIndicesVHT20;
    timeSec           = timeSecVHT;
    frameType         = frameTypeVHT;
    fcsVerified       = fcsVHT;
    primaryFormat     = 'VHT';
else
    csi               = csiNonHT;
    subcarrierIndices = subcarrierIndicesNonHT;
    timeSec           = timeSecNonHT;
    frameType         = frameTypeNonHT;
    fcsVerified       = fcsNonHT;
    primaryFormat     = 'Non-HT';
end
phyFormat = repmat({primaryFormat}, size(csi, 1), 1);

% ResultCSI.m は packetStartIndex と sampleRate から時間軸を作る
packetStartIndex = round(timeSec(:) * sampleRate);

if ~isempty(csi)
    fprintf('  主データ(変数 csi): %s フォーマット, %d パケット x %d サブキャリア\n', ...
        primaryFormat, size(csi, 1), size(csi, 2));
end

%% ------------------------------------------------------------------------
%  8. 保存 (.mat)
%  ------------------------------------------------------------------------
% 変数名は calculateCSI.m / ResultCSI.m と揃えて csiMeta とする
csiMeta = struct();
csiMeta.description      = 'CSI filtered by target SSID (Non-HT and SU-VHT/NSTS=1/20MHz)';
csiMeta.primaryFormat    = primaryFormat;   % 変数 csi がどちらの形式か
csiMeta.centerFrequency  = centerFrequency;
csiMeta.sampleRate       = sampleRate;
csiMeta.gain             = gain;
csiMeta.captureDuration  = captureDuration;
csiMeta.wifiChannel      = 36;
csiMeta.platform         = usrpPlatform;
csiMeta.serialNum        = usrpSerialNum;
csiMeta.pktDetThreshold  = pktDetThreshold;
csiMeta.overrunCount     = overrunCount;
csiMeta.decodeStats      = stats;
csiMeta.captureDatetime  = timestamp;
csiMeta.matlabVersion    = version;

targetSSIDOut  = targetSSID;  %#ok<NASGU>
targetBSSIDOut = targetBSSID; %#ok<NASGU>

save(outMatFile, ...
    'csi', 'subcarrierIndices', 'packetStartIndex', 'csiMeta', ...
    'phyFormat', 'fcsVerified', 'timeSec', 'frameType', ...
    'csiNonHT', 'timeSecNonHT', 'frameTypeNonHT', 'fcsNonHT', ...
    'csiVHT', 'timeSecVHT', 'frameTypeVHT', 'fcsVHT', ...
    'subcarrierIndicesNonHT', 'subcarrierIndicesVHT20', ...
    'targetSSIDOut', 'targetBSSIDOut', 'seenNetworks', '-v7.3');

fprintf('\n結果を保存しました: %s\n', outMatFile);
fprintf('すべての処理が完了しました。\n');

%% ------------------------------------------------------------------------
%  ローカル関数
%  ------------------------------------------------------------------------
function M = stackCSI(entries)
    % パケットごとの CSI 行ベクトルを [パケット数 x サブキャリア数] の行列に積む。
    % サブキャリア数が揃わないものが混ざっていた場合は最頻の長さに合わせる。
    if isempty(entries)
        M = [];
        return;
    end
    lens = cellfun(@numel, {entries.csi});
    n = mode(lens);
    keep = (lens == n);
    M = cat(1, entries(keep).csi);
end

function y = applyCFO(x, fs, cfoHz)
    % x に周波数オフセット cfoHz [Hz] 分の位相回転を与える (補正には -cfo を渡す)
    n = (0:numel(x)-1).';
    y = x .* exp(1j * 2 * pi * cfoHz * n / fs);
end

function [status, consumed, entry] = processNonHT(pkt, chanEst, noiseEst, chanBW)
    % Non-HT (802.11a/g) パケットを復号する。
    %   status: 1=成功(entryあり), 0=失敗/読み捨て, -1=サンプル不足
    %   consumed: pkt 先頭から消費したサンプル数
    entry = [];
    consumed = 400;   % L-STF..L-SIG

    % --- L-SIG 復号 (レート・PSDU長を取得) ---
    [recLSIGBits, lsigFail] = wlanLSIGRecover(pkt(321:400), chanEst, noiseEst, chanBW);
    if lsigFail
        status = 0;
        return;
    end

    [mcs, psduLen] = decodeLSIGBits(recLSIGBits);
    if isempty(mcs) || psduLen <= 0 || psduLen > 4095
        status = 0;
        return;
    end

    try
        cfgNonHT = wlanNonHTConfig('MCS', mcs, 'PSDULength', psduLen, ...
            'ChannelBandwidth', chanBW);
        ind = wlanFieldIndices(cfgNonHT);
    catch
        status = 0;
        return;
    end

    if numel(pkt) < ind.NonHTData(2)
        status = -1;
        return;   % データ部がキャプチャ範囲外
    end
    consumed = double(ind.NonHTData(2));

    rxPSDU = wlanNonHTDataRecover(pkt(ind.NonHTData(1):ind.NonHTData(2)), ...
        chanEst, noiseEst, cfgNonHT);

    % 未対応サブタイプの警告は頻出するためここでは抑止する (サマリで集計)
    wState = warning('off', 'all');
    [cfgMAC, payload, mpduStatus] = wlanMPDUDecode(rxPSDU, cfgNonHT, 'DataFormat', 'bits');
    warning(wState);

    if ~strcmpi(string(mpduStatus), "Success")
        status = 0;
        return;
    end

    entry = buildEntry(cfgMAC, payload, 'Non-HT', chanEst);
    status = 1;
end

function [status, consumed, entry, info] = processVHT(pkt, lltfChanEst, noiseEst, chanBW)
    % SU-VHT (802.11ac, NSTS=1, 20MHz) パケットを復号する。
    %   status: 1=成功, 0=失敗/非対応, -1=サンプル不足
    %   info:   診断用 (棄却理由・観測された帯域幅/NSTS/MCS)
    entry = [];
    consumed = 560;   % L-STF..VHT-SIG-A
    info = struct('reason', '', 'bwMHz', 0, 'nsts', 0, 'mcs', 0, ...
        'reservedOK', false, 'gi', 0, 'coding', 0, 'lsigLen', 0, 'nsym', 0, ...
        'apepFromLSIG', 0, 'apepFromSIGB', 0, 'apepUsed', 0, ...
        'mpduCount', 0, 'deagStatus', '');

    % --- VHT-SIG-A 復号 (2シンボル) ---
    [sigaBits, sigaFail] = wlanVHTSIGARecover(pkt(401:560), lltfChanEst, noiseEst, chanBW);
    if sigaFail
        info.reason = 'SIG-A CRC失敗';
        status = 0;
        return;
    end

    vhtA = parseVHTSIGABits(sigaBits);
    info.bwMHz      = vhtA.bwMHz;
    info.nsts       = vhtA.nsts;
    info.mcs        = vhtA.mcs;
    info.reservedOK = vhtA.reservedOK;
    info.gi         = vhtA.gi;
    info.coding     = vhtA.coding;

    % 以下はいずれも SISO(受信1本)/20MHz 受信では扱えない構成
    if ~vhtA.isValid
        info.reason = 'SIG-A解釈不能';
        status = 0; return;
    elseif ~vhtA.is20MHz
        info.reason = sprintf('帯域幅%dMHz(20MHz受信では復号不可)', vhtA.bwMHz);
        status = 0; return;
    elseif ~vhtA.isSU
        info.reason = 'MU-MIMO';
        status = 0; return;
    elseif vhtA.nsts ~= 1
        info.reason = sprintf('NSTS=%d(アンテナ1本では復号不可)', vhtA.nsts);
        status = 0; return;
    elseif vhtA.stbc
        info.reason = 'STBC';
        status = 0; return;
    end

    codingStr = ternary(vhtA.coding == 1, 'LDPC', 'BCC');
    giStr     = ternary(vhtA.gi == 1, 'Short', 'Long');

    % --- VHT-STF/LTF/SIG-B の位置を取得 ---
    %     これらの位置は帯域幅と NSTS だけで決まり APEPLength に依存しないため、
    %     暫定の APEPLength を持つコンフィグから求めてよい。
    try
        cfgProbe = wlanVHTConfig('ChannelBandwidth', chanBW, ...
            'NumTransmitAntennas', 1, 'NumSpaceTimeStreams', 1, ...
            'MCS', vhtA.mcs, 'APEPLength', 1024, ...
            'ChannelCoding', codingStr, 'GuardInterval', giStr);
        indProbe = wlanFieldIndices(cfgProbe);
    catch
        info.reason = 'VHTコンフィグ生成失敗';
        status = 0;
        return;
    end

    if numel(pkt) < indProbe.VHTSIGB(2)
        status = -1;
        return;
    end

    % --- VHT-LTF を復調してチャネル推定 (= CSI) ---
    demodVHTLTF = wlanVHTLTFDemodulate(pkt(indProbe.VHTLTF(1):indProbe.VHTLTF(2)), chanBW, 1);
    vhtChanEst  = wlanVHTLTFChannelEstimate(demodVHTLTF, chanBW, 1);

    % --- パケット長の決定 ---
    % MathWorks の helperVHTConfigRecover と同様、L-SIG と VHT-SIG-A から
    % 求めるのを主とする。VHT-SIG-B の Length は参考値として併記する。
    [lsigBits, lsigFail] = wlanLSIGRecover(pkt(321:400), lltfChanEst, noiseEst, chanBW);
    if ~lsigFail
        [~, info.lsigLen] = decodeLSIGBits(lsigBits);
    end

    % VHT PPDU の受信時間 [us] (IEEE 802.11ac の L-SIG LENGTH との関係)
    %   RXTIME = 4*ceil((L_LENGTH+3)/3) + 20
    ndbps  = vhtNDBPS20MHz(vhtA.mcs);
    symLen = ternary(vhtA.gi == 1, 72, 80);   % Short GI: 3.6us, Long GI: 4us
    if info.lsigLen > 0 && ~isempty(ndbps)
        totalSamples    = (4 * ceil((info.lsigLen + 3) / 3) + 20) * 20;  % 20 samples/us
        dataSamples     = totalSamples - indProbe.VHTSIGB(2);   % プリアンブル分を除く
        info.nsym       = floor(dataSamples / symLen);
        if info.nsym > 0
            % NSYM = ceil((8*APEP + 22)/NDBPS) を満たす最大の APEP
            info.apepFromLSIG = floor((info.nsym * ndbps - 22) / 8);
        end
    end

    % --- VHT-SIG-B 復号 (参考・比較用) ---
    try
        sigbBits = wlanVHTSIGBRecover(pkt(indProbe.VHTSIGB(1):indProbe.VHTSIGB(2)), ...
            vhtChanEst, noiseEst, chanBW);
        sigbLen = decodeVHTSIGBLength(sigbBits);
        if ~isempty(sigbLen)
            info.apepFromSIGB = sigbLen;
        end
    catch
    end

    % L-SIG 由来を優先し、駄目なら VHT-SIG-B 由来を使う
    apepCandidates = [info.apepFromLSIG, info.apepFromSIGB];
    apepCandidates = apepCandidates(apepCandidates > 0);
    if isempty(apepCandidates)
        info.reason = 'パケット長を決定できず';
        status = 0;
        return;
    end

    cfgVHT = [];
    for c = 1:numel(apepCandidates)
        try
            cfgTry = wlanVHTConfig('ChannelBandwidth', chanBW, ...
                'NumTransmitAntennas', 1, 'NumSpaceTimeStreams', 1, ...
                'MCS', vhtA.mcs, 'APEPLength', apepCandidates(c), ...
                'ChannelCoding', codingStr, 'GuardInterval', giStr);
            indTry = wlanFieldIndices(cfgTry);
            if numel(pkt) >= indTry.VHTData(2)
                cfgVHT = cfgTry;
                ind = indTry;
                info.apepUsed = apepCandidates(c);
                break;
            end
        catch
            % この候補は不正。次の候補へ
        end
    end

    if isempty(cfgVHT)
        info.reason = sprintf('APEPLength不正/長さ不足(LSIG=%d,SIGB=%d)', ...
            info.apepFromLSIG, info.apepFromSIGB);
        status = -1;   % データがまだ揃っていない可能性もある
        return;
    end

    consumed = double(ind.VHTData(2));

    rxPSDU = wlanVHTDataRecover(pkt(ind.VHTData(1):ind.VHTData(2)), ...
        vhtChanEst, noiseEst, cfgVHT);

    % --- VHT の PSDU は A-MPDU なので分解してから MPDU を復号 ---
    [cfgMAC, payload, ok, deagInfo] = decodeVHTPSDU(rxPSDU, cfgVHT);
    info.mpduCount  = deagInfo.mpduCount;
    info.deagStatus = deagInfo.status;
    if ~ok
        info.reason = sprintf('MPDU復号失敗(分解=%s,MPDU数=%d)', ...
            deagInfo.status, deagInfo.mpduCount);
        status = 0;
        return;
    end

    entry = buildEntry(cfgMAC, payload, 'VHT', vhtChanEst);
    if ~isempty(entry)
        entry.fcsVerified = ~deagInfo.headerOnly;
    end
    status = 1;
end

function [cfgMAC, payload, ok, deagInfo] = decodeVHTPSDU(rxPSDU, cfgVHT)
    % VHT の PSDU (A-MPDU) を分解し、最初に復号できた MPDU を返す。
    cfgMAC = [];
    payload = [];
    ok = false;
    deagInfo = struct('status', 'N/A', 'mpduCount', 0, 'headerOnly', false);

    mpduList = {};
    try
        [mpduList, ~, deagStatus] = wlanAMPDUDeaggregate(rxPSDU, cfgVHT, 'DataFormat', 'bits');
        deagInfo.status = char(string(deagStatus));
    catch ME
        deagInfo.status = ['例外: ' ME.message];
    end
    deagInfo.mpduCount = numel(mpduList);

    % 全体の status が Success でなくても、取り出せた MPDU は個別に試す
    % (一部のサブフレームだけ壊れている場合でも残りは復号できるため)
    wState = warning('off', 'all');
    for m = 1:numel(mpduList)
        try
            [c, p, st] = wlanMPDUDecode(mpduList{m}, cfgVHT, 'DataFormat', 'bits');
            if strcmpi(string(st), "Success")
                warning(wState);
                cfgMAC = c; payload = p; ok = true;
                return;
            end
        catch
            % この MPDU は読み飛ばす
        end
    end
    warning(wState);

    % A-MPDU として分解できなかった場合は単一 MPDU として試す
    if isempty(mpduList)
        try
            wState = warning('off', 'all');
            [c, p, st] = wlanMPDUDecode(rxPSDU, cfgVHT, 'DataFormat', 'bits');
            warning(wState);
            if strcmpi(string(st), "Success")
                cfgMAC = c; payload = p; ok = true;
                return;
            end
        catch
        end
    end

    % --- フォールバック: FCS 検証に失敗しても MAC ヘッダから送信元を読む ---
    % FCS は MPDU 全体 (最大 1500B 超) を検証するため、末尾の 1 ビット誤りでも
    % 失敗する。一方 CSI の帰属に必要なのは先頭 24 バイトのヘッダだけであり、
    % ここが無傷である確率は高い。
    candidates = mpduList;
    if isempty(candidates)
        candidates = {rxPSDU};
    end
    for m = 1:numel(candidates)
        hdr = parseMACHeaderFromBits(candidates{m});
        if hdr.valid
            cfgMAC  = hdr;      % wlanMACFrameConfig ではなく自前の構造体
            payload = [];
            ok = true;
            deagInfo.headerOnly = true;
            return;
        end
    end
end

function hdr = parseMACHeaderFromBits(mpduBits)
    % MPDU のビット列から MAC ヘッダ (Frame Control / Address1-3) を直接読む。
    % FCS を検証しないため、ヘッダ内容が正しい保証は無い点に注意。
    hdr = struct('valid', false, 'FrameType', '', 'Address2', '', ...
        'ManagementConfig', [], 'headerOnly', true);

    if isempty(mpduBits)
        return;
    end
    oct = bitsToOctets(mpduBits);
    if numel(oct) < 24
        return;   % Address2 まで読めない
    end

    fc0     = oct(1);
    version = bitand(fc0, 3);
    type    = bitand(bitshift(fc0, -2), 3);
    subtype = bitand(bitshift(fc0, -4), 15);

    if version ~= 0
        return;   % プロトコルバージョンは 0 のはず。違えば復号が壊れている
    end

    % Address2 を持たないフレーム (ACK=1101, CTS=1100 の制御フレーム) は対象外
    if type == 1 && any(subtype == [12 13])
        return;
    end

    switch type
        case 0
            typeName = 'Management';
            if subtype == 8
                typeName = 'Beacon';
            elseif subtype == 5
                typeName = 'Probe Response';
            end
        case 1
            typeName = 'Control';
        case 2
            typeName = 'Data';
        otherwise
            return;   % type=3 は予約値
    end

    addr2 = oct(11:16);
    hdr.valid     = true;
    hdr.FrameType = typeName;
    hdr.Address2  = upper(sprintf('%02X', addr2));
end

function ndbps = vhtNDBPS20MHz(mcs)
    % VHT 20MHz / 1空間ストリーム の NDBPS (1 OFDM シンボルあたりのデータビット数)
    % NSD=52, NDBPS = 52 * NBPSCS * 符号化率
    table = [26 52 78 104 156 208 234 260 312];   % MCS0..MCS8
    if mcs >= 0 && mcs <= 8
        ndbps = table(mcs + 1);
    else
        ndbps = [];   % MCS9 は 20MHz/1SS では未定義
    end
end

function entry = buildEntry(cfgMAC, payload, phyFormat, chanEst)
    % 復号済み MAC 情報と CSI から記録用の構造体を作る。
    % Address2 (送信元アドレス) を持たないフレームは BSSID 判定に使えない
    % ため空を返す。
    entry = [];
    frameType = char(string(cfgMAC.FrameType));

    % ACK と CTS は Address1 のみを持ち Address2 が無い。
    % (この場合 cfgMAC.Address2 には既定値が残るため BSSID として使えない)
    if any(strcmpi(frameType, {'ACK', 'CTS'}))
        return;
    end

    ssidStr = '';
    if any(strcmpi(frameType, {'Beacon', 'Probe Response'}))
        ssidStr = extractSSID(cfgMAC, payload);
    end

    entry = struct( ...
        'timeSec',     0, ...            % 呼び出し側で設定
        'bssid',       char(string(cfgMAC.Address2)), ...
        'frameType',   frameType, ...
        'ssid',        ssidStr, ...
        'phyFormat',   phyFormat, ...
        'fcsVerified', true, ...         % VHT のヘッダのみ復号時は呼び出し側で false
        'csi',         chanEst(:).');
end

function ssidStr = extractSSID(cfgMAC, payload)
    % Beacon/Probe Response から SSID を取得する。
    % wlanMPDUDecode は管理フレームの内容を cfgMAC.ManagementConfig
    % (wlanMACManagementConfig) に格納するため、まずそこから読む。
    ssidStr = '';
    try
        mgmt = cfgMAC.ManagementConfig;
        if ~isempty(mgmt)
            s = char(string(mgmt.SSID));
            if ~isempty(s)
                ssidStr = s;
                return;
            end
        end
    catch
        % ManagementConfig を持たない/参照できない場合は下のフォールバックへ
    end

    % フォールバック: フレームボディを自前で解析する
    ssidStr = parseSSIDFromMgmtFrame(payload);
end

function [mcs, psduLen] = decodeLSIGBits(bits)
    % 802.11a/g L-SIG (24bit: RATE(4)+Reserved(1)+LENGTH(12)+Parity(1)+Tail(6))
    % を解釈し、Non-HT MCS インデックス (0..7) と PSDU長 [byte] を返す。
    % RATE の対応は IEEE Std 802.11-2016 Table 17-6 による。
    bits = double(bits(:)).';
    if numel(bits) < 17
        mcs = []; psduLen = 0;
        return;
    end

    rateKey = sprintf('%d', bits(1:4));
    rateMap = containers.Map( ...
        {'1101','1111','0101','0111','1001','1011','0001','0011'}, ...
        {0,      1,     2,     3,     4,     5,     6,     7});
    if ~isKey(rateMap, rateKey)
        mcs = []; psduLen = 0;
        return;
    end
    mcs = rateMap(rateKey);

    lenBits = bits(6:17);   % LENGTH は LSB 先頭
    psduLen = sum(lenBits .* 2.^(0:11));
end

function out = parseVHTSIGABits(bits)
    % VHT-SIG-A (48bit = SIG-A1(24) + SIG-A2(24)) を解釈する。
    % IEEE Std 802.11ac-2013, Table 22-12 に基づく。多ビットフィールドは
    % LSB 先頭で送信される。
    %   bits(1:2)   BW (00=20MHz)          bits(4)     STBC
    %   bits(5:10)  Group ID               bits(11:13) NSTS-1
    %   bits(25)    Short GI               bits(27)    Coding (0=BCC,1=LDPC)
    %   bits(29:32) SU VHT-MCS
    bits = double(bits(:)).';
    out = struct('isValid', false, 'is20MHz', false, 'isSU', false, ...
        'nsts', 0, 'stbc', false, 'groupId', 0, 'gi', 0, 'coding', 0, ...
        'mcs', 0, 'bwMHz', 0, 'reservedOK', false);

    if numel(bits) < 34
        return;
    end

    % 自己チェック: 仕様上 1 に固定されている予約ビット
    %   SIG-A1 B2  -> bits(3),  SIG-A1 B23 -> bits(24),  SIG-A2 B9 -> bits(34)
    % これが揃わない場合、本関数のビット位置の想定がズレている。
    out.reservedOK = (bits(3) == 1) && (bits(24) == 1) && (bits(34) == 1);

    bw       = bits(1:2);
    stbcBit  = bits(4);
    groupId  = bits(5:10);
    nstsBits = bits(11:13);

    bwTable     = [20 40 80 160];
    out.bwMHz   = bwTable(sum(bw .* 2.^(0:1)) + 1);
    out.is20MHz = all(bw == 0);
    % SU-VHT は Group ID = 0 または 63 (全ビット0 または 全ビット1)
    out.isSU    = all(groupId == 0) || all(groupId == 1);
    if all(nstsBits == 0)
        out.nsts = 1;   % 全ビット0ならビット順によらず NSTS=1 と確定できる
    else
        out.nsts = 1 + sum(nstsBits .* 2.^(0:2));
    end
    out.stbc    = stbcBit == 1;
    out.groupId = sum(groupId .* 2.^(0:5));
    out.gi      = bits(25);
    out.coding  = bits(27);
    out.mcs     = sum(bits(29:32) .* 2.^(0:3));
    out.isValid = true;
end

function apepLen = decodeVHTSIGBLength(bits)
    % VHT-SIG-B (20MHz, SU) の Length フィールド(17bit, LSB先頭)を解釈し、
    % APEP長 [byte] を返す。20MHz では 4 オクテット単位で符号化されるため
    % 4 倍する。※この換算は実機未検証で、VHT が復号できない場合に
    %   最初に見直すべき箇所。
    bits = double(bits(:)).';
    if numel(bits) < 17
        apepLen = [];
        return;
    end
    apepLen = sum(bits(1:17) .* 2.^(0:16)) * 4;
end

function ssidStr = parseSSIDFromMgmtFrame(payload)
    % Beacon/Probe Response のフレームボディから先頭の
    % Information Element (SSID, Element ID = 0) を抽出する。
    % 固定フィールド: Timestamp(8) + BeaconInterval(2) + CapabilityInfo(2) = 12byte
    ssidStr = '';
    if isempty(payload)
        return;
    end
    if iscell(payload)
        payload = payload{1};
    end
    payload = double(payload(:)).';
    if isempty(payload)
        return;
    end

    % wlanMPDUDecode に 'DataFormat','bits' を渡しているため payload は
    % ビット列で返る。オクテット列 (MACはオクテット内 LSB 先頭) に戻す。
    if max(payload) <= 1 && mod(numel(payload), 8) == 0
        payload = bitsToOctets(payload);
    end
    payload = uint8(payload);

    if numel(payload) < 14
        return;
    end
    ieOffset = 12;
    elemId  = payload(ieOffset + 1);
    elemLen = payload(ieOffset + 2);
    if elemId ~= 0
        return;   % 先頭 IE が SSID でない
    end
    if numel(payload) < ieOffset + 2 + elemLen || elemLen == 0
        return;
    end
    ssidStr = char(payload(ieOffset + 3 : ieOffset + 2 + elemLen));
end

function bytes = bitsToOctets(bits)
    % ビット列をオクテット列へ変換する (各オクテット内は LSB 先頭)
    bits = double(bits(:));
    n = floor(numel(bits) / 8) * 8;
    if n == 0
        bytes = [];
        return;
    end
    b = reshape(bits(1:n), 8, []);
    bytes = (2.^(0:7)) * b;   % 1 x (n/8)
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end

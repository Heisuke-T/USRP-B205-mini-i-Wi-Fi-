%% decodeIQ_VHT.m
% =========================================================================
%  [第2段] キャプチャ済み IQ をオフライン復号し、指定SSIDのCSIを記録する
% -------------------------------------------------------------------------
%  概要:
%    captureIQ.m が HDD に保存した生 IQ を読み込み、WLAN Toolbox で
%    パケットを検出・復号して、
%      1) Beacon/Probe Response から SSID と BSSID の対応を学習
%      2) 復号できた全パケットについて、プリアンブル (Non-HTは L-LTF、
%         HTは HT-LTF、VHTは VHT-LTF) から CSI (チャネル応答 H(k)) を算出
%      3) targetSSID に対応する BSSID のパケットのCSIだけを抽出
%    して .mat 保存する。
%
%    受信と独立しているため、条件 (targetSSID, pktDetThreshold 等) を
%    変えて同じ IQ を何度でも復号し直せる。
%
%  対応フォーマット:
%    Non-HT (802.11a/g レガシー) / HT-Mixed (802.11n / Wi-Fi 4) /
%    SU-VHT (802.11ac / Wi-Fi 5) に対応。
%    HT-Greenfield と HE (802.11ax / Wi-Fi 6) は未対応 (検出のみ)。
%
%    ※ファイル名の "_VHT" は「VHT (Wi-Fi 5) までを対象とする復号器」の意。
%      HE (802.11ax / Wi-Fi 6) は VHT とプリアンブル構造・SIG フィールドが
%      大きく異なるため、本ファイルを拡張せず decodeIQ_HE.m として別に
%      用意する方針。入力 (captureIQ.m の *_raw.mat) と出力 (*_CSI.mat) の
%      形式は共通とし、ResultCSI.m がどちらの出力も読めるようにする。
%
%    HT・VHT対応の制約 (いずれもUSRP B205 mini-iが受信アンテナ1本=SISOのため):
%      - 空間ストリーム数 = 1 のパケットのみ復号します。
%        マルチストリームは1本アンテナでは原理的に復号できません。
%        (HT: MCS0-7 のみ対応。MCS8以上は複数ストリームのため非対応)
%      - 20MHz動作のパケットのみ対象です。40/80/160MHzで送信された
%        パケットは本機の受信帯域(20MHz)では正しく復号できません。
%      - VHT は SU-VHT (GroupID = 0 または 63) のみ対応。MU-MIMO非対応。
%      - STBC を使用しているパケットは非対応としてスキップします。
%
%    HT-SIG / VHT-SIG-A / VHT-SIG-B のビットフィールド解釈は WLAN Toolbox に
%    専用の解釈関数が無いため、IEEE Std 802.11-2016 / 802.11ac-2013 の仕様に
%    基づき本スクリプト内で実装しています。
%
%  必要環境:
%    - MATLAB / Communications Toolbox / WLAN Toolbox
%    - captureIQ.m が出力した *_raw.mat
%
%  入力ファイル:
%    <hddInputPath>/<yyyymmddHHMM>_raw.mat   (captureIQ.m の出力)
%
%  出力ファイル (HDD と USB メモリの両方に同じ内容を保存):
%    <hddSavePath>/<yyyymmddHHMM>_<SSID>_CSI.mat
%    <usbSavePath>/<yyyymmddHHMM>_<SSID>_CSI.mat
%      ※ファイル名の日時はキャプチャ時刻 (復号時刻ではない)。元の生IQと
%        対応が取れるようにするため。
%
%    [主データ] ResultCSI.m がそのまま読める形式
%      csi               … [パケット数 x サブキャリア数] complex の行列
%      subcarrierIndices … [1 x サブキャリア数] 使用サブキャリア番号 k
%      packetStartIndex  … [パケット数 x 1] 各パケットのサンプル位置
%      csiMeta           … 処理条件・復号統計 (sampleRate 等を含む)
%      ※ Non-HT/HT は 52 本、VHT(20MHz) は 56 本とサブキャリア数が異なる
%         ため 1 つの行列には混在させられない。3種類のうちパケット数が
%         最多の形式を主データとし、どれを採用したかは
%         csiMeta.primaryFormat に記録する。
%
%    [フォーマット別] 複数形式が必要な場合はこちらを使う
%      csiNonHT / timeSecNonHT / frameTypeNonHT / fcsNonHT
%      csiHT    / timeSecHT    / frameTypeHT    / fcsHT
%      csiVHT   / timeSecVHT   / frameTypeVHT   / fcsVHT
%      subcarrierIndicesNonHT … [1 x 52]   Non-HT の使用サブキャリア番号
%      subcarrierIndicesHT20  … [1 x 52]   HT-20MHz の使用サブキャリア番号
%      subcarrierIndicesVHT20 … [1 x 56]   VHT-20MHz の使用サブキャリア番号
%
%    [補助]
%      timeSec      … [パケット数 x 1] キャプチャ開始からの相対時刻 [s]
%      frameType    … [パケット数 x 1] 各パケットのフレーム種別
%      phyFormat    … [パケット数 x 1] 'Non-HT' / 'HT' / 'VHT'
%      fcsVerified  … [パケット数 x 1] false は FCS 未検証 (MAC ヘッダのみ
%                      から送信元を判定したもの)。CSI 自体には影響しない。
%      targetSSIDOut / targetBSSIDOut … 指定SSIDと学習されたBSSID
%      seenNetworks … 検出できた全 BSSID/SSID の一覧 (診断用)
%
%    ※ 対象 SSID への帰属判定は、送信元(Address2)・宛先(Address1)の
%      どちらかが対象 BSSID と一致するかで行う (ダウンリンクは Address2、
%      アップリンクは Address1 が BSSID になるため)。
%
%  処理時間の目安:
%    実測でキャプチャ 1 秒あたり 300〜2400 秒 (電波の混雑度に強く依存)。
%    混雑した環境での 5 秒キャプチャで約 3 時間20分を要した実績がある。
% =========================================================================

clear; clc;

%% ------------------------------------------------------------------------
%  1. ユーザ設定パラメータ
%  ------------------------------------------------------------------------

% --- 入力元 (ホストPC に接続された HDD。captureIQ.m の保存先と揃える) ----
%     現在の環境: HDPC-UT (D:)
hddInputPath = 'D:\IQ_raw';

% --- 読み込む生IQファイル ------------------------------------------------
%     '' にすると hddInputPath 内で最も新しい *_raw.mat を自動選択する。
%     特定のファイルを指定したい場合はファイル名かフルパスを書く。
inputRawFile = '';

% --- 出力先 --------------------------------------------------------------
%     同じ内容を HDD と USB メモリの両方へ保存する。
%     HDD は HDPC-UT (D:)。
hddSavePath = 'D:\IQ_csi';

%     USB メモリのドライブレターをここに設定する (例: 'F:\IQ')。
%     '' のままにすると USB への保存はスキップし、HDD にのみ保存する。
%     ※ResultCSI.m の既定の読み込み先は 'D:\IQ' なので、USB を使わない
%       場合は ResultCSI.m 側の usbSavePath を 'D:\IQ_csi' に変更すること。
usbSavePath = '';

% --- 抽出したい Wi-Fi の SSID --------------------------------------------
targetSSID = 'OpenWrt-A';

% --- 復号パラメータ ------------------------------------------------------
chanBW          = 'CBW20';      % WLAN Toolbox のチャネル帯域幅指定
                                 % (キャプチャ時の 20MHz に対応)
pktDetThreshold = 0.5;          % wlanPacketDetect のしきい値 (0〜1)
                                 % 下げると弱いパケットも拾うが誤検出が増え、
                                 % 復号時間も伸びる
verboseErrors   = false;        % true にすると復号エラーを毎回表示する
                                 % (通常は最後に集計のみ表示)

%% ------------------------------------------------------------------------
%  2. 生IQファイルの読み込み
%  ------------------------------------------------------------------------
if isempty(inputRawFile)
    listing = dir(fullfile(hddInputPath, '*_raw.mat'));
    if isempty(listing)
        error('decodeIQ_VHT:noInputFile', ...
            ['入力ファイルが見つかりません: %s\\*_raw.mat\n', ...
             '先に captureIQ.m を実行するか、inputRawFile にパスを指定してください。'], ...
            hddInputPath);
    end
    [~, newest] = max([listing.datenum]);
    inputRawFile = fullfile(listing(newest).folder, listing(newest).name);
    fprintf('最新の生IQファイルを自動選択しました:\n  %s\n', inputRawFile);
elseif isempty(fileparts(inputRawFile))
    % ディレクトリ部分が無い = ファイル名だけの指定なので hddInputPath 配下とみなす
    inputRawFile = fullfile(hddInputPath, inputRawFile);
end

if ~isfile(inputRawFile)
    error('decodeIQ_VHT:inputNotFound', '入力ファイルが存在しません: %s', inputRawFile);
end

S = load(inputRawFile, 'iq', 'meta');
if ~isfield(S, 'iq') || ~isfield(S, 'meta')
    error('decodeIQ_VHT:badInputFile', ...
        ['入力ファイルに変数 iq / meta がありません: %s\n', ...
         'captureIQ.m が出力した *_raw.mat を指定してください。'], inputRawFile);
end

% captureIQ.m は complex double で保存するため通常は変換不要だが、
% 古い single 形式のファイルも読めるよう double() を通しておく
% (WLAN Toolbox の関数群は double を前提とする)
iq   = double(S.iq(:));
meta = S.meta;
clear S;

% 以降の処理・保存メタデータはキャプチャ時の条件を引き継ぐ
centerFrequency = meta.centerFrequency;
sampleRate      = meta.sampleRate;
gain            = meta.gain;
captureDuration = meta.captureDuration;
usrpPlatform    = meta.platform;
usrpSerialNum   = meta.serialNum;
overrunCount    = meta.overrunCount;
timestamp       = meta.captureDatetime;

fprintf('\n読み込んだ生IQ:\n');
fprintf('  ファイル        : %s\n', inputRawFile);
fprintf('  キャプチャ日時  : %s\n', timestamp);
fprintf('  サンプル数      : %d (%.3f s)\n', numel(iq), numel(iq) / sampleRate);
fprintf('  中心周波数      : %.4f GHz\n', centerFrequency / 1e9);
fprintf('  サンプルレート  : %.3f MSps\n', sampleRate / 1e6);
fprintf('  ゲイン          : %d dB\n', gain);
fprintf('  オーバーラン    : %d 回\n', overrunCount);
if overrunCount > 0
    fprintf('  ※キャプチャ時に取りこぼしがあります。時刻軸に不連続が含まれる\n');
    fprintf('    可能性があるため、CSI の時系列解釈には注意してください。\n');
end
fprintf('  対象 SSID       : %s\n', targetSSID);

%% ------------------------------------------------------------------------
%  3. 出力先の準備 (HDD と USB メモリの両方)
%  ------------------------------------------------------------------------
ssidSafe   = regexprep(targetSSID, '[^A-Za-z0-9_-]', '_');
outFileName = [timestamp '_' ssidSafe '_CSI.mat'];

outMatFiles = {};   % 実際に書き込めた出力先
outTargets  = { 'HDD', hddSavePath; 'USB', usbSavePath };
for k = 1:size(outTargets, 1)
    label = outTargets{k, 1};
    pth   = outTargets{k, 2};

    if isempty(pth)
        % 未設定 (例: USB メモリを使わない) のでこの出力先はスキップ
        continue;
    end

    if ~exist(pth, 'dir')
        fprintf('%s 保存先フォルダが存在しないため作成します: %s\n', label, pth);
        [ok, msg] = mkdir(pth);
        if ~ok
            warning('decodeIQ_VHT:mkdirFailed', ...
                '%s 保存先フォルダを作成できませんでした (%s): %s', label, pth, msg);
            continue;
        end
    end

    testFile = fullfile(pth, '.write_test.tmp');
    fidTest  = fopen(testFile, 'w');
    if fidTest == -1
        warning('decodeIQ_VHT:notWritable', ...
            '%s 保存先に書き込みできないためスキップします: %s', label, pth);
        continue;
    end
    fclose(fidTest);
    delete(testFile);

    outMatFiles{end+1} = fullfile(pth, outFileName); %#ok<SAGROW>
end

if isempty(outMatFiles)
    error('decodeIQ_VHT:noWritableOutput', ...
        ['書き込み可能な出力先がありません。\n', ...
         '  HDD: %s\n  USB: %s\n', ...
         'ドライブの接続とパスを確認してください。'], hddSavePath, usbSavePath);
end

fprintf('  保存先          :\n');
for k = 1:numel(outMatFiles)
    fprintf('    %s\n', outMatFiles{k});
end

%% ------------------------------------------------------------------------
%  4. パケット検出 → CSI 推定 → MAC 復号
%  ------------------------------------------------------------------------
subcarrierIndicesNonHT = [-26:-1, 1:26];    % Non-HT CBW20 (52本)
subcarrierIndicesHT20  = [-26:-1, 1:26];    % HT     CBW20 (52本, Non-HTと同じ配置)
subcarrierIndicesVHT20 = [-28:-1, 1:28];    % VHT    CBW20 (56本)

pktLog = struct('timeSec', {}, 'bssid', {}, 'addr1', {}, 'frameType', {}, 'ssid', {}, ...
    'phyFormat', {}, 'fcsVerified', {}, 'mpduCount', {}, 'csi', {});
bssidToSSID = containers.Map('KeyType', 'char', 'ValueType', 'char');

% 復号統計 (診断用)
stats = struct('detected', 0, 'timingSkip', 0, 'nonHT', 0, 'ht', 0, 'vht', 0, ...
    'htGF', 0, 'other', 0, 'htUnsupported', 0, 'vhtUnsupported', 0, ...
    'decodeOK', 0, 'noAddr2', 0, 'errors', 0);
errMsgs       = containers.Map('KeyType', 'char', 'ValueType', 'double');
deagStatusCnt = containers.Map('KeyType', 'char', 'ValueType', 'double');
nonBinaryShown = false;  % 非0/1データを検出した旨を一度だけ表示するためのフラグ
htRejects     = containers.Map('KeyType', 'char', 'ValueType', 'double');
htMCSCounts   = containers.Map('KeyType', 'double', 'ValueType', 'double');
htReservedOK  = 0;   % HT-SIG の予約ビットが仕様どおりだった件数
htSigParsed   = 0;   % HT-SIG の CRC を通り、ビット解釈まで到達した件数
vhtSigParsed  = 0;   % VHT-SIG-A の CRC を通り、ビット解釈まで到達した件数
vhtRejects    = containers.Map('KeyType', 'char', 'ValueType', 'double');
vhtBWCounts   = containers.Map('KeyType', 'double', 'ValueType', 'double');
vhtNSTSCounts = containers.Map('KeyType', 'double', 'ValueType', 'double');
vhtReservedOK = 0;   % VHT-SIG-A の予約ビットが仕様どおりだった件数
vhtDetailShown = 0;  % 内訳を表示した VHT パケット数
vhtDetailMax   = 5;  % 内訳を表示する最大件数

minPreambleLen = 560;   % L-STF..L-SIG + 2シンボル (フォーマット検出まで)
searchOffset = 0;

fprintf('\nオフライン復号を開始します...\n');
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

            case fmtStr == "HT-MF"
                stats.ht = stats.ht + 1;
                [st, consumed, entry, htInfo] = processHT(pkt, lltfChanEst, noiseEst, chanBW);

                if htInfo.mcs >= 0
                    htSigParsed = htSigParsed + 1;
                    if isKey(htMCSCounts, htInfo.mcs)
                        htMCSCounts(htInfo.mcs) = htMCSCounts(htInfo.mcs) + 1;
                    else
                        htMCSCounts(htInfo.mcs) = 1;
                    end
                end
                if htInfo.reservedOK
                    htReservedOK = htReservedOK + 1;
                end
                [deagStatusCnt, nonBinaryShown] = ...
                    recordDeag(deagStatusCnt, nonBinaryShown, 'HT', htInfo);
                if ~isempty(htInfo.reason)
                    stats.htUnsupported = stats.htUnsupported + 1;
                    if isKey(htRejects, htInfo.reason)
                        htRejects(htInfo.reason) = htRejects(htInfo.reason) + 1;
                    else
                        htRejects(htInfo.reason) = 1;
                    end
                end

            case fmtStr == "VHT"
                stats.vht = stats.vht + 1;
                [st, consumed, entry, vhtInfo] = processVHT(pkt, lltfChanEst, noiseEst, chanBW);

                if vhtInfo.bwMHz > 0
                    vhtSigParsed = vhtSigParsed + 1;
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
                [deagStatusCnt, nonBinaryShown] = ...
                    recordDeag(deagStatusCnt, nonBinaryShown, 'VHT', vhtInfo);
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

            case fmtStr == "HT-GF"
                % Greenfield はプリアンブル構造自体が異なり非対応
                stats.htGF = stats.htGF + 1;
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

                fprintf('  [%7.4fs] %-6s %-16s BSSID=%s%s%s\n', entry.timeSec, ...
                    entry.phyFormat, entry.frameType, entry.bssid, ...
                    ternary(~isempty(entry.ssid), sprintf('  SSID="%s"', entry.ssid), ''), ...
                    ternary(entry.mpduCount > 1, sprintf('  (MPDU集約x%d)', entry.mpduCount), ''));
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
            warning('decodeIQ_VHT:decodeError', 'パケット処理中にエラー: %s', ME.message);
        end
        searchOffset = nextSearch;
    end
end

elapsedDecode = toc(decodeTic);

%% ------------------------------------------------------------------------
%  5. 復号結果のサマリ表示
%  ------------------------------------------------------------------------
fprintf('\n[復号サマリ] (所要 %.2f s)\n', elapsedDecode);
fprintf('  パケット検出数            : %d\n', stats.detected);
fprintf('    タイミング不正でスキップ: %d\n', stats.timingSkip);
fprintf('    Non-HT として検出       : %d\n', stats.nonHT);
fprintf('    HT     として検出       : %d  (うち非対応構成 %d)\n', stats.ht, stats.htUnsupported);
fprintf('    VHT    として検出       : %d  (うち非対応構成 %d)\n', stats.vht, stats.vhtUnsupported);
fprintf('    HT-Greenfield            : %d  (非対応)\n', stats.htGF);
fprintf('    その他フォーマット      : %d  (非対応)\n', stats.other);

if stats.ht > 0
    fprintf('  [HT 詳細]\n');
    if htMCSCounts.Count > 0
        mKeys = sort(cell2mat(keys(htMCSCounts)));
        fprintf('    MCS の内訳: ');
        for k = 1:numel(mKeys)
            fprintf('MCS%d=%d回  ', mKeys(k), htMCSCounts(mKeys(k)));
        end
        fprintf('\n');
    end
    if htRejects.Count > 0
        rKeys = keys(htRejects);
        fprintf('    非対応の理由: ');
        for k = 1:numel(rKeys)
            fprintf('%s=%d回  ', rKeys{k}, htRejects(rKeys{k}));
        end
        fprintf('\n');
    end
    % 分母は「HT-SIG の CRC を通り、ビット解釈まで到達した件数」。検出だけ
    % されて CRC で落ちたものを含めると、自前パースの妥当性を測れない。
    fprintf('    HT-SIG予約ビット検証: %d/%d 個が仕様どおり (CRC通過分のみ)', ...
        htReservedOK, htSigParsed);
    if htSigParsed > 0 && htReservedOK < htSigParsed * 0.5
        fprintf('  <-- 大半が不一致。ビット解釈がズレている可能性あり\n');
    else
        fprintf('  (ビット解釈は妥当)\n');
    end
end

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
        multiKeys = nKeys(nKeys >= 2);
        nMultiStream = sum(cellfun(@(k) vhtNSTSCounts(k), num2cell(multiKeys)));
        fprintf('\n');
        if any(nKeys >= 2)
            fprintf(['    ※NSTS>=2 の分 (計%d件) は送信元 BSSID が分からず、\n', ...
                     '      特定のSSIDに帰属させることができません。\n'], nMultiStream);
        end
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
    fprintf('    SIG-A予約ビット検証: %d/%d 個が仕様どおり (CRC通過分のみ)', ...
        vhtReservedOK, vhtSigParsed);
    if vhtSigParsed > 0 && vhtReservedOK < vhtSigParsed * 0.5
        fprintf('  <-- 大半が不一致。ビット解釈がズレている可能性あり\n');
    else
        fprintf('  (ビット解釈は妥当)\n');
    end
end
fprintf('  MAC まで復号成功          : %d\n', stats.decodeOK);
fprintf('    うち Address2 無し(ACK/CTS等、BSSID判定不可): %d\n', stats.noAddr2);
fprintf('  復号エラー                : %d\n', stats.errors);

% --- A-MPDU 分解の結果 (データ復号が機能しているかの判断材料) ---
if deagStatusCnt.Count > 0
    fprintf('  [A-MPDU 分解結果]\n');
    dKeys = keys(deagStatusCnt);
    for k = 1:numel(dKeys)
        fprintf('    %-40s : %d回\n', dKeys{k}, deagStatusCnt(dKeys{k}));
    end
end

% --- FCS 検証の内訳 (全BSSID) ---
% FCS検証OK が 0 のフォーマットは、そのフォーマットのデータ復号が
% 成立していない (SNR不足かパラメータ推定ミス) ことを意味する。
if ~isempty(pktLog)
    fprintf('  [FCS 検証の内訳 (全BSSID)]\n');
    allFmt  = {pktLog.phyFormat};
    allFcsV = logical([pktLog.fcsVerified]);
    for f = {'Non-HT', 'HT', 'VHT'}
        sel = strcmpi(allFmt, f{1});
        if any(sel)
            fprintf('    %-6s : 記録%d件 (FCS検証OK=%d, ヘッダのみ推定=%d)\n', ...
                f{1}, sum(sel), sum(sel & allFcsV), sum(sel & ~allFcsV));
        end
    end
    fprintf(['    ※あるフォーマットで FCS検証OK が 0 件の場合、そのフォーマットの\n', ...
             '      データ復号が成立していない (受信SNR不足、または変調パラメータの\n', ...
             '      推定ミス) ことを示す。\n']);
end

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
%  6. 対象 SSID の BSSID を特定し、CSI を抽出
%  ------------------------------------------------------------------------
seenNetworks = struct('bssid', {}, 'ssid', {});
bssidKeys = keys(bssidToSSID);
for k = 1:numel(bssidKeys)
    seenNetworks(end+1) = struct('bssid', bssidKeys{k}, ...
        'ssid', bssidToSSID(bssidKeys{k})); %#ok<SAGROW>
end

% パケットが「その BSSID のネットワークに属する」かどうかは、送信元(Address2)
% と宛先(Address1)のどちらか一方が BSSID と一致すれば判定できる。
%   ダウンリンク (AP→クライアント): Address2(送信元) = BSSID
%   アップリンク (クライアント→AP): Address1(宛先)   = BSSID
% 送信元だけを見ると、クライアントが送るアップリンクの実データ(iperf3等)を
% 取りこぼすので注意。
allBssid = {pktLog.bssid};
allAddr1 = {pktLog.addr1};

fprintf('\n検出できたネットワーク一覧 (BSSID別の内訳):\n');
if isempty(seenNetworks)
    fprintf('  (なし)\n');
else
    isHTPkt  = strcmpi({pktLog.phyFormat}, 'HT');
    isVHTPkt = strcmpi({pktLog.phyFormat}, 'VHT');
    for k = 1:numel(seenNetworks)
        bssid = seenNetworks(k).bssid;
        belongs = strcmp(allBssid, bssid) | strcmp(allAddr1, bssid);
        nAll  = sum(belongs);
        nHT   = sum(isHTPkt  & belongs);
        nVHT  = sum(isVHTPkt & belongs);
        fprintf('  BSSID=%s  SSID="%s"  (全%d件, うちHT=%d件, VHT=%d件)\n', ...
            bssid, seenNetworks(k).ssid, nAll, nHT, nVHT);
    end
    fprintf(['  ※ここでの件数は MAC ヘッダまで読めた(=BSSID が判明した)パケットのみ。\n', ...
             '    複数空間ストリームのパケットは SISO 受信ではヘッダも含めて\n', ...
             '    全く復号できないため、この一覧には出てきません。\n']);
end

% --- 診断: 既知の BSSID を宛先(Address1)とする「見慣れない送信元」を列挙 ---
% これはその BSSID に接続しているクライアント端末の MAC アドレス候補
% (=アップリンク送信元) であり、Beacon を送らないため上の一覧には出てこない。
knownBssidSet = {seenNetworks.bssid};
uplinkSenders = containers.Map('KeyType', 'char', 'ValueType', 'double');
for k = 1:numel(pktLog)
    if any(strcmp(knownBssidSet, pktLog(k).addr1)) && ~any(strcmp(knownBssidSet, pktLog(k).bssid))
        s = pktLog(k).bssid;
        if isKey(uplinkSenders, s)
            uplinkSenders(s) = uplinkSenders(s) + 1;
        else
            uplinkSenders(s) = 1;
        end
    end
end
if uplinkSenders.Count > 0
    fprintf('  クライアント(アップリンク送信元)と思われる MAC アドレス:\n');
    ukeys = keys(uplinkSenders);
    for k = 1:numel(ukeys)
        % どの BSSID 宛てのアップリンクだったかも添える
        destBssid = '';
        for j = 1:numel(pktLog)
            if strcmp(pktLog(j).bssid, ukeys{k}) && any(strcmp(knownBssidSet, pktLog(j).addr1))
                destBssid = pktLog(j).addr1;
                break;
            end
        end
        fprintf('    %s -> BSSID=%s  (%d件)\n', ukeys{k}, destBssid, uplinkSenders(ukeys{k}));
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
    warning('decodeIQ_VHT:ssidNotFound', ...
        ['指定した SSID "%s" の Beacon/Probe Response を検出できませんでした。\n', ...
         'captureIQ.m の captureDuration を長くする、pktDetThreshold を下げる (例 0.3)、', ...
         'SSID の綴り・電波状況を確認する、などをお試しください。'], targetSSID);
else
    isMatch = strcmp(allBssid, targetBSSID) | strcmp(allAddr1, targetBSSID);
    matched = pktLog(isMatch);
    fprintf('\n対象 SSID "%s" (BSSID=%s) のパケット数: %d (アップリンク+ダウンリンク)\n', ...
        targetSSID, targetBSSID, numel(matched));
end

% --- フォーマット別に [パケット数 x サブキャリア数] の行列へまとめる ---
% Non-HT/HT は 52 本、VHT(20MHz) は 56 本とサブキャリア数が異なるため、
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
isHT    = strcmpi(allFormats, 'HT');
isVHT   = strcmpi(allFormats, 'VHT');

csiNonHT       = stackCSI(matched(isNonHT));
timeSecNonHT   = allTimeSec(isNonHT);
frameTypeNonHT = allFrameType(isNonHT);
fcsNonHT       = allFcs(isNonHT);

csiHT       = stackCSI(matched(isHT));
timeSecHT   = allTimeSec(isHT);
frameTypeHT = allFrameType(isHT);
fcsHT       = allFcs(isHT);

csiVHT       = stackCSI(matched(isVHT));
timeSecVHT   = allTimeSec(isVHT);
frameTypeVHT = allFrameType(isVHT);
fcsVHT       = allFcs(isVHT);

if ~isempty(matched)
    fprintf('  内訳: Non-HT=%d, HT=%d, VHT=%d\n', ...
        size(csiNonHT, 1), size(csiHT, 1), size(csiVHT, 1));
    totalMPDU = sum([matched.mpduCount]);
    fprintf('  PPDU(電波上の送信単位)数=%d に対し、集約されたMPDU(データ単位)の合計=%d\n', ...
        numel(matched), totalMPDU);
    nUnverified = sum(~allFcs);
    if nUnverified > 0
        fprintf(['  ※うち %d 件は FCS 未検証 (ペイロードにビット誤りがあり、\n', ...
                 '    MAC ヘッダのみから送信元を判定したもの)。CSI 自体は\n', ...
                 '    プリアンブルから算出しており影響を受けません。\n'], nUnverified);
    end
end

% --- ResultCSI.m 互換の「主」データを決める ---
% ResultCSI.m は csi (行列) / subcarrierIndices / packetStartIndex /
% csiMeta という変数名を前提とするため、3種類のうちパケット数が最も
% 多い形式をその名前でも保存する。
counts = [size(csiNonHT,1), size(csiHT,1), size(csiVHT,1)];
[~, bestIdx] = max(counts);
formatNames = {'Non-HT', 'HT', 'VHT'};
csiSets    = {csiNonHT, csiHT, csiVHT};
subcSets   = {subcarrierIndicesNonHT, subcarrierIndicesHT20, subcarrierIndicesVHT20};
timeSets   = {timeSecNonHT, timeSecHT, timeSecVHT};
frameSets  = {frameTypeNonHT, frameTypeHT, frameTypeVHT};
fcsSets    = {fcsNonHT, fcsHT, fcsVHT};

primaryFormat     = formatNames{bestIdx};
csi               = csiSets{bestIdx};
subcarrierIndices = subcSets{bestIdx};
timeSec           = timeSets{bestIdx};
frameType         = frameSets{bestIdx};
fcsVerified       = fcsSets{bestIdx};
phyFormat         = repmat({primaryFormat}, size(csi, 1), 1);

% ResultCSI.m は packetStartIndex と sampleRate から時間軸を作る
packetStartIndex = round(timeSec(:) * sampleRate);

if ~isempty(csi)
    fprintf('  主データ(変数 csi): %s フォーマット, %d パケット x %d サブキャリア\n', ...
        primaryFormat, size(csi, 1), size(csi, 2));
end

%% ------------------------------------------------------------------------
%  7. 保存 (.mat) — HDD と USB メモリの両方へ
%  ------------------------------------------------------------------------
% 変数名は ResultCSI.m と揃えて csiMeta とする
csiMeta = struct();
csiMeta.description      = 'CSI filtered by target SSID (Non-HT / HT(NSS=1,20MHz) / SU-VHT(NSTS=1,20MHz))';
csiMeta.primaryFormat    = primaryFormat;   % 変数 csi がどの形式か
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
csiMeta.captureDatetime  = timestamp;         % キャプチャ時刻
csiMeta.decodeDatetime   = datestr(now, 'yyyymmddHHMM');   % 復号を行った時刻
csiMeta.sourceRawFile    = inputRawFile;      % どの生IQから作られたか
csiMeta.captureMeta      = meta;              % キャプチャ時の全メタデータ
csiMeta.matlabVersion    = version;

targetSSIDOut  = targetSSID;  %#ok<NASGU>
targetBSSIDOut = targetBSSID; %#ok<NASGU>

saveVars = { ...
    'csi', 'subcarrierIndices', 'packetStartIndex', 'csiMeta', ...
    'phyFormat', 'fcsVerified', 'timeSec', 'frameType', ...
    'csiNonHT', 'timeSecNonHT', 'frameTypeNonHT', 'fcsNonHT', ...
    'csiHT', 'timeSecHT', 'frameTypeHT', 'fcsHT', ...
    'csiVHT', 'timeSecVHT', 'frameTypeVHT', 'fcsVHT', ...
    'subcarrierIndicesNonHT', 'subcarrierIndicesHT20', 'subcarrierIndicesVHT20', ...
    'targetSSIDOut', 'targetBSSIDOut', 'seenNetworks'};

fprintf('\n');
nSaved = 0;
for k = 1:numel(outMatFiles)
    try
        save(outMatFiles{k}, saveVars{:}, '-v7.3');
        fprintf('結果を保存しました: %s\n', outMatFiles{k});
        nSaved = nSaved + 1;
    catch ME
        % 片方 (USB の抜き差し等) が失敗しても、もう片方は残す
        warning('decodeIQ_VHT:saveFailed', ...
            '保存に失敗しました (%s): %s', outMatFiles{k}, ME.message);
    end
end

if nSaved == 0
    error('decodeIQ_VHT:allSavesFailed', ...
        'すべての出力先への保存に失敗しました。ドライブの接続を確認してください。');
end

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

function [cnt, shown] = recordDeag(cnt, shown, fmtName, info)
    % A-MPDU 分解の結果を集計する。ここが失敗しているか成功しているかで、
    % 「データ復号自体が壊れている」のか「FCS だけ通らない」のかを切り分ける。
    % deagStatus が空 = 分解まで到達せず (SIG段階で棄却) なので集計しない
    if ~isempty(info.deagStatus)
        key = sprintf('%s: %s', fmtName, info.deagStatus);
        if isKey(cnt, key)
            cnt(key) = cnt(key) + 1;
        else
            cnt(key) = 1;
        end
    end

    % 想定外のデータ形式は一度だけ警告する (原因究明の手掛かりになるため)。
    % ビット列と16進文字列はどちらも扱えるので、ここに来るのはそれ以外。
    if info.nonBinary && ~shown
        shown = true;
        fprintf(['    <警告> 分解した MPDU がビット列でも16進文字列でもありません。\n', ...
                 '            class=%s, 値域=[%g %g]\n', ...
                 '            この形式は MAC ヘッダとして解釈できないため除外します。\n'], ...
            info.dataClass, info.dataRange(1), info.dataRange(2));
    end
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

function [status, consumed, entry, info] = processHT(pkt, lltfChanEst, noiseEst, chanBW)
    % HT-Mixed (802.11n, NSS=1, 20MHz) パケットを復号する。
    %   status: 1=成功, 0=失敗/非対応, -1=サンプル不足
    %   info:   診断用 (棄却理由・観測されたMCS/NSS/帯域幅)
    entry = [];
    consumed = 560;   % L-STF..HT-SIG
    info = struct('reason', '', 'mcs', -1, 'nss', 0, 'cbw40', false, ...
        'coding', 0, 'gi', 0, 'aggregated', false, 'reservedOK', false, ...
        'mpduCount', 0, 'deagStatus', '', ...
        'nonBinary', false, 'dataClass', '', 'dataRange', [0 0]);

    % --- HT-SIG 復号 (2シンボル, pkt(401:560)) ---
    [htsigBits, htsigFail] = wlanHTSIGRecover(pkt(401:560), lltfChanEst, noiseEst, chanBW);
    if htsigFail
        info.reason = 'HT-SIG CRC失敗';
        status = 0;
        return;
    end

    ht = parseHTSIGBits(htsigBits);
    info.mcs        = ht.mcs;
    info.nss        = ht.nss;
    info.cbw40      = ht.cbw40;
    info.coding     = ht.coding;
    info.gi         = ht.gi;
    info.aggregated = ht.aggregated;
    info.reservedOK = ht.reservedOK;

    % 以下はいずれも SISO(受信1本)/20MHz 受信では扱えない構成
    if ~ht.isValid
        info.reason = 'HT-SIG解釈不能';
        status = 0; return;
    elseif ht.cbw40
        info.reason = '帯域幅40MHz(20MHz受信では復号不可)';
        status = 0; return;
    elseif ht.nss ~= 1
        info.reason = sprintf('NSS=%d(アンテナ1本では復号不可)', ht.nss);
        status = 0; return;
    elseif ht.psduLen <= 0 || ht.psduLen > 65535
        info.reason = 'HT-LENGTH不正';
        status = 0; return;
    end

    codingStr = ternary(ht.coding == 1, 'LDPC', 'BCC');
    giStr     = ternary(ht.gi == 1, 'Short', 'Long');

    try
        cfgHT = wlanHTConfig('ChannelBandwidth', chanBW, ...
            'NumTransmitAntennas', 1, 'NumSpaceTimeStreams', 1, ...
            'MCS', ht.mcs, 'PSDULength', ht.psduLen, ...
            'ChannelCoding', codingStr, 'GuardInterval', giStr);
        ind = wlanFieldIndices(cfgHT);
    catch
        info.reason = 'HTコンフィグ生成失敗';
        status = 0;
        return;
    end

    if numel(pkt) < ind.HTData(2)
        status = -1;
        return;   % データ部がまだ届いていない -> 次フレーム待ち
    end
    consumed = double(ind.HTData(2));

    % --- HT-LTF を復調してチャネル推定 (= CSI) ---
    htDemod   = wlanHTLTFDemodulate(pkt(ind.HTLTF(1):ind.HTLTF(2)), cfgHT);
    htChanEst = wlanHTLTFChannelEstimate(htDemod, cfgHT);

    rxPSDU = wlanHTDataRecover(pkt(ind.HTData(1):ind.HTData(2)), htChanEst, noiseEst, cfgHT);

    % --- HT の PSDU は A-MPDU の場合があるので分解してから MPDU を復号 ---
    [cfgMAC, payload, ok, deagInfo] = decodeAggregatedPSDU(rxPSDU, cfgHT);
    info.mpduCount  = deagInfo.mpduCount;
    info.deagStatus = deagInfo.status;
    info.nonBinary  = deagInfo.nonBinary;
    info.dataClass  = deagInfo.dataClass;
    info.dataRange  = deagInfo.dataRange;
    if ~ok
        info.reason = sprintf('MPDU復号失敗(分解=%s,MPDU数=%d)', ...
            deagInfo.status, deagInfo.mpduCount);
        status = 0;
        return;
    end

    entry = buildEntry(cfgMAC, payload, 'HT', htChanEst);
    if ~isempty(entry)
        entry.fcsVerified = ~deagInfo.headerOnly;
        entry.mpduCount   = deagInfo.mpduCount;   % 0 = A-MPDU分解失敗
    end
    status = 1;
end

function out = parseHTSIGBits(bits)
    % HT-SIG (48bit = HT-SIG1(24) + HT-SIG2(24)) を解釈する。
    % IEEE Std 802.11-2016, 19.3.9.4 (HT-SIG) に基づく。多ビットフィールドは
    % LSB 先頭で送信される。
    %   bits(1:7)   MCS (0-31。0-7=1ストリーム,8-15=2,16-23=3,24-31=4)
    %   bits(8)     CBW (0=20MHz, 1=40MHz)
    %   bits(9:24)  HT-LENGTH (PSDU長 [byte] を直接表す)
    %   bits(28)    Aggregation (1=HT-DataがA-MPDU)
    %   bits(31)    FEC Coding (0=BCC, 1=LDPC)
    %   bits(32)    Short GI
    bits = double(bits(:)).';
    out = struct('isValid', false, 'mcs', -1, 'nss', 0, 'cbw40', false, ...
        'psduLen', 0, 'coding', 0, 'gi', 0, 'aggregated', false, ...
        'reservedOK', false);

    if numel(bits) < 34
        return;
    end

    % 自己チェック: HT-SIG2 の Reserved ビット (全体で27ビット目) は仕様上 1 固定
    out.reservedOK = bits(27) == 1;

    mcsBits = bits(1:7);
    out.mcs   = sum(mcsBits .* 2.^(0:6));
    out.nss   = 1 + floor(out.mcs / 8);
    out.cbw40 = bits(8) == 1;

    lenBits = bits(9:24);   % LSB先頭
    out.psduLen = sum(lenBits .* 2.^(0:15));

    out.aggregated = bits(28) == 1;
    out.coding      = bits(31);
    out.gi           = bits(32);
    out.isValid = true;
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
        'mpduCount', 0, 'deagStatus', '', ...
        'nonBinary', false, 'dataClass', '', 'dataRange', [0 0]);

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
    if info.lsigLen > 0 && ~isempty(ndbps)
        % RXTIME [us] からプリアンブル (L-STF..VHT-SIG-B) を引いてデータ部の
        % 継続時間を求める。20MHz/NSTS=1 のプリアンブルは 40us。
        rxTimeUs   = 4 * ceil((info.lsigLen + 3) / 3) + 20;
        dataTimeUs = rxTimeUs - indProbe.VHTSIGB(2) / 20;   % 20 samples/us

        if vhtA.gi == 1
            % Short GI (3.6us/シンボル)。データ部は 4us 境界までパディング
            % されるため、パディングが 3.6us 以上になる NSYM ≡ 9 (mod 10) の
            % ときだけ floor が 1 多くなる。この曖昧性を VHT-SIG-A2 B1
            % (Short GI NSYM Disambiguation) が示すので、それで補正する。
            info.nsym = floor(dataTimeUs / 3.6);
            if vhtA.sgiDisamb == 1
                info.nsym = info.nsym - 1;
            end
        else
            info.nsym = round(dataTimeUs / 4);
        end

        if info.nsym > 0
            if vhtA.coding == 1
                % LDPC: NSYM = ceil((8*APEP + 16)/NDBPS) + (追加シンボル)
                %       末尾ビット(6bit)が無く、LDPC Extra OFDM Symbol
                %       (VHT-SIG-A2 B3) の分は APEP に寄与しない。
                nsymData = info.nsym - vhtA.ldpcExtra;
                if nsymData > 0
                    info.apepFromLSIG = floor((nsymData * ndbps - 16) / 8);
                end
            else
                % BCC: NSYM = ceil((8*APEP + 16 + 6*NES)/NDBPS), 20MHz は NES=1
                info.apepFromLSIG = floor((info.nsym * ndbps - 22) / 8);
            end
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
    [cfgMAC, payload, ok, deagInfo] = decodeAggregatedPSDU(rxPSDU, cfgVHT);
    info.mpduCount  = deagInfo.mpduCount;
    info.deagStatus = deagInfo.status;
    info.nonBinary  = deagInfo.nonBinary;
    info.dataClass  = deagInfo.dataClass;
    info.dataRange  = deagInfo.dataRange;
    if ~ok
        info.reason = sprintf('MPDU復号失敗(分解=%s,MPDU数=%d)', ...
            deagInfo.status, deagInfo.mpduCount);
        status = 0;
        return;
    end

    entry = buildEntry(cfgMAC, payload, 'VHT', vhtChanEst);
    if ~isempty(entry)
        entry.fcsVerified = ~deagInfo.headerOnly;
        entry.mpduCount   = deagInfo.mpduCount;   % 0 = A-MPDU分解失敗
    end
    status = 1;
end

function [cfgMAC, payload, ok, deagInfo] = decodeAggregatedPSDU(rxPSDU, cfgFormat)
    % HT/VHT の PSDU を A-MPDU として分解し、最初に復号できた MPDU を返す。
    % cfgFormat は wlanHTConfig または wlanVHTConfig のいずれでもよい。
    cfgMAC = [];
    payload = [];
    ok = false;
    deagInfo = struct('status', 'N/A', 'mpduCount', 0, 'headerOnly', false, ...
        'nonBinary', false, 'dataClass', '', 'dataRange', [0 0]);

    mpduList = {};
    try
        [mpduList, ~, deagStatus] = wlanAMPDUDeaggregate(rxPSDU, cfgFormat, 'DataFormat', 'bits');
        deagInfo.status = char(string(deagStatus));
    catch ME
        deagInfo.status = ['例外: ' ME.message];
    end
    deagInfo.mpduCount = numel(mpduList);

    % 全体の status が Success でなくても、取り出せた MPDU は個別に試す
    % (一部のサブフレームだけ壊れている場合でも残りは復号できるため)
    %
    % 重要: wlanAMPDUDeaggregate に 'DataFormat','bits' を指定しても、返る
    % MPDU が16進文字列 (char, '0'-'9''A'-'F') になる場合がある。その形式の
    % まま 'DataFormat','bits' で wlanMPDUDecode を呼ぶと形式不一致で必ず
    % 失敗するため、実際のデータ形式を判定して合わせる。
    wState = warning('off', 'all');
    for m = 1:numel(mpduList)
        dfmt = mpduDataFormat(mpduList{m});
        if isempty(dfmt)
            continue;   % 判別できない形式
        end
        try
            [c, p, st] = wlanMPDUDecode(mpduList{m}, cfgFormat, 'DataFormat', dfmt);
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
        dfmt = mpduDataFormat(rxPSDU);
        if ~isempty(dfmt)
            try
                wState = warning('off', 'all');
                [c, p, st] = wlanMPDUDecode(rxPSDU, cfgFormat, 'DataFormat', dfmt);
                warning(wState);
                if strcmpi(string(st), "Success")
                    cfgMAC = c; payload = p; ok = true;
                    return;
                end
            catch
            end
        end
    end

    % --- フォールバック: FCS 検証に失敗しても MAC ヘッダから送信元を読む ---
    % FCS は MPDU 全体 (最大 1500B 超) を検証するため、末尾の 1 ビット誤りでも
    % 失敗する。一方 CSI の帰属に必要なのは先頭 24 バイトのヘッダだけであり、
    % ここが無傷である確率は高い。
    %
    % ただし対象は A-MPDU 分解に成功した MPDU に限る。分解に失敗した PSDU
    % 全体をヘッダとみなす処理は、デリミタで区切れていない = 復号が壊れて
    % いる証拠があるにもかかわらず、緩い妥当性検査を偶然通ったランダムな
    % ビット列を「送信元アドレス」として大量に生んでしまうため行わない。
    for m = 1:numel(mpduList)
        [dfmt, oct] = mpduDataFormat(mpduList{m});
        if isempty(dfmt)
            % ビット列でも16進文字列でもない = 想定外のデータ形式。
            % 静かに壊れた結果を出さないよう記録して弾く。
            raw = double(mpduList{m}(:));
            deagInfo.nonBinary = true;
            deagInfo.dataClass = class(mpduList{m});
            if ~isempty(raw)
                deagInfo.dataRange = [min(raw) max(raw)];
            end
            continue;
        end
        hdr = parseMACHeaderFromOctets(oct);
        if hdr.valid
            cfgMAC  = hdr;      % wlanMACFrameConfig ではなく自前の構造体
            payload = [];
            ok = true;
            deagInfo.headerOnly = true;
            return;
        end
    end
end

function [fmtStr, oct] = mpduDataFormat(data)
    % MPDU データの実際の形式を判定し、オクテット列も返す。
    %   fmtStr: 'bits'   … 0/1 の数値ベクトル
    %           'octets' … 16進文字列 (char/string)
    %           ''       … 判別不能
    % wlanMPDUDecode / wlanAMPDUDeaggregate の 'DataFormat' にはこの値を
    % そのまま渡せる。
    fmtStr = '';
    oct    = [];

    if isempty(data)
        return;
    end

    if ischar(data) || isstring(data)
        s = char(data);
        s = s(:).';
        s = s(~isspace(s));
        if isempty(s) || ~all(isstrprop(s, 'xdigit'))
            return;   % 16進文字列ではない
        end
        fmtStr = 'octets';
        oct    = hexStrToOctets(s);
        return;
    end

    d = double(data(:));
    if all(d == 0 | d == 1)
        fmtStr = 'bits';
        oct    = bitsToOctets(d);
    end
end

function oct = hexStrToOctets(s)
    % 16進文字列 ('4A3B...') をオクテット列 [74 59 ...] へ変換する
    n = floor(numel(s) / 2) * 2;
    if n == 0
        oct = [];
        return;
    end
    v = sscanf(s(1:n), '%2x');
    oct = v(:).';
end

function hdr = parseMACHeaderFromOctets(oct)
    % MPDU のオクテット列から MAC ヘッダ (Frame Control / Address1-3) を
    % 直接読む。FCS を検証しないため、内容が正しい保証は無い点に注意。
    hdr = struct('valid', false, 'FrameType', '', 'Address1', '', 'Address2', '', ...
        'ManagementConfig', [], 'headerOnly', true);

    if numel(oct) < 24
        return;   % Address2 まで読めない
    end
    if any(oct < 0 | oct > 255 | mod(oct, 1) ~= 0)
        return;   % オクテットとして解釈できない値が混じっている
    end

    fc0     = oct(1);
    fc1     = oct(2);
    version = bitand(fc0, 3);
    type    = bitand(bitshift(fc0, -2), 3);
    subtype = bitand(bitshift(fc0, -4), 15);

    if version ~= 0
        return;   % プロトコルバージョンは 0 のはず。違えば復号が壊れている
    end

    % ToDS/FromDS の組み合わせ検査。ToDS=1&FromDS=1 は 4アドレス形式
    % (WDS) で、通常のインフラストラクチャ BSS では使われない。
    % また管理フレームでは両方 0 でなければならない。
    toDS   = bitand(fc1, 1);
    fromDS = bitand(bitshift(fc1, -1), 1);
    if toDS == 1 && fromDS == 1
        return;
    end
    if type == 0 && (toDS ~= 0 || fromDS ~= 0)
        return;
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

    addr1 = oct(5:10);
    addr2 = oct(11:16);

    % 送信元 (Address2) が全0 / ブロードキャストのものは復号が壊れている。
    % また Address2 のグループビット (先頭オクテットの bit0) は送信元
    % アドレスでは必ず 0 になる。
    if all(addr2 == 0) || all(addr2 == 255) || bitand(addr2(1), 1) == 1
        return;
    end

    hdr.valid     = true;
    hdr.FrameType = typeName;
    hdr.Address1  = upper(sprintf('%02X', addr1));
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
    %
    % 'bssid' には Address2 (送信元/TA) を入れる。ダウンリンク (AP→クライアント)
    % では Address2=APのBSSIDだが、アップリンク (クライアント→AP) では
    % Address2=クライアントのMACになり、代わりに Address1(宛先/RA)=APのBSSID
    % となる。どちらの向きも取りこぼさないよう 'addr1' も別途保持する。
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

    addr1Str = '';
    try
        addr1Str = char(string(cfgMAC.Address1));
    catch
        % Address1 を持たないオブジェクトの場合は空のままにする
    end

    entry = struct( ...
        'timeSec',     0, ...            % 呼び出し側で設定
        'bssid',       char(string(cfgMAC.Address2)), ...
        'addr1',       addr1Str, ...
        'frameType',   frameType, ...
        'ssid',        ssidStr, ...
        'phyFormat',   phyFormat, ...
        'fcsVerified', true, ...         % HT/VHT のヘッダのみ復号時は呼び出し側で false
        'mpduCount',   1, ...            % 集約されている場合は呼び出し側で上書き
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
    %   bits(25)    Short GI               bits(26)    Short GI NSYM 曖昧性解消
    %   bits(27)    Coding (0=BCC,1=LDPC)  bits(28)    LDPC Extra OFDM Symbol
    %   bits(29:32) SU VHT-MCS
    bits = double(bits(:)).';
    out = struct('isValid', false, 'is20MHz', false, 'isSU', false, ...
        'nsts', 0, 'stbc', false, 'groupId', 0, 'gi', 0, 'coding', 0, ...
        'mcs', 0, 'bwMHz', 0, 'reservedOK', false, ...
        'sgiDisamb', 0, 'ldpcExtra', 0);

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
    out.gi        = bits(25);
    out.sgiDisamb = bits(26);
    out.coding    = bits(27);
    out.ldpcExtra = bits(28);
    out.mcs       = sum(bits(29:32) .* 2.^(0:3));
    out.isValid   = true;
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
    if isempty(payload)
        return;
    end

    % payload はビット列でも16進文字列でも返り得るので、実際の形式を
    % 判定してオクテット列へ揃える。
    [dfmt, oct] = mpduDataFormat(payload);
    if isempty(dfmt)
        return;
    end
    payload = uint8(oct);

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
    % ビット列をオクテット列へ変換する (各オクテット内は LSB 先頭)。
    %
    % 入力が 0/1 でない場合 (軟判定値・オクテット列など) は変換結果が
    % 0..255 の範囲を外れ、MAC アドレスとして解釈すると 6バイトのはずが
    % 12バイト分の文字列になるなど、静かに壊れたデータを生む。そのため
    % ここで明示的に検査し、2値でなければ空を返して呼び出し側で弾く。
    bits = double(bits(:));
    if isempty(bits) || ~all(bits == 0 | bits == 1)
        bytes = [];
        return;
    end
    n = floor(numel(bits) / 8) * 8;
    if n == 0
        bytes = [];
        return;
    end
    b = reshape(bits(1:n), 8, []);
    bytes = (2.^(0:7)) * b;   % 1 x (n/8), 各要素は 0..255
end

function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end

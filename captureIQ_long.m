%% captureIQ_long.m
% =========================================================================
%  [第1段・長時間版] 生 IQ をディスクへ逐次書き出しながら長時間キャプチャする
% -------------------------------------------------------------------------
%  captureIQ.m との違い:
%    captureIQ.m は全サンプルをメモリ上のバッファに溜めてから .mat 保存する。
%    そのためキャプチャ長の上限が搭載メモリで決まってしまう。
%
%      complex double = 16 byte/sample、20 MSps なので 320 MB/s
%        5 秒 →  1.6 GB   (ピークはその2倍)
%      120 秒 → 38.4 GB   ← 16GB 搭載機では確保すらできない
%
%    本スクリプトはメモリに溜めず、受信したフレームをその都度ファイルへ
%    書き出す。上限はメモリではなくディスク容量と書き込み速度になる。
%
%  int16 で保存する理由:
%    書き込みが受信に追いつかないとオーバーラン (サンプルの取りこぼし) に
%    なるため、必要な書き込み速度を下げる必要がある。
%
%      complex double : 16 byte/sample → 320 MB/s  USB接続HDDでは不可能
%      complex single :  8 byte/sample → 160 MB/s  厳しい
%      int16 (sc16)   :  4 byte/sample →  80 MB/s  HDDでも可能  ← これを使う
%
%    int16 は USRP が USB で送ってくる元のデータ形式 (sc16) そのものであり、
%    ADC は 12bit なので情報は一切失われない。復号側で double に戻す。
%
%    ※OutputDataType に 'int16' は直接指定できない (有効なのは
%      'Same as transport data type' / 'double' / 'single' の3つ)。
%      転送形式 (sc16 = int16) に合わせるため
%      'Same as transport data type' を指定し、実際に届いたデータの型は
%      最初のフレームで判定して書き出し精度と倍率を決めている。
%
%  出力ファイル:
%    [1] <hddSavePath>/<yyyymmddHHMM>_raw.bin
%          I,Q を交互に並べたべた書き (I0,Q0,I1,Q1,...)。通常は int16。
%          120 秒で約 9.6 GB
%    [2] <hddSavePath>/<yyyymmddHHMM>_rawmeta.mat
%          変数 meta … 取得条件・オーバーラン回数・書き込み統計
%    [3] writeSegments = true のとき
%        <hddSavePath>/<yyyymmddHHMM>_seg01_raw.mat, _seg02_raw.mat, ...
%          [1] を segmentDuration 秒ごとに分割し、captureIQ.m と同じ形式
%          (complex double の iq + meta) に変換したもの。
%          既存の decodeIQ_*.m / decode_VHT_v2.m がそのまま読める。
%
%  なぜ分割するのか:
%    復号側 (decodeIQ_*.m) も iq をまるごと double でメモリに載せるため、
%    120 秒を1ファイルにすると復号時に 38.4 GB 必要になり同じ壁にぶつかる。
%    10 秒ごとに分けておけば 1 ファイル 3.2 GB で処理できる。
%
%    分割は受信の「後」に行うので、受信そのものは 120 秒間途切れない。
%    各セグメントの meta.segmentStartTimeSec にキャプチャ開始からの
%    経過時刻が入るので、複数セグメントの CSI を時系列で繋ぐときは
%    decodeIQ_*.m が出力する timeSec にこの値を足すこと。
%
%  必要環境:
%    - MATLAB / Communications Toolbox
%    - Communications Toolbox Support Package for USRP Radio
%    - USB 3.0 接続された USRP B205 mini-i
%    - 持続書き込み 80 MB/s 以上を出せる保存先
%      (2.5インチ USB HDD で 100〜130 MB/s 程度なので条件は満たすが余裕は
%       少ない。オーバーランが出る場合は SSD への保存を検討すること)
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

% --- 保存先 (ホストPC に接続された HDD) ---------------------------------
%     現在の環境: HDPC-UT (D:)
hddSavePath = 'D:\IQ_raw';

% --- Wi-Fi 受信パラメータ (5GHz 帯 ch36 / 20MHz) -------------------------
centerFrequency = 5.180e9;      % [Hz] 5GHz 帯 ch36 (20MHz 帯域幅)
sampleRate      = 20e6;         % [Sps] 20 MHz 帯域
gain            = 40;           % [dB] B200 系は 0〜76 dB 程度
captureDuration = 120.0;        % [s] 長時間キャプチャ
samplesPerFrame = 20000;        % [samples/frame]
usrpPlatform    = 'B200';
usrpSerialNum   = '3240497';

% --- 分割出力の設定 ------------------------------------------------------
% true にすると、キャプチャ後に .bin を分割して captureIQ.m と同じ形式の
% *_raw.mat を書き出す。既存の decodeIQ_*.m をそのまま使いたい場合は true。
writeSegments   = true;
segmentDuration = 5.0;          % [s] 1セグメントの長さ。
                                 %     5s = complex double で 1.6 GB、
                                 %     変換中のピークで約 3.2 GB 必要。
                                 %     搭載メモリに合わせて調整すること
                                 %     (16GB 搭載なら 5〜10s が妥当)。

%% ------------------------------------------------------------------------
%  2. 保存先の準備と容量の確認
%  ------------------------------------------------------------------------
if ~exist(hddSavePath, 'dir')
    fprintf('保存先フォルダが存在しないため作成します: %s\n', hddSavePath);
    [ok, msg] = mkdir(hddSavePath);
    if ~ok
        error('captureIQ_long:mkdirFailed', ...
            'HDD 保存先フォルダを作成できませんでした (%s): %s', hddSavePath, msg);
    end
end

testFile = fullfile(hddSavePath, '.write_test.tmp');
fidTest  = fopen(testFile, 'w');
if fidTest == -1
    error('captureIQ_long:hddNotWritable', ...
        'HDD 保存先に書き込みできません: %s', hddSavePath);
end
fclose(fidTest);
delete(testFile);

% int16 の I/Q なので 4 byte/sample
bytesPerSample = 4;
binBytes  = captureDuration * sampleRate * bytesPerSample;
segBytes  = 0;
if writeSegments
    % 分割後の .mat は complex double (16 byte/sample)
    segBytes = captureDuration * sampleRate * 16;
end
totalBytes = binBytes + segBytes;

fprintf('保存予定サイズ:\n');
fprintf('  生データ(.bin, int16)  : %6.2f GB\n', binBytes / 1e9);
if writeSegments
    fprintf('  分割後(.mat, double)   : %6.2f GB\n', segBytes / 1e9);
end
fprintf('  合計                   : %6.2f GB\n', totalBytes / 1e9);
fprintf('  必要な書き込み速度     : %6.1f MB/s\n', ...
    sampleRate * bytesPerSample / 1e6);

% 空き容量の確認 (取得できない環境では警告のみ)
try
    freeBytes = double(java.io.File(hddSavePath).getFreeSpace());
    fprintf('  保存先の空き容量       : %6.2f GB\n', freeBytes / 1e9);
    if freeBytes < totalBytes * 1.1
        error('captureIQ_long:notEnoughSpace', ...
            ['保存先の空き容量が不足しています。\n', ...
             '  必要: %.2f GB / 空き: %.2f GB\n', ...
             'captureDuration を短くするか、writeSegments を false にしてください。'], ...
            totalBytes / 1e9, freeBytes / 1e9);
    end
catch ME
    if strcmp(ME.identifier, 'captureIQ_long:notEnoughSpace')
        rethrow(ME);
    end
    warning('captureIQ_long:freeSpaceUnknown', ...
        '空き容量を確認できませんでした。手動で確認してください。');
end

% 分割セグメントがメモリに載るかの確認
if writeSegments
    segPeakBytes = segmentDuration * sampleRate * 16 * 2;   % 変換中は約2倍
    fprintf('  分割時のピークメモリ   : %6.2f GB (segmentDuration=%.1fs)\n', ...
        segPeakBytes / 1e9, segmentDuration);
end

timestamp   = datestr(now, 'yyyymmddHHMM');
binFile     = fullfile(hddSavePath, [timestamp '_raw.bin']);
metaFile    = fullfile(hddSavePath, [timestamp '_rawmeta.mat']);

%% ------------------------------------------------------------------------
%  3. USRP B205 mini-i 受信機オブジェクトの生成
%  ------------------------------------------------------------------------
radioInfo = [];
try
    radioInfo = findsdru();
    if isempty(radioInfo)
        warning('captureIQ_long:noRadio', ...
            'USRP 機器が検出されませんでした。USB 接続と電源を確認してください。');
    else
        fprintf('\n検出された USRP 機器:\n');
        for k = 1:numel(radioInfo)
            fprintf('  Platform=%s, SerialNum=%s, Status=%s\n', ...
                radioInfo(k).Platform, radioInfo(k).SerialNum, radioInfo(k).Status);
        end
    end
catch ME
    warning('captureIQ_long:findsdruFailed', 'findsdru の実行に失敗しました: %s', ME.message);
end

if isempty(usrpSerialNum)
    error('captureIQ_long:noSerialNum', ...
        'usrpSerialNum を指定してください (findsdru の表示を参照)。');
end

% USRP が USB で送ってくる形式 (sc16 = 複素 int16) のまま受け取るのが
% このスクリプトの要点。変換コストが無く、書き込み量が double の 1/4 で済む。
%
% 注意: OutputDataType に 'int16' は指定できない。指定できるのは
%   'Same as transport data type' / 'double' / 'single' の3つで、
%   転送形式に合わせるには 'Same as transport data type' を使う。
rx = comm.SDRuReceiver( ...
    'Platform',            usrpPlatform, ...
    'SerialNum',           usrpSerialNum, ...
    'CenterFrequency',     centerFrequency, ...
    'Gain',                gain, ...
    'MasterClockRate',     sampleRate * 2, ...
    'DecimationFactor',    2, ...
    'OutputDataType',      'Same as transport data type', ...
    'SamplesPerFrame',     samplesPerFrame);

% 転送形式を明示する。既定は sc16 (int16) だが、プロパティを持たない
% バージョンもあるため失敗しても続行する。
try
    rx.TransportDataType = 'int16';
catch
    % 既定のまま使う。実際の型は最初のフレームを受けてから判定する。
end

fprintf('\n受信設定:\n');
fprintf('  中心周波数      : %.4f GHz\n', centerFrequency / 1e9);
fprintf('  サンプルレート  : %.3f MSps (帯域幅)\n', sampleRate / 1e6);
fprintf('  ゲイン          : %d dB\n', gain);
fprintf('  キャプチャ時間  : %.2f s\n', captureDuration);
fprintf('  データ形式      : 転送形式のまま (最初のフレームで判定)\n');
fprintf('  保存先(.bin)    : %s\n', binFile);

%% ------------------------------------------------------------------------
%  4. IQ キャプチャ (受信しながらディスクへ逐次書き出し)
%  ------------------------------------------------------------------------
totalSamplesTarget   = round(captureDuration * sampleRate);
totalSamplesCaptured = 0;
overrunCount         = 0;
writeSecTotal        = 0;    % fwrite に費やした総時間
writeSecMax          = 0;    % 1回の fwrite の最大時間
nextReportSample     = sampleRate;   % 1秒ごとに進捗表示

% 受信データの実際の型は最初のフレームを見て決める。
% 'Same as transport data type' が何になるかは環境依存のため、
% 型に応じて書き出し精度・1サンプルあたりのバイト数・double へ戻すときの
% 倍率を切り替える。
rawPrecision   = '';   % fwrite/fread に渡す精度
rawScaleFactor = 1;    % double へ戻すときに掛ける倍率

fidBin = fopen(binFile, 'w');
if fidBin == -1
    release(rx);
    error('captureIQ_long:binOpenFailed', ...
        '出力ファイルを開けませんでした: %s', binFile);
end

fprintf('\nIQ キャプチャを開始します (%.0f 秒)...\n', captureDuration);
captureTic = tic;

% エラーや中断が起きても USRP とファイルを必ず解放する
try
    while totalSamplesCaptured < totalSamplesTarget
        [iqData, dataLen, overrun] = rx();
        if dataLen == 0
            continue;
        end
        if overrun
            overrunCount = overrunCount + 1;
        end

        % --- 最初のフレームで実際のデータ型を判定する ---
        if isempty(rawPrecision)
            switch class(iqData)
                case 'int16'
                    rawPrecision = 'int16';  rawScaleFactor = 1/32768;
                    bytesPerSample = 4;
                case 'int8'
                    rawPrecision = 'int8';   rawScaleFactor = 1/128;
                    bytesPerSample = 2;
                case 'single'
                    rawPrecision = 'single'; rawScaleFactor = 1;
                    bytesPerSample = 8;
                case 'double'
                    rawPrecision = 'double'; rawScaleFactor = 1;
                    bytesPerSample = 16;
                otherwise
                    fclose(fidBin);
                    release(rx);
                    error('captureIQ_long:unsupportedType', ...
                        '想定外のデータ型です: %s', class(iqData));
            end
            reqMBs = sampleRate * bytesPerSample / 1e6;
            fprintf('  受信データ型    : %s (%d byte/sample, 必要書込速度 %.0f MB/s)\n', ...
                rawPrecision, bytesPerSample, reqMBs);
            if ~strcmp(rawPrecision, 'int16')
                fprintf(['  ※int16 以外で受信しています。書き込み量が増えるため\n', ...
                         '    オーバーランしやすくなります。想定サイズも\n', ...
                         '    %.2f GB に変わります。\n'], ...
                    captureDuration * sampleRate * bytesPerSample / 1e9);
            end
        end

        % [I0;Q0], [I1;Q1], ... の 2 x N 行列にする。fwrite は列優先で
        % 書き出すので、これで I,Q が交互に並んだべた書きになる。
        v = iqData(1:dataLen);
        writeTic = tic;
        fwrite(fidBin, [real(v).'; imag(v).'], rawPrecision);
        wSec = toc(writeTic);
        writeSecTotal = writeSecTotal + wSec;
        if wSec > writeSecMax
            writeSecMax = wSec;
        end

        totalSamplesCaptured = totalSamplesCaptured + dataLen;

        % 進捗表示 (1秒ごと)
        if totalSamplesCaptured >= nextReportSample
            elapsedNow = toc(captureTic);
            fprintf('  %5.1f / %.0f s   オーバーラン %d 回   書込負荷 %4.1f%%\n', ...
                totalSamplesCaptured / sampleRate, captureDuration, ...
                overrunCount, writeSecTotal / max(elapsedNow, eps) * 100);
            nextReportSample = nextReportSample + sampleRate;
        end
    end
catch captureErr
    try
        fclose(fidBin);
    catch
    end
    try
        release(rx);
    catch
    end
    rethrow(captureErr);
end

elapsedCapture = toc(captureTic);
release(rx);
fclose(fidBin);

if isempty(rawPrecision)
    error('captureIQ_long:noSamples', ...
        ['1フレームも受信できませんでした。USRP の接続と設定を確認してください。\n', ...
         '生成された %s は空です。'], binFile);
end

fprintf('\nキャプチャ完了: %d サンプル, %.2f s\n', ...
    totalSamplesCaptured, elapsedCapture);
fprintf('  オーバーラン    : %d 回\n', overrunCount);
fprintf('  書き込み総時間  : %.2f s (キャプチャ時間の %.1f%%)\n', ...
    writeSecTotal, writeSecTotal / max(elapsedCapture, eps) * 100);
fprintf('  最大書き込み待ち: %.3f s (1回あたり)\n', writeSecMax);

% 書き込みが受信に追いつけていたかの判定
writeLoadPct = writeSecTotal / max(elapsedCapture, eps) * 100;
if overrunCount > 0
    fprintf(['  ※オーバーランが発生しました。サンプルの取りこぼしがあるため、\n', ...
             '    時刻軸に不連続が含まれます。書込負荷が高い場合は保存先を\n', ...
             '    SSD にする、他の負荷の高いアプリを終了する等をお試しください。\n']);
elseif writeLoadPct > 70
    fprintf(['  ※書込負荷が高めです (%.1f%%)。今回はオーバーランしていませんが、\n', ...
             '    余裕が少ないため保存先を SSD にすることを検討してください。\n'], ...
             writeLoadPct);
end

%% ------------------------------------------------------------------------
%  5. メタデータの保存
%  ------------------------------------------------------------------------
% 生値を complex double へ戻すときの倍率 (型判定時に決めている)。
% USRP の 'double' 出力はおおよそ ±1 に正規化されているので、それに合わせる。
% 定数倍は CSI の相対値や復号結果には影響しないが、振幅の絶対値は
% captureIQ.m で取ったデータとわずかに異なり得る点に注意。
meta = struct();
meta.description      = 'USRP B205 mini-i captured Wi-Fi IQ samples (5GHz ch36), raw interleaved stream';
meta.dataFormat       = sprintf('%s interleaved I,Q in .bin (I0,Q0,I1,Q1,...)', rawPrecision);
meta.binFile          = binFile;
meta.rawPrecision     = rawPrecision;
meta.rawScaleFactor   = rawScaleFactor;
meta.bytesPerSample   = bytesPerSample;
meta.wifiChannel      = 36;
meta.centerFrequency  = centerFrequency;        % [Hz]
meta.sampleRate       = sampleRate;             % [Sps] = 帯域幅
meta.bandwidth        = 20e6;                   % [Hz]
meta.gain             = gain;                   % [dB]
meta.platform         = usrpPlatform;
meta.serialNum        = usrpSerialNum;
meta.captureDuration  = captureDuration;        % [s]
meta.samplesPerFrame  = samplesPerFrame;
meta.totalSamples     = totalSamplesCaptured;
meta.overrunCount     = overrunCount;
meta.elapsedCapture   = elapsedCapture;         % [s]
meta.writeSecTotal    = writeSecTotal;          % [s]
meta.writeSecMax      = writeSecMax;            % [s]
meta.captureDatetime  = timestamp;              % 'yyyymmddHHMM'
meta.matlabVersion    = version;

save(metaFile, 'meta');
fprintf('\nメタデータを保存しました: %s\n', metaFile);

%% ------------------------------------------------------------------------
%  6. セグメント分割 (既存の decodeIQ_*.m が読める形式へ変換)
%  ------------------------------------------------------------------------
% .bin を segmentDuration ごとに読み出し、captureIQ.m と同じ変数構成
% (complex double の iq + meta) の *_raw.mat として書き出す。
% ここはメモリに1セグメント分しか載せないので、長時間でも破綻しない。
if writeSegments
    segSamples = round(segmentDuration * sampleRate);
    numSegs    = ceil(totalSamplesCaptured / segSamples);

    fprintf('\n[分割] %d 個のセグメントに分割します (1個 %.1f s)...\n', ...
        numSegs, segmentDuration);
    splitTic = tic;

    fidIn = fopen(binFile, 'r');
    if fidIn == -1
        error('captureIQ_long:binOpenFailedForRead', ...
            '生データを開けませんでした: %s', binFile);
    end

    segFiles = cell(numSegs, 1);
    try
        for s = 1:numSegs
            startSample = (s - 1) * segSamples;
            nRead = min(segSamples, totalSamplesCaptured - startSample);

            % 2 x nRead として読む (列が [I;Q] の組)。書き出したときと
            % 同じ精度を指定する。
            raw = fread(fidIn, [2, nRead], ['*' rawPrecision]);
            if isempty(raw)
                break;
            end
            nActual = size(raw, 2);

            % complex double へ変換 (WLAN Toolbox は double 前提)。
            % 実部・虚部を順に作って raw を早めに解放し、ピークを抑える。
            re = double(raw(1, :)).' * rawScaleFactor;
            im = double(raw(2, :)).' * rawScaleFactor;
            clear raw;
            iq = complex(re, im); %#ok<NASGU>
            clear re im;

            % セグメント固有のメタデータ
            segMeta = meta;
            segMeta.dataFormat          = 'complex double column vector, variable name: iq';
            segMeta.segmentIndex        = s;
            segMeta.segmentCount        = numSegs;
            segMeta.segmentStartSample  = startSample;
            segMeta.segmentStartTimeSec = startSample / sampleRate;
            segMeta.totalSamples        = nActual;
            segMeta.captureDuration     = nActual / sampleRate;
            segMeta.sourceBinFile       = binFile;
            meta_backup = meta;             %#ok<NASGU>
            meta = segMeta;                 %#ok<NASGU>

            segFiles{s} = fullfile(hddSavePath, ...
                sprintf('%s_seg%02d_raw.mat', timestamp, s));

            % IQ は非圧縮で保存する。ランダムに近いデータなので圧縮は
            % 効かないうえ、数GBでは圧縮処理に時間を取られるだけになる。
            try
                save(segFiles{s}, 'iq', 'meta', '-v7.3', '-nocompression');
            catch
                % 古い MATLAB では -nocompression が使えない
                save(segFiles{s}, 'iq', 'meta', '-v7.3');
            end
            meta = meta_backup;
            clear iq;

            fprintf('  seg%02d/%02d: %.1f s 分 (開始 %.1f s) -> %s\n', ...
                s, numSegs, nActual / sampleRate, startSample / sampleRate, ...
                segFiles{s});
        end
    catch splitErr
        fclose(fidIn);
        rethrow(splitErr);
    end
    fclose(fidIn);

    fprintf('[分割] 完了 (%.1f s)\n', toc(splitTic));
    fprintf(['\n次に decodeIQ_VHT.m / decode_VHT_v2.m 等で各セグメントを\n', ...
             '復号してください。inputRawFile にセグメントのパスを指定します。\n', ...
             '複数セグメントの CSI を時系列で繋ぐ場合は、各セグメントの\n', ...
             'meta.segmentStartTimeSec を timeSec に足してください。\n']);
else
    fprintf(['\n生データ (.bin) のみ保存しました。decodeIQ_*.m は *_raw.mat を\n', ...
             '前提としているため、そのままでは読めません。writeSegments を\n', ...
             'true にして再実行するか、別途変換してください。\n']);
end

fprintf('\nすべての処理が完了しました。\n');

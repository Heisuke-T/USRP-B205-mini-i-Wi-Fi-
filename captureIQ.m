%% captureIQ.m
% =========================================================================
%  USRP B205 mini-i を用いた Wi-Fi トラフィックの IQ 信号取得スクリプト
% -------------------------------------------------------------------------
%  概要:
%    Ettus Research USRP B205 mini-i を SDR 受信機として使用し、
%    Wi-Fi (IEEE 802.11) が使用する周波数帯の複素ベースバンド (IQ) 信号を
%    受信して、USB 外部ストレージにバイナリ形式で保存します。
%
%  必要環境:
%    - MATLAB
%    - Communications Toolbox
%    - Communications Toolbox Support Package for USRP Radio
%      (comm.SDRuReceiver / findsdru が利用可能であること)
%    - USB 3.0 接続された USRP B205 mini-i
%    - 十分な空き容量のある USB 外部ストレージ
%
%  出力ファイル:
%    <保存先>/<プレフィックス>_<日時>.bin  … IQ 生データ (complex, interleaved)
%    <保存先>/<プレフィックス>_<日時>.mat  … 取得条件などのメタデータ
%
%  データ形式 (.bin):
%    I0, Q0, I1, Q1, ... の順にインターリーブされた float32 (single) 実数列。
%    後で読み込む場合は本ファイル末尾の loadIQ ローカル関数を参照。
%
%  注意:
%    電波の受信・記録は、利用地域の電波法および関連法令を遵守し、
%    自身が管理する機器・許可された環境でのみ実施してください。
% =========================================================================

clear; clc;

%% ------------------------------------------------------------------------
%  1. ユーザ設定パラメータ
%  ------------------------------------------------------------------------

% --- 保存先 (USB 外部ストレージ) -----------------------------------------
% Windows の例: 'E:\iq_capture'
% Linux   の例: '/media/username/USBDRIVE/iq_capture'
% macOS   の例: '/Volumes/USBDRIVE/iq_capture'
usbSavePath = fullfile('E:', 'iq_capture');   % ★環境に合わせて変更してください

filePrefix  = 'wifi_iq';                       % 出力ファイル名のプレフィックス

% --- Wi-Fi 受信パラメータ -----------------------------------------------
% Wi-Fi チャンネルの中心周波数 [Hz]
%   2.4GHz 帯:  ch1=2.412e9, ch6=2.437e9, ch11=2.462e9
%   5GHz   帯:  ch36=5.180e9, ch40=5.200e9, ch44=5.220e9, ch48=5.240e9
centerFrequency = 2.412e9;      % [Hz] 例: 2.4GHz 帯 ch1

% サンプリングレート [Sps] = 受信帯域幅
%   20MHz 幅の Wi-Fi チャンネルを取り込むには 20e6 以上を推奨。
%   B205 mini-i は USB3.0 で最大 ~61.44 MSps (複素) 程度。
sampleRate      = 20e6;         % [Sps] 20 MHz 帯域

% 受信ゲイン [dB] (B200 系は 0〜76 dB 程度)
%   信号が飽和する場合は下げ、弱すぎる場合は上げる。
gain            = 40;           % [dB]

% キャプチャ時間 [s]
captureDuration = 2.0;          % [s]

% 1 回の受信で取得するサンプル数 (フレーム長)
%   大きいほどオーバーフローが起きにくいが遅延・メモリは増える。
samplesPerFrame = 20000;        % [samples/frame]

% USRP プラットフォーム名 (B205 mini-i は B200 系列)
usrpPlatform    = 'B200';

% USRP のシリアル番号 ('' で最初に見つかった機器を使用)
usrpSerialNum   = '';

%% ------------------------------------------------------------------------
%  2. 保存先の準備
%  ------------------------------------------------------------------------
if ~exist(usbSavePath, 'dir')
    fprintf('保存先フォルダが存在しないため作成します: %s\n', usbSavePath);
    [ok, msg] = mkdir(usbSavePath);
    if ~ok
        error('captureIQ:mkdirFailed', ...
            'USB 保存先フォルダを作成できませんでした (%s): %s', usbSavePath, msg);
    end
end

% USB が実際に書き込み可能か簡易確認
testFile = fullfile(usbSavePath, '.write_test.tmp');
fidTest  = fopen(testFile, 'w');
if fidTest == -1
    error('captureIQ:usbNotWritable', ...
        'USB 保存先に書き込みできません。ドライブの接続・パス・空き容量を確認してください: %s', ...
        usbSavePath);
end
fclose(fidTest);
delete(testFile);

% タイムスタンプ付きの出力ファイル名を生成
timestamp   = datestr(now, 'yyyymmdd_HHMMSS');
baseName    = sprintf('%s_%s', filePrefix, timestamp);
binFileName = fullfile(usbSavePath, [baseName '.bin']);
matFileName = fullfile(usbSavePath, [baseName '.mat']);

%% ------------------------------------------------------------------------
%  3. USRP B205 mini-i 受信機オブジェクトの生成
%  ------------------------------------------------------------------------
% 接続確認 (任意): findsdru で機器情報を取得
try
    radioInfo = findsdru();
    if isempty(radioInfo)
        warning('captureIQ:noRadio', ...
            'USRP 機器が検出されませんでした。USB 接続と電源を確認してください。');
    else
        fprintf('検出された USRP 機器:\n');
        for k = 1:numel(radioInfo)
            fprintf('  Platform=%s, SerialNum=%s, Status=%s\n', ...
                radioInfo(k).Platform, radioInfo(k).SerialNum, radioInfo(k).Status);
        end
    end
catch ME
    warning('captureIQ:findsdruFailed', ...
        'findsdru の実行に失敗しました: %s', ME.message);
end

% comm.SDRuReceiver の構築
rxArgs = { ...
    'Platform',            usrpPlatform, ...
    'CenterFrequency',     centerFrequency, ...
    'Gain',                gain, ...
    'MasterClockRate',     sampleRate * 2, ...   % B200: MCR は 5MHz〜61.44MHz
    'DecimationFactor',    2, ...                 % 出力レート = MCR / Decimation
    'OutputDataType',      'single', ...          % 複素 single で出力
    'SamplesPerFrame',     samplesPerFrame };

% シリアル番号が指定されていれば追加
if ~isempty(usrpSerialNum)
    rxArgs = [rxArgs, {'SerialNum', usrpSerialNum}];
end

rx = comm.SDRuReceiver(rxArgs{:});

% 実際の出力サンプルレートを確認 (= MasterClockRate / DecimationFactor)
actualSampleRate = sampleRate;
fprintf('\n受信設定:\n');
fprintf('  中心周波数      : %.4f GHz\n', centerFrequency / 1e9);
fprintf('  サンプルレート  : %.3f MSps (帯域幅)\n', actualSampleRate / 1e6);
fprintf('  ゲイン          : %d dB\n', gain);
fprintf('  キャプチャ時間  : %.2f s\n', captureDuration);
fprintf('  保存先(.bin)    : %s\n', binFileName);

%% ------------------------------------------------------------------------
%  4. IQ 信号のキャプチャと USB への逐次保存
%  ------------------------------------------------------------------------
% メモリ枯渇を避けるため、受信フレームごとにバイナリファイルへ追記書き込み。

totalSamplesTarget = round(captureDuration * actualSampleRate);
totalSamplesSaved  = 0;
overrunCount       = 0;

fidBin = fopen(binFileName, 'w');
if fidBin == -1
    release(rx);
    error('captureIQ:openBinFailed', ...
        '出力バイナリファイルを開けませんでした: %s', binFileName);
end

fprintf('\nキャプチャを開始します...\n');
captureTic = tic;

% cleanup: エラー時でも必ずファイルと機器を解放する
cleanupObj = onCleanup(@() cleanupResources(fidBin, rx));

while totalSamplesSaved < totalSamplesTarget
    % USRP から 1 フレーム受信
    [iqData, dataLen, overrun] = rx();

    if dataLen == 0
        % 有効データが無いフレームはスキップ
        continue;
    end

    if overrun
        overrunCount = overrunCount + 1;   % ホスト側処理が追いつかず取りこぼし
    end

    % complex single を [I Q I Q ...] の実数列(float32)に変換して書き込み
    interleaved = zeros(2 * dataLen, 1, 'single');
    interleaved(1:2:end) = real(iqData(1:dataLen));
    interleaved(2:2:end) = imag(iqData(1:dataLen));
    fwrite(fidBin, interleaved, 'single');

    totalSamplesSaved = totalSamplesSaved + dataLen;
end

elapsed = toc(captureTic);

% ファイル・機器を明示的に解放 (onCleanup と二重でも安全)
fclose(fidBin);
release(rx);
clear cleanupObj;   % onCleanup を無効化 (既に解放済みのため)

fprintf('キャプチャ完了。\n');
fprintf('  取得サンプル数  : %d\n', totalSamplesSaved);
fprintf('  経過時間        : %.2f s\n', elapsed);
fprintf('  オーバーラン回数: %d\n', overrunCount);
if overrunCount > 0
    fprintf('  ※オーバーランが発生しました。samplesPerFrame を増やすか、\n');
    fprintf('    sampleRate を下げる、USB3.0 ポートを使う等を検討してください。\n');
end

%% ------------------------------------------------------------------------
%  5. メタデータの保存
%  ------------------------------------------------------------------------
meta = struct();
meta.description      = 'USRP B205 mini-i captured Wi-Fi IQ samples';
meta.binFileName      = binFileName;
meta.dataFormat       = 'interleaved float32 (single): I0,Q0,I1,Q1,...';
meta.centerFrequency  = centerFrequency;      % [Hz]
meta.sampleRate       = actualSampleRate;     % [Sps]
meta.gain             = gain;                 % [dB]
meta.platform         = usrpPlatform;
meta.serialNum        = usrpSerialNum;
meta.captureDuration  = captureDuration;      % [s]
meta.samplesPerFrame  = samplesPerFrame;
meta.totalSamplesSaved= totalSamplesSaved;
meta.overrunCount     = overrunCount;
meta.captureDatetime  = timestamp;
meta.matlabVersion    = version;

save(matFileName, 'meta');
fprintf('\nメタデータを保存しました: %s\n', matFileName);
fprintf('すべての処理が完了しました。\n');

%% ------------------------------------------------------------------------
%  ローカル関数
%  ------------------------------------------------------------------------
function cleanupResources(fidBin, rx)
    % エラー発生時などに呼ばれるクリーンアップ処理
    try
        if ~isempty(fidBin) && fidBin ~= -1
            fs = fopen('all');
            if any(fs == fidBin)
                fclose(fidBin);
            end
        end
    catch
    end
    try
        if ~isempty(rx) && isvalid(rx)
            release(rx);
        end
    catch
    end
end

%% ------------------------------------------------------------------------
%  参考: 保存した IQ データを読み込む関数
%  ------------------------------------------------------------------------
%  使い方:
%     iq = loadIQ('E:\iq_capture\wifi_iq_20260714_120000.bin');
%  以下をコピーして別ファイル loadIQ.m として保存すると単体で使えます。
% -------------------------------------------------------------------------
function iq = loadIQ(binFile) %#ok<DEFNU>
    fid = fopen(binFile, 'r');
    if fid == -1
        error('loadIQ:openFailed', 'ファイルを開けません: %s', binFile);
    end
    raw = fread(fid, Inf, 'single=>single');
    fclose(fid);
    % [I Q I Q ...] を complex に復元
    iq = complex(raw(1:2:end), raw(2:2:end));
end

% captureIQ.m
% =========================================================================
%  USRP B205 mini-i を用いた Wi-Fi トラフィックの IQ 信号取得スクリプト
% -------------------------------------------------------------------------
%  概要:
%    Ettus Research USRP B205 mini-i を SDR 受信機として使用し、
%    Wi-Fi (IEEE 802.11) の 5GHz 帯 チャネル36 (中心 5.180 GHz, 帯域 20 MHz)
%    の複素ベースバンド (IQ) 信号を受信して、USB 外部ストレージに
%    .mat 形式で保存します。
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
%    <保存先>/yyyymmddcccc.mat
%      yyyy=年, mmdd=月日, cccc=時刻(HHMM)。 例: 202607141822.mat
%
%  .mat の内容:
%    iq   … 受信 IQ サンプル (complex single 列ベクトル)
%    meta … 中心周波数・サンプルレート等の取得条件 (構造体)
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
usbSavePath = 'D:\IQ';


% --- Wi-Fi 受信パラメータ -----------------------------------------------
% Wi-Fi チャンネルの中心周波数 [Hz]
%   2.4GHz 帯:  ch1=2.412e9, ch6=2.437e9, ch11=2.462e9
%   5GHz   帯:  ch36=5.180e9, ch40=5.200e9, ch44=5.220e9, ch48=5.240e9
centerFrequency = 5.180e9;      % [Hz] 5GHz 帯 ch36 (20MHz 帯域幅)

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
usrpSerialNum   = '3240497';


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

% 出力ファイル名を生成
%   形式: yyyymmddcccc.mat  (yyyy=年, mmdd=月日, cccc=時刻HHMM)
%   例: 2026年07月14日 18:22 に取得 -> 202607141822.mat
timestamp   = datestr(now, 'yyyymmddHHMM');   % 例: '202607141822'
baseName    = timestamp;
matFileName = fullfile(usbSavePath, [baseName '.mat']);

%% ------------------------------------------------------------------------
%  3. USRP B205 mini-i 受信機オブジェクトの生成
%  ------------------------------------------------------------------------
% findsdru で機器情報を取得 (B200 系は SerialNum を明示指定しないと
% comm.SDRuReceiver の setupImpl でエラーになるため、ここで自動取得する)
radioInfo = [];
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

% usrpSerialNum が未指定の場合、findsdru の検出結果 (Status=Success かつ
% usrpPlatform に一致する機器) から自動的に補完する。
if isempty(usrpSerialNum) && ~isempty(radioInfo)
    isMatch = strcmp({radioInfo.Platform}, usrpPlatform) & ...
              strcmp({radioInfo.Status}, 'Success');
    matched = radioInfo(isMatch);
    if numel(matched) == 1
        usrpSerialNum = matched(1).SerialNum;
        fprintf('SerialNum を自動検出しました: %s\n', usrpSerialNum);
    elseif numel(matched) > 1
        error('captureIQ:multipleRadios', ...
            ['%s の機器が複数検出されました。usrpSerialNum に使用する ', ...
             'シリアル番号を明示的に指定してください。'], usrpPlatform);
    end
end

if isempty(usrpSerialNum)
    error('captureIQ:noSerialNum', ...
        ['USRP のシリアル番号を自動検出できませんでした。findsdru の表示を確認し、', ...
         'usrpSerialNum に文字列で指定してください (例: ''3240497'')。']);
end

% comm.SDRuReceiver の構築
rxArgs = { ...
    'Platform',            usrpPlatform, ...
    'SerialNum',           usrpSerialNum, ...
    'CenterFrequency',     centerFrequency, ...
    'Gain',                gain, ...
    'MasterClockRate',     sampleRate * 2, ...   % B200: MCR は 5MHz〜61.44MHz
    'DecimationFactor',    2, ...                 % 出力レート = MCR / Decimation
    'OutputDataType',      'single', ...          % 複素 single で出力
    'SamplesPerFrame',     samplesPerFrame };

rx = comm.SDRuReceiver(rxArgs{:});

% 実際の出力サンプルレートを確認 (= MasterClockRate / DecimationFactor)
actualSampleRate = sampleRate;
fprintf('\n受信設定:\n');
fprintf('  中心周波数      : %.4f GHz\n', centerFrequency / 1e9);
fprintf('  サンプルレート  : %.3f MSps (帯域幅)\n', actualSampleRate / 1e6);
fprintf('  ゲイン          : %d dB\n', gain);
fprintf('  キャプチャ時間  : %.2f s\n', captureDuration);
fprintf('  保存先(.mat)    : %s\n', matFileName);

%% ------------------------------------------------------------------------
%  4. IQ 信号のキャプチャ
%  ------------------------------------------------------------------------
% 取得した複素 IQ サンプルをメモリ上のバッファに蓄積し、
% 終了後に .mat ファイルへまとめて保存する。

totalSamplesTarget = round(captureDuration * actualSampleRate);
totalSamplesSaved  = 0;
overrunCount       = 0;

% 出力バッファを事前確保 (complex single)。少し余裕を持たせる。
iqBuffer = complex(zeros(totalSamplesTarget + samplesPerFrame, 1, 'single'));

fprintf('\nキャプチャを開始します...\n');
captureTic = tic;

% cleanup: エラー時でも必ず機器を解放する
cleanupObj = onCleanup(@() cleanupResources(rx));

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

    % バッファへ格納
    idx = totalSamplesSaved + (1:dataLen);
    iqBuffer(idx) = iqData(1:dataLen);

    totalSamplesSaved = totalSamplesSaved + dataLen;
end

elapsed = toc(captureTic);

% 機器を明示的に解放 (onCleanup と二重でも安全)
release(rx);
clear cleanupObj;   % onCleanup を無効化 (既に解放済みのため)

% 目標サンプル数ちょうどに切り詰め
iq = iqBuffer(1:totalSamplesSaved);
clear iqBuffer;

fprintf('キャプチャ完了。\n');
fprintf('  取得サンプル数  : %d\n', totalSamplesSaved);
fprintf('  経過時間        : %.2f s\n', elapsed);
fprintf('  オーバーラン回数: %d\n', overrunCount);
if overrunCount > 0
    fprintf('  ※オーバーランが発生しました。samplesPerFrame を増やすか、\n');
    fprintf('    sampleRate を下げる、USB3.0 ポートを使う等を検討してください。\n');
end

%% ------------------------------------------------------------------------
%  5. IQ 信号とメタデータを .mat ファイルへ保存 (USB 外部ストレージ)
%  ------------------------------------------------------------------------
% 変数 iq  : complex single 列ベクトル (受信 IQ サンプル)
% 変数 meta: 取得条件などのメタデータ構造体
meta = struct();
meta.description      = 'USRP B205 mini-i captured Wi-Fi IQ samples (5GHz ch36)';
meta.matFileName      = matFileName;
meta.dataFormat       = 'complex single column vector, variable name: iq';
meta.wifiChannel      = 36;
meta.centerFrequency  = centerFrequency;      % [Hz]
meta.sampleRate       = actualSampleRate;     % [Sps] = 帯域幅
meta.bandwidth        = 20e6;                 % [Hz]
meta.gain             = gain;                 % [dB]
meta.platform         = usrpPlatform;
meta.serialNum        = usrpSerialNum;
meta.captureDuration  = captureDuration;      % [s]
meta.samplesPerFrame  = samplesPerFrame;
meta.totalSamplesSaved= totalSamplesSaved;
meta.overrunCount     = overrunCount;
meta.captureDatetime  = timestamp;            % 'yyyymmddHHMM'
meta.matlabVersion    = version;

% IQ が大きい場合に備えて -v7.3 (HDF5 ベース, 2GB 超対応) で保存
save(matFileName, 'iq', 'meta', '-v7.3');
fprintf('\nIQ 信号を保存しました: %s\n', matFileName);
fprintf('すべての処理が完了しました。\n');

%% ------------------------------------------------------------------------
%  ローカル関数
%  ------------------------------------------------------------------------
function cleanupResources(rx)
    % エラー発生時などに呼ばれるクリーンアップ処理
    try
        if ~isempty(rx) && isvalid(rx)
            release(rx);
        end
    catch
    end
end

%% ------------------------------------------------------------------------
%  参考: 保存した IQ データを読み込む例
%  ------------------------------------------------------------------------
%  .mat には変数 iq (complex single) と meta が保存されています。
%     S  = load('E:\iq_capture\202607141822.mat');
%     iq = S.iq;          % 受信 IQ サンプル
%     fs = S.meta.sampleRate;
%     % 例: スペクトログラム表示
%     % spectrogram(iq, 1024, 512, 1024, fs, 'centered', 'yaxis');
% -------------------------------------------------------------------------

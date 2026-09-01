%% decodeIQ_VHT_Pico.m
% =========================================================================
%  [第2段] PicoScenes などが注入したトラフィックの CSI を記録する
% -------------------------------------------------------------------------
%  decode_VHT_v2.m (高速版) をベースに、SSID ではなく「送信元アドレス /
%  PHYパラメータ」でパケットを選別する版。
%
%  なぜ SSID が使えないか:
%    decodeIQ_VHT.m は Beacon / Probe Response に含まれる SSID 情報要素から
%    「SSID ↔ BSSID」の対応を学習し、その BSSID のパケットを抽出していた。
%    PicoScenes の injector モードは AP として動作せず Beacon を出さないため、
%    この学習ができない。
%
%  代わりに使える手がかり:
%    [A] 送信元 MAC アドレス (Address2)
%        注入源は同じアドレスを繰り返し使うため、パケット数が突出した
%        1つのアドレスとして現れる。既知APは Beacon から SSID が分かるので、
%        「SSID が付いていないのに大量に送っているアドレス」が注入源。
%    [B] PHY パラメータ
%        例: --preset TX_CBW_20_VHT --mcs 3 --txcm 1
%        → VHT / 20MHz / MCS3 / 1ストリーム に固定される。周囲の AP
%          トラフィックは MCS が揃わないので、MCS で絞ると効果が高い。
%    [C] 送信間隔
%        例: --delay 5780 → 5780us 周期 (約173 packets/s)。
%        選別後のパケット間隔の中央値がこれと一致すれば、対象が正しいと
%        確認できる (expectedIntervalUs で照合し診断表示する)。
%
%  想定している送信側の設定例:
%    array_prepare_for_picoscenes 1 "5180 HT20"
%    PicoScenes "-d debug -i 1 --mode injector --preset TX_CBW_20_VHT \
%                --repeat 1e5 --delay 5780 --mcs 3 --txcm 1"
%      5180 = ch36 (captureIQ.m の centerFrequency と一致させること)
%      txcm 1 = 送信チェーン1本 → NSS=1 なので SISO 受信で復号可能
%      mcs 3  = 16-QAM R=1/2。MCS8(256QAM) より遥かに頑健で復号しやすい
%
%  使い方の流れ:
%    1. まず targetTxAddress = '' のまま実行する。
%       「送信元アドレス一覧」に全送信元が件数つきで表示される。
%    2. 注入源と思われるアドレスを確認する。自動選択の結果もそこに出るので、
%       妥当ならそのままでよい。違っていれば targetTxAddress に明示する。
%
%  MAC が読めない注入パケットの扱い (keepPHYOnlyCSI):
%    VHT のデータ部は、プリアンブルや SIG フィールドより遥かに高い受信品質を
%    要求する。例えば --mcs 3 は 16QAM R=1/2 であり、BPSK R=1/2 の
%    L-SIG / VHT-SIG-A / VHT-SIG-B より約 10dB 高い SNR が要る。そのため
%
%      VHT として検出 857 / SIG-A CRC 通過 854 / 送信帯域も NSTS も正しく読める
%      → しかし MPDU の FCS は 1 件も通らない
%
%    という状態が普通に起こる (注入源が近すぎて ADC が飽和している、逆に
%    遠すぎて SNR が足りない、など)。送信元 MAC はこの MPDU の中にあるため、
%    MAC アドレスでの選別 ([A]) はこの状態では機能しない。
%
%    一方、本スクリプトが欲しい CSI は VHT-LTF (プリアンブル) から求めており、
%    MPDU 復号の成否とは無関係に正しく得られている。そこで
%    keepPHYOnlyCSI = true (既定) では、MAC を復号できなかったパケットも
%      擬似アドレス '(PHY:VHT-MCS<n>)'
%    の送信元としてまとめて記録し、手掛かり [B] PHY パラメータ と
%    [C] 送信間隔 だけで注入トラフィックを選別できるようにしている。
%    送信間隔の中央値が --delay と一致すれば、それが注入源である裏付けになる。
%
%    なぜ FCS が通らないのかは [VHT データ部の品質診断] を見ると分かる。
%      振幅ピークが 1.0 付近      → ADC 飽和。captureIQ.m の gain を下げる
%      SNR は高いが EVM が悪い    → 歪み (飽和・非線形・隣接波干渉)
%      SNR も EVM も悪い          → 受信品質不足。距離/gain/--mcs を見直す
%
%  ベースからの変更点はパケットの選別方法と、MAC を復号できなかった
%  パケットの CSI を残すかどうか (keepPHYOnlyCSI) のみ。パケット検出・同期・
%  CSI 推定・MAC 復号の処理自体は decode_VHT_v2.m と同一で、出力する .mat の
%  変数名も同じなので ResultCSI.m はそのまま使える。
%
%  高速化の内容:
%    [1] CFO 補正範囲の限定 (支配的な改善。約800倍)
%        旧版は精CFO補正で iq(pktStart+1:end) と「バッファの残り全体」を
%        切り出して複素指数を掛けていた。5秒キャプチャでは1パケットあたり
%        最大1億サンプルの exp() を、しかも粗補正・精補正で2回、検出パケット
%        1件ごとに実行していた。実際に使うのは高々パケット長分だけである。
%        本版では 802.11 の最大 PPDU 長 (5.484ms = 109,680サンプル) を上限に
%        切り出す。さらに粗CFO と精CFO を合算して1回で適用する
%        (exp(-jθc)·exp(-jθf) = exp(-j(θc+θf)) なので結果は同一)。
%
%        旧版の所要時間が「パケット数 x バッファ長」に比例していた実測:
%          0.5s/1249パケット →   165 s
%          1.0s/2130パケット →   569 s
%          5.0s/7968パケット → 12000 s   (キャプチャ長に対しほぼ二乗で悪化)
%
%    [2] 復号失敗時もパケット全長をスキップ
%        旧版は復号に失敗すると探索位置を 560 サンプル(28us)しか進めず、
%        同じパケットの本体を何度も再走査していた。OFDMデータ部は雑音に
%        近い統計を持つため、そこで偽のプリアンブル検出が発生する
%        (旧版の実測では HT-Greenfield 2969件・HT 309件、検出全体の41%)。
%        本版は L-SIG から PPDU 全長を先に求め、以降どの段階で失敗しても
%        その分だけ探索位置を進める。
%
%    [3] パケット検出窓の適応化
%        wlanPacketDetect は最初の1件しか返さないため、窓が大きいほど
%        「見つけた位置より後ろ」を無駄に走査する。密なら小さい窓、
%        見つからなければ窓を倍々に広げる方式にした。
%
%    [4] LDPC の早期終了
%        パリティ検査が全て通った時点で反復を打ち切る。既に有効な符号語に
%        なっているため、追加の反復は結果を変えない (出力はビット単位で同一)。
%        復号方式 (既定のベリーフ伝搬) は結果が変わり得るため変更しない。
%
%    [5] 細部
%        - warning の抑止をループ外で一度だけ行う (状態の保存・復元は
%          呼び出しコストが高く、パケットごとだと数万回に達する)
%        - pktLog を事前確保 (逐次成長による O(n^2) のコピーを回避)
%        - 既定でパケット毎の逐次表示を止める (verbosePackets)
% -------------------------------------------------------------------------
%  概要:
%    captureIQ.m が HDD に保存した生 IQ を読み込み、WLAN Toolbox で
%    パケットを検出・復号して、
%      1) 復号できた全パケットについて、プリアンブル (Non-HTは L-LTF、
%         HTは HT-LTF、VHTは VHT-LTF) から CSI (チャネル応答 H(k)) を算出
%      2) 送信元アドレスごとの一覧を作り、注入源を特定
%      3) その送信元のパケットのCSIだけを抽出
%    して .mat 保存する。
%
%    Beacon/Probe Response が受かれば SSID も学習し、一覧に併記する
%    (注入源と周囲の AP を見分けるための情報として使う)。
%
%    受信と独立しているため、条件 (targetTxAddress, pktDetThreshold 等) を
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
%      macDecoded   … [パケット数 x 1] false は MAC を全く復号できず、PHY
%                      パラメータ (フォーマット/MCS/帯域/NSTS) と送信間隔
%                      だけで注入トラフィックと判断したもの。CSI は有効。
%      targetTxAddressOut … 実際に選別に使った送信元アドレス
%      txCensus     … 送信元アドレスごとの集計 (診断用。下記フィールド)
%                       .address .count .nNonHT .nHT .nVHT .ssid
%                       .mcsList .medianIntervalUs
%      seenNetworks … Beacon から判明した BSSID/SSID の一覧 (診断用)
%      targetSSIDOut / targetBSSIDOut … 旧形式との互換のため空文字で出力
%
%    ※ 帰属判定は送信元(Address2)・宛先(Address1)のどちらかが対象
%      アドレスと一致するかで行う。注入フレームは通常 Address2 が注入源
%      になるが、応答(ACK等)を拾いたい場合に備えて両方を見ている。
%
%  decode_VHT_v2.m との比較:
%    パケットの選別方法以外は同一なので、同じ *_raw.mat に対して両方を
%    実行すれば、選別前の復号結果 (検出数・FCS検証数) は一致するはず。
%    ただし keepPHYOnlyCSI = true のときは、decode_VHT_v2.m が捨てている
%    「MAC を復号できなかった VHT パケット」が本版では記録されるため、
%    記録件数と csiVHT の行数は一致しない (増える)。厳密に比較したい
%    場合は keepPHYOnlyCSI = false にすること。
%    出力ファイル名が同じになるので、片方は別フォルダへ保存するか、
%    実行後にファイル名を変えておく。
%
%      A = load('..._CSI.mat');   % decodeIQ_VHT.m の出力
%      B = load('..._CSI.mat');   % decode_VHT_v2.m の出力
%      isequal(A.csiVHT,     B.csiVHT)       % CSI が完全一致するか
%      isequal(A.csiNonHT,   B.csiNonHT)
%      isequal(A.timeSecVHT, B.timeSecVHT)   % 検出位置も一致するか
%
%    ※[2] の「失敗時に全長スキップ」により、旧版が同一パケット内で
%      重複検出していた偽パケットは本版では現れない。そのため
%      csiMeta.decodeStats の検出数・HT-Greenfield 数などは一致しない。
%      一致すべきなのは「正しく復号できたパケットの CSI」である。
%
%  処理時間の目安:
%    旧版は混雑環境での 5 秒キャプチャに約 3 時間20分を要した。
%    本版は数分程度を見込む (実測したら本コメントを更新すること)。
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

% --- 抽出したいトラフィックの指定 ----------------------------------------
% PicoScenes の injector は Beacon を出さないため SSID では絞れない。
% 代わりに以下で選別する。

% [A] 送信元 MAC アドレス (12桁の16進、区切り文字なし。例: '0016EA123456')
%     '' にすると自動選択する。まずは '' のまま実行し、表示される
%     「送信元アドレス一覧」を見てから必要に応じて明示指定する。
targetTxAddress = '';

% [B] 対象とする PHY フォーマット。自動選択の候補もこの中から選ぶ。
%     PicoScenes の TX_CBW_20_VHT なら {'VHT'}。
%     絞りたくない場合は {'Non-HT','HT','VHT'} とする。
targetFormats = {'VHT'};

% [B'] 対象とする MCS。[] なら絞らない。
%      PicoScenes の --mcs 3 に合わせると周囲の AP トラフィックを
%      かなり除去できる。自動選択の候補もこれで絞られる。
targetMCS = 3;

% [C] 期待する送信間隔 [us]。PicoScenes の --delay に相当。
%     0 にすると照合しない。選別後のパケット間隔の中央値と比較して
%     診断表示するほか、送信元の自動選択でも「この間隔で送っている
%     送信元」を優先する材料に使う。
expectedIntervalUs = 5780;

% [D] MAC を復号できなかったパケットの CSI を残すか
%     VHT のデータ部 (例 MCS3 = 16QAM) は、プリアンブルや SIG フィールド
%     (BPSK R=1/2) より 10dB 近く高い受信品質を要求する。そのため
%       L-SIG / VHT-SIG-A / VHT-SIG-B は毎回読めるのに、
%       MPDU の FCS だけが一度も通らない
%     という状態が普通に起こる (注入源が近すぎて ADC が飽和している、
%     逆に遠すぎて SNR が足りない、など)。
%
%     一方 CSI は VHT-LTF から求めており MPDU 復号とは無関係なので、
%     この状態でも CSI 自体は正しく得られている。true にすると、MAC を
%     復号できなかったパケットも捨てずに
%       擬似アドレス '(PHY:VHT-MCS<n>)'
%     の送信元としてまとめ、CSI を残す。PHY パラメータ (フォーマット /
%     MCS / 20MHz / NSTS=1) と送信間隔だけで注入トラフィックを識別する
%     運用になる (本ファイル冒頭の手掛かり [B][C] のみで選別する形)。
%
%     false にすると従来どおり MAC ヘッダを読めたパケットだけを記録する。
keepPHYOnlyCSI = true;

% --- 出力ファイル名に入れる識別子 ----------------------------------------
%     <yyyymmddHHMM>_<outputTag>_CSI.mat として保存される。
outputTag = 'Pico';

% --- 復号パラメータ ------------------------------------------------------
chanBW          = 'CBW20';      % WLAN Toolbox のチャネル帯域幅指定
                                 % (キャプチャ時の 20MHz に対応)
pktDetThreshold = 0.5;          % wlanPacketDetect のしきい値 (0〜1)
                                 % 下げると弱いパケットも拾うが誤検出が増え、
                                 % 復号時間も伸びる
verboseErrors   = false;        % true にすると復号エラーを毎回表示する
                                 % (通常は最後に集計のみ表示)
verbosePackets  = false;         % true にすると復号できたパケットを1件ずつ表示。
                                 % コマンドウィンドウへの出力は1件あたり数ms
                                 % かかり、数千件では無視できないため既定で
                                 % 止めている。保存される .mat の内容は同じ。

% --- 高速化に関する内部パラメータ (通常は変更不要) -----------------------
% 1パケットの処理に切り出すサンプル数の上限。
% 802.11 の最大 PPDU 長は 5.484 ms = 109,680 サンプル @20MSps なので、
% これを超える長さを要求するパケットは正規のものではない。
maxPktSamples = 120000;

% パケット検出窓。wlanPacketDetect は窓内の最初の1件しか返さないため、
% 大きい窓はその後ろを無駄に走査することになる。密な区間では小さく、
% 見つからなければ倍々に広げてループ回数を抑える。
detectWindowMin = 40000;        % 2 ms 分
detectWindowMax = 400000;       % 20 ms 分

%% ------------------------------------------------------------------------
%  2. 生IQファイルの読み込み
%  ------------------------------------------------------------------------
if isempty(inputRawFile)
    listing = dir(fullfile(hddInputPath, '*_raw.mat'));
    if isempty(listing)
        error('decodeIQ_VHT_Pico:noInputFile', ...
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
    error('decodeIQ_VHT_Pico:inputNotFound', '入力ファイルが存在しません: %s', inputRawFile);
end

S = load(inputRawFile, 'iq', 'meta');
if ~isfield(S, 'iq') || ~isfield(S, 'meta')
    error('decodeIQ_VHT_Pico:badInputFile', ...
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
if isempty(targetTxAddress)
    fprintf('  対象送信元      : (自動選択)\n');
else
    fprintf('  対象送信元      : %s\n', targetTxAddress);
end
fprintf('  対象フォーマット: %s\n', strjoin(targetFormats, ', '));
if ~isempty(targetMCS)
    fprintf('  対象 MCS        : %d\n', targetMCS);
end

%% ------------------------------------------------------------------------
%  3. 出力先の準備 (HDD と USB メモリの両方)
%  ------------------------------------------------------------------------
tagSafe     = regexprep(outputTag, '[^A-Za-z0-9_-]', '_');
outFileName = [timestamp '_' tagSafe '_CSI.mat'];

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
            warning('decodeIQ_VHT_Pico:mkdirFailed', ...
                '%s 保存先フォルダを作成できませんでした (%s): %s', label, pth, msg);
            continue;
        end
    end

    testFile = fullfile(pth, '.write_test.tmp');
    fidTest  = fopen(testFile, 'w');
    if fidTest == -1
        warning('decodeIQ_VHT_Pico:notWritable', ...
            '%s 保存先に書き込みできないためスキップします: %s', label, pth);
        continue;
    end
    fclose(fidTest);
    delete(testFile);

    outMatFiles{end+1} = fullfile(pth, outFileName); %#ok<SAGROW>
end

if isempty(outMatFiles)
    error('decodeIQ_VHT_Pico:noWritableOutput', ...
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

% pktLog は事前確保する。1件ずつ伸ばすと要素ごとに配列全体がコピーされ、
% 件数の二乗に比例したコストになる (各要素が52〜56要素の複素CSIを持つため
% 無視できない)。足りなくなったら倍々に伸ばし、最後に切り詰める。
% フィールドの順序は buildEntry の返す構造体と一致させること
% (順序が違うと構造体配列への代入でエラーになる)。
pktLogTemplate = struct('timeSec', 0, 'bssid', '', 'addr1', '', 'frameType', '', ...
    'ssid', '', 'phyFormat', '', 'mcs', -1, 'fcsVerified', false, ...
    'macDecoded', false, 'mpduCount', 0, 'csi', []);
pktLog  = repmat(pktLogTemplate, 20000, 1);
numPkts = 0;   % pktLog に実際に格納した件数

bssidToSSID = containers.Map('KeyType', 'char', 'ValueType', 'char');

% 復号統計 (診断用)
stats = struct('detected', 0, 'timingSkip', 0, 'nonHT', 0, 'ht', 0, 'vht', 0, ...
    'htGF', 0, 'other', 0, 'htUnsupported', 0, 'vhtUnsupported', 0, ...
    'decodeOK', 0, 'phyOnly', 0, 'vhtPhyOnly', 0, 'noAddr2', 0, 'errors', 0);

% --- VHT データ部の品質診断 -----------------------------------------------
% 「SIG は読めるのに FCS が通らない」原因を切り分けるための材料。
%   受信SNR (L-LTF)      : プリアンブルから見た受信品質
%   データ部 EVM         : 実際に復調した信号点の誤差。SNR が高いのに EVM が
%                          悪ければ、雑音ではなく歪み (ADC 飽和など) が原因
%   振幅ピーク           : 1.0 (正規化フルスケール) に張り付いていれば飽和
%   共通位相誤差 (CPE)   : 残留CFO/SFOの指標。大きいとEVMも悪化する
vhtQualMax = 20000;
vhtQual = struct('snrdB', nan(vhtQualMax, 1), 'evmPct', nan(vhtQualMax, 1), ...
    'peak', nan(vhtQualMax, 1), 'mcs', nan(vhtQualMax, 1), 'cpeDeg', nan(vhtQualMax, 1));
nVhtQual   = 0;
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

% 警告の抑止はループの外で一度だけ行う。warning の状態保存・復元は
% 呼び出しコストが高く、パケットごとに実行すると数万回に達して無視できない。
% ※スクリプトが途中でエラー終了すると抑止が解除されないままになる。
%   その場合は warning('on', 'all') を手で実行して戻すこと。
wStateSaved = warning('off', 'all');

% 検出は窓に区切って行う (毎回バッファ全体を走査すると非常に遅くなるため)。
% 窓の境界にまたがるプリアンブルを取りこぼさないよう少し重ねて進める。
detectWindow  = detectWindowMin;
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
        % 見つからなかった区間は疎とみなし、窓を広げてループ回数を抑える
        detectWindow = min(detectWindow * 2, detectWindowMax);
        continue;
    end
    detectWindow = detectWindowMin;   % 見つかったら密な区間として窓を戻す
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
        % [高速化1] このパケットの処理に必要な範囲だけを切り出す。
        %   旧版は iq(pktStart+1:end) と残りバッファ全体を渡していたため、
        %   1パケットあたり最大1億サンプルの複素指数を計算していた。
        %   802.11 の最大 PPDU 長を超える分は決して使われない。
        % 精CFO推定には L-LTF (161:320) までがあればよいので、まず先頭だけ
        % 補正して推定し、粗CFO と合算して1回で全体に適用する。
        %   exp(-j*theta_c) * exp(-j*theta_f) = exp(-j*(theta_c+theta_f))
        % なので、2回に分けて掛ける旧版と結果は同一。
        segEnd  = min(numel(iq), pktStart + maxPktSamples);
        pre     = applyCFO(iq(pktStart + (1:320)), sampleRate, -coarseCFO);
        fineCFO = wlanFineCFOEstimate(pre(161:320), chanBW);
        pkt     = applyCFO(iq(pktStart+1:segEnd), sampleRate, -(coarseCFO + fineCFO));

        % --- L-LTF を復調してからチャネル推定・ノイズ推定 ---
        %     (wlanLLTFChannelEstimate は時間領域波形ではなく復調済み
        %      シンボルを受け取る点に注意)
        lltfDemod   = wlanLLTFDemodulate(pkt(161:320), chanBW);
        lltfChanEst = wlanLLTFChannelEstimate(lltfDemod, chanBW);
        noiseEst    = wlanLLTFNoiseEstimate(lltfDemod);

        % --- フォーマット検出 (L-SIG + 後続2シンボル) ---
        %     内部の L-SIG チェック失敗警告はループ外で抑止済み
        fmt = wlanFormatDetect(pkt(321:560), lltfChanEst, noiseEst, chanBW);
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
                [st, consumed, entry, vhtInfo] = processVHT(pkt, lltfChanEst, ...
                    noiseEst, chanBW, keepPHYOnlyCSI);

                % データ部の品質を記録 (原因切り分け用)
                if vhtInfo.hasQuality && nVhtQual < vhtQualMax
                    nVhtQual = nVhtQual + 1;
                    vhtQual.snrdB(nVhtQual)  = vhtInfo.snrdB;
                    vhtQual.evmPct(nVhtQual) = vhtInfo.evmPct;
                    vhtQual.peak(nVhtQual)   = vhtInfo.peakAmp;
                    vhtQual.mcs(nVhtQual)    = vhtInfo.mcs;
                    vhtQual.cpeDeg(nVhtQual) = vhtInfo.cpeDeg;
                end

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
                    if vhtInfo.hasQuality
                        fprintf(['            品質: 受信SNR=%.1f dB データ部EVM=%.1f%% ', ...
                                 '振幅ピーク=%.3f 共通位相誤差=%.1f度 採用APEP=%d\n'], ...
                            vhtInfo.snrdB, vhtInfo.evmPct, vhtInfo.peakAmp, ...
                            vhtInfo.cpeDeg, vhtInfo.apepUsed);
                    end
                end
                if ~isempty(vhtInfo.reason)
                    % csiKept = true は「MAC は読めなかったが CSI は残した」。
                    % 構成が非対応で捨てたものと区別して数える。
                    if vhtInfo.csiKept
                        stats.vhtPhyOnly = stats.vhtPhyOnly + 1;
                    else
                        stats.vhtUnsupported = stats.vhtUnsupported + 1;
                    end
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
            if ~isempty(entry) && ~entry.macDecoded
                stats.phyOnly = stats.phyOnly + 1;
            end

            if isempty(entry)
                % ACK/CTS など Address2 を持たないフレーム (BSSID 判定に使えない)
                stats.noAddr2 = stats.noAddr2 + 1;
            else
                entry.timeSec = pktStart / sampleRate;

                numPkts = numPkts + 1;
                if numPkts > numel(pktLog)
                    % 事前確保が尽きたら倍に伸ばす (毎回1件ずつ伸ばすより安い)
                    pktLog = [pktLog; repmat(pktLogTemplate, numel(pktLog), 1)]; %#ok<AGROW>
                end
                pktLog(numPkts) = entry;

                if any(strcmpi(entry.frameType, {'Beacon', 'Probe Response'})) && ~isempty(entry.ssid)
                    bssidToSSID(entry.bssid) = entry.ssid;
                end

                if verbosePackets
                    fprintf('  [%7.4fs] %-6s %-16s BSSID=%s%s%s\n', entry.timeSec, ...
                        entry.phyFormat, entry.frameType, entry.bssid, ...
                        ternary(~isempty(entry.ssid), sprintf('  SSID="%s"', entry.ssid), ''), ...
                        ternary(entry.mpduCount > 1, sprintf('  (MPDU集約x%d)', entry.mpduCount), ''));
                end
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
            % warning はループ外で抑止しているため fprintf で表示する
            fprintf('  <エラー> パケット処理中にエラー: %s\n', ME.message);
        end
        searchOffset = nextSearch;
    end
end

% 警告の抑止を解除し、pktLog を実際の件数へ切り詰める
warning(wStateSaved);
pktLog = pktLog(1:numPkts);

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
if stats.vhtPhyOnly > 0
    fprintf('      うち MAC復号不可だが CSI は取得: %d\n', stats.vhtPhyOnly);
end
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
        fprintf('    MAC を復号できなかった理由: ');
        for k = 1:numel(rKeys)
            fprintf('%s=%d回  ', rKeys{k}, vhtRejects(rKeys{k}));
        end
        fprintf('\n');
        if keepPHYOnlyCSI && stats.vhtPhyOnly > 0
            fprintf(['      ※このうち PHY 構成が対応範囲内 (SU/20MHz/NSTS=1) の %d 件は\n', ...
                     '        keepPHYOnlyCSI = true により CSI を残している。\n'], ...
                stats.vhtPhyOnly);
        end
    end
    % VHT-SIG-A のビット解釈が正しいかの自己判定
    fprintf('    SIG-A予約ビット検証: %d/%d 個が仕様どおり (CRC通過分のみ)', ...
        vhtReservedOK, vhtSigParsed);
    if vhtSigParsed > 0 && vhtReservedOK < vhtSigParsed * 0.5
        fprintf('  <-- 大半が不一致。ビット解釈がズレている可能性あり\n');
    else
        fprintf('  (ビット解釈は妥当)\n');
    end

    % --- データ部の受信品質 (FCS が通らない原因の切り分け) ---
    if nVhtQual > 0
        snrMed  = median(vhtQual.snrdB(1:nVhtQual),  'omitnan');
        evmMed  = median(vhtQual.evmPct(1:nVhtQual), 'omitnan');
        peakMed = median(vhtQual.peak(1:nVhtQual),   'omitnan');
        peakHi  = mean(vhtQual.peak(1:nVhtQual) > 0.95) * 100;

        % 観測されたMCSの最頻値 (要求SNRとの比較に使う)
        mcsObs = mode(vhtQual.mcs(1:nVhtQual));
        evmCeil = evmCeilingPct(mcsObs);          % 信号点がランダムなときのEVM
        evmSaturated = ~isnan(evmMed) && ~isnan(evmCeil) && evmMed > 0.8 * evmCeil;
        reqSNR  = vhtRequiredSNRdB(mcsObs);       % そのMCSに必要な受信SNRの目安

        fprintf('  [VHT データ部の品質診断] (%d パケットの中央値, MCS%d)\n', ...
            nVhtQual, mcsObs);
        fprintf('    受信SNR (L-LTF)     : %.1f dB', snrMed);
        if ~isnan(reqSNR)
            fprintf('   (MCS%d の目安 %.0f dB に対し %+.1f dB)', ...
                mcsObs, reqSNR, snrMed - reqSNR);
        end
        fprintf('\n');
        if isnan(evmMed)
            fprintf('    データ部 EVM        : 測定できず\n');
        elseif evmSaturated
            % EVM は「最も近い理想信号点」を基準に測っているため、信号点が
            % 判定領域からはみ出すほど劣化すると値が頭打ちになる。頭打ちに
            % 達している = データ部が事実上復調できていない、という意味。
            fprintf('    データ部 EVM        : %.1f %%  <-- 判定飽和 (この変調の上限 ≒ %.1f%%)\n', ...
                evmMed, evmCeil);
            fprintf('                          信号点が理想点と対応づかない = データ部は復調できていない\n');
        else
            fprintf('    データ部 EVM        : %.1f %%  (等価SNR %.1f dB)\n', ...
                evmMed, -20*log10(max(evmMed, realmin)/100));
        end
        fprintf('    振幅ピーク          : %.3f  (フルスケール=1.0, >0.95 が %.1f%%)\n', ...
            peakMed, peakHi);
        cpeMed = median(vhtQual.cpeDeg(1:nVhtQual), 'omitnan');
        if ~isnan(cpeMed)
            fprintf('    共通位相誤差(CPE)   : %.1f 度  (残留CFO/SFOの目安。大きいほど劣化)\n', cpeMed);
        end
        if ~isnan(snrMed) && snrMed > 0
            % CSI の位相ばらつきの目安 (推定SNR gamma に対し標準偏差 ~ 1/sqrt(2*gamma))
            gammaLin = 10^(snrMed / 10);
            fprintf('    CSI の位相ばらつき  : 約 ±%.0f 度 (この SNR での目安)\n', ...
                rad2deg(1 / sqrt(2 * gammaLin)));
        end

        % --- 判定 ---
        % SIG フィールドは BPSK R=1/2 なので低い SNR でも読めてしまう。
        % データ部が要求する SNR との差が「SIG は読めるのに FCS が通らない」
        % 状態を生む。振幅ピークは飽和の有無だけを見る指標であり、
        % 小さいこと自体は SNR とは直接関係しない (SNR はアンテナ入力の
        % 信号対雑音比で決まり、受信機の gain を上げても改善しない)。
        fprintf('    判定: ');
        if ~isnan(peakMed) && (peakMed > 0.95 || peakHi > 20)
            fprintf(['振幅がフルスケールに達している = ADC が飽和している。\n', ...
                     '      注入源が近すぎるか captureIQ.m の gain が高すぎる。\n', ...
                     '      gain を 10〜20dB 下げて取り直すこと。\n']);
        elseif ~isnan(snrMed) && ~isnan(reqSNR) && snrMed < reqSNR
            fprintf(['受信SNR が MCS%d の要求に %.1f dB 足りない。\n'], ...
                mcsObs, reqSNR - snrMed);
            % gain の効き方: 受信機の雑音指数は gain の配分 (LNA と後段の
            % どちらで稼いでいるか) によって変わるため、gain を上げると
            % ある程度まで SNR が改善することがある (改善量は1dB gainあたり
            % 1dBより小さく、上げるほど頭打ちに近づく)。ただし外来の干渉・
            % 距離で決まる分は gain では改善しない。振幅ピークに余裕がある
            % うちは飽和のリスクなく gain を上げられるので、まずそこを試す
            % 価値がある。
            if peakMed < 0.3 && gain < 76
                fprintf(['      振幅ピーク %.3f はまだ余裕がある (目安 0.3〜0.5 まで\n', ...
                         '      飽和しない)。現在の gain=%ddB を上げると SNR が\n', ...
                         '      ある程度改善する可能性がある (伸びは gain を上げるほど\n', ...
                         '      鈍る)。振幅ピークを見ながら 10dB 刻みで上げて試すこと。\n'], ...
                    peakMed, gain);
            end
            fprintf(['      それでも届かない場合、効くのは次の順:\n', ...
                     '        1. 送信側を --mcs 0 (BPSK R=1/2, 目安 %.0f dB) にする\n', ...
                     '           ※CSI の品質は MCS に依存しないので副作用は無い\n', ...
                     '        2. 送受信アンテナを近づける / 向きと見通しを改善する\n', ...
                     '        3. PicoScenes 側の送信出力を上げる\n', ...
                     '        4. 空いているチャネルへ移る (周囲のAPと干渉している)\n', ...
                     '      ※日をまたぐと周囲の混雑で SNR が数dB変動するため、\n', ...
                     '        gain の効果を比べるときは同じセッション内で\n', ...
                     '        連続して取り直すこと。\n'], ...
                vhtRequiredSNRdB(0));
        elseif ~isnan(snrMed) && ~isnan(evmMed) && ~evmSaturated && ...
                snrMed > 20 && evmMed > 2 * 100 / 10^(snrMed/20)
            fprintf(['受信SNR は十分なのに EVM が悪い = 雑音ではなく歪みが原因。\n', ...
                     '      ADC 飽和・送信側の非線形・強い隣接チャネル干渉を疑う。\n']);
        else
            fprintf('データ部の品質は良好。\n');
        end
        fprintf(['    ※CSI は VHT-LTF (プリアンブル) から求めており、データ部の\n', ...
                 '      復号可否とは無関係に有効である。ただし CSI 自体の\n', ...
                 '      ばらつきは受信SNR で決まるため、SNR の改善は CSI の\n', ...
                 '      品質向上にも直接効く。\n']);
    end
end
fprintf('  CSI を記録できたパケット  : %d\n', stats.decodeOK - stats.noAddr2);
fprintf('    うち MAC まで復号成功   : %d\n', ...
    stats.decodeOK - stats.noAddr2 - stats.phyOnly);
fprintf('    うち MAC不明 (PHYのみ)  : %d\n', stats.phyOnly);
fprintf('  Address2 無し(ACK/CTS等)で除外: %d\n', stats.noAddr2);
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
    allMacD = logical([pktLog.macDecoded]);
    for f = {'Non-HT', 'HT', 'VHT'}
        sel = strcmpi(allFmt, f{1});
        if any(sel)
            fprintf('    %-6s : 記録%d件 (FCS検証OK=%d, ヘッダのみ推定=%d, MAC不明=%d)\n', ...
                f{1}, sum(sel), sum(sel & allFcsV), ...
                sum(sel & ~allFcsV & allMacD), sum(sel & ~allMacD));
        end
    end
    fprintf(['    ※あるフォーマットで FCS検証OK が 0 件の場合、そのフォーマットの\n', ...
             '      データ復号が成立していない (受信品質不足、または変調パラメータの\n', ...
             '      推定ミス) ことを示す。原因の切り分けは上の\n', ...
             '      [VHT データ部の品質診断] を参照。CSI 自体はプリアンブルから\n', ...
             '      求めているため、この結果に関わらず有効である。\n']);
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
%  6. 送信元の一覧を作り、対象とするトラフィックを選ぶ
%  ------------------------------------------------------------------------

% --- ビット誤りで化けた送信元アドレスを擬似アドレスへ統合する -------------
% FCS が通らない状況では、MAC ヘッダのみ復号したパケットの Address2 は
% ビット誤りでバラバラの値に化ける。実測では 1 つの注入源が 29 個の別々の
% アドレスとして現れ、下位バイトだけが共通する (…10121122 など) 形になった。
% これらは実在しない送信元であり、一覧を埋め尽くすうえに本来同じ注入
% トラフィックである CSI を分断してしまう。
%
% そこで「FCS を一度も通しておらず、かつ数件しか現れないアドレス」は
% 化けたものとみなし、同じ PHY 設定の擬似アドレスへまとめ直す。
% 統合するのは Address2 の帰属だけで、CSI には手を触れない。
suspiciousMaxCount = 5;   % これ未満の件数なら化けたアドレスとみなす
nFoldedBogus = 0;
if keepPHYOnlyCSI && ~isempty(pktLog)
    bogusAddrs = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    tmpBssid = {pktLog.bssid};
    tmpFcs   = logical([pktLog.fcsVerified]);
    for k = 1:numel(pktLog)
        a = tmpBssid{k};
        if isKey(bogusAddrs, a) || strncmp(a, '(PHY:', 5)
            continue;
        end
        sel = strcmp(tmpBssid, a);
        bogusAddrs(a) = (sum(sel) < suspiciousMaxCount) && ~any(sel & tmpFcs);
    end

    for k = 1:numel(pktLog)
        if pktLog(k).macDecoded && ~pktLog(k).fcsVerified && ...
                bogusAddrs(pktLog(k).bssid)
            pktLog(k).bssid      = phyOnlyAddress(pktLog(k).phyFormat, pktLog(k).mcs);
            pktLog(k).addr1      = '';
            pktLog(k).frameType  = '(MAC復号不可)';
            pktLog(k).macDecoded = false;
            nFoldedBogus = nFoldedBogus + 1;
        end
    end

    if nFoldedBogus > 0
        fprintf(['\n[送信元アドレスの整理] %d 件を擬似アドレスへ統合しました。\n', ...
                 '  FCS を一度も通しておらず %d 件未満しか現れないアドレスは、\n', ...
                 '  ペイロードのビット誤りで MAC ヘッダが化けたものとみなしています。\n'], ...
            nFoldedBogus, suspiciousMaxCount);
    end
end

% Beacon から学習できた BSSID/SSID (注入源と周囲のAPを見分けるために使う)
seenNetworks = struct('bssid', {}, 'ssid', {});
bssidKeys = keys(bssidToSSID);
for k = 1:numel(bssidKeys)
    seenNetworks(end+1) = struct('bssid', bssidKeys{k}, ...
        'ssid', bssidToSSID(bssidKeys{k})); %#ok<SAGROW>
end

allBssid = {pktLog.bssid};   % Address2 (送信元)
allAddr1 = {pktLog.addr1};   % Address1 (宛先)

% --- 送信元アドレスごとの集計 -------------------------------------------
% PicoScenes の注入源は Beacon を出さないので SSID が付かない。
% 「SSID が無いのに大量に送っているアドレス」が注入源である。
txCensus = struct('address', {}, 'count', {}, 'nNonHT', {}, 'nHT', {}, ...
    'nVHT', {}, 'nFCSOK', {}, 'ssid', {}, 'mcsList', {}, 'medianIntervalUs', {});

if ~isempty(pktLog)
    allFmtAll  = {pktLog.phyFormat};
    allMcsAll  = [pktLog.mcs];
    allTimeAll = [pktLog.timeSec];
    allFcsAll  = logical([pktLog.fcsVerified]);
    uniqTx = unique(allBssid);

    for k = 1:numel(uniqTx)
        sel = strcmp(allBssid, uniqTx{k});

        ssidStr = '';
        if isKey(bssidToSSID, uniqTx{k})
            ssidStr = bssidToSSID(uniqTx{k});
        end

        % 送信間隔の中央値 (注入の周期性を確認するため)
        tSel = sort(allTimeAll(sel));
        if numel(tSel) >= 2
            medIntervalUs = median(diff(tSel)) * 1e6;
        else
            medIntervalUs = NaN;
        end

        txCensus(end+1) = struct( ...
            'address',          uniqTx{k}, ...
            'count',            sum(sel), ...
            'nNonHT',           sum(sel & strcmpi(allFmtAll, 'Non-HT')), ...
            'nHT',              sum(sel & strcmpi(allFmtAll, 'HT')), ...
            'nVHT',             sum(sel & strcmpi(allFmtAll, 'VHT')), ...
            'nFCSOK',           sum(sel & allFcsAll), ...
            'ssid',             ssidStr, ...
            'mcsList',          unique(allMcsAll(sel)), ...
            'medianIntervalUs', medIntervalUs); %#ok<SAGROW>
    end

    % 件数の多い順に並べる (注入源が先頭に来るようにする)
    [~, ord] = sort([txCensus.count], 'descend');
    txCensus = txCensus(ord);
end

fprintf('\n送信元アドレス一覧 (Address2 別。パケット数の多い順):\n');
if isempty(txCensus)
    fprintf('  (なし)\n');
else
    fprintf('  %-18s %6s %7s %5s %6s %7s  %-10s %12s  %s\n', ...
        'MAC', '件数', 'Non-HT', 'HT', 'VHT', 'FCS OK', 'MCS', '平均間隔[us]', 'SSID');
    for k = 1:numel(txCensus)
        c = txCensus(k);
        mcsStr = strjoin(arrayfun(@(m) sprintf('%d', m), c.mcsList, ...
            'UniformOutput', false), ',');
        fprintf('  %-18s %6d %7d %5d %6d %7d  %-10s %12.1f  %s\n', ...
            c.address, c.count, c.nNonHT, c.nHT, c.nVHT, c.nFCSOK, mcsStr, ...
            c.medianIntervalUs, c.ssid);
    end
    fprintf(['  ※SSID 欄が空 = Beacon を出していない送信元。PicoScenes の\n', ...
             '    injector はここに現れる。件数が突出していれば注入源。\n', ...
             '  ※"(PHY:VHT-MCSn)" は MAC を復号できなかったパケットを PHY 設定\n', ...
             '    ごとにまとめた擬似的な送信元 (keepPHYOnlyCSI = true のとき)。\n', ...
             '    送信元 MAC は分からないが CSI は有効で、平均間隔が --delay と\n', ...
             '    一致していれば注入トラフィックそのものである。\n', ...
             '  ※"FCS OK" が 0 かつ件数が 1〜数件のアドレスは、ペイロードの\n', ...
             '    ビット誤りで MAC ヘッダが化けたものである可能性が高く、実在\n', ...
             '    しない送信元なので選別には使わないこと。keepPHYOnlyCSI が\n', ...
             '    true なら擬似アドレス側へ自動的に統合される。\n', ...
             '  ※複数空間ストリームのパケットは SISO 受信では復号できず現れない。\n']);
end

% --- 対象の決定 ----------------------------------------------------------
% targetFormats / targetMCS を満たすパケットに限定したうえで、
% targetTxAddress が空なら「SSID を持たない送信元のうち最多」を注入源とみなす。
% 1件も復号できていない場合は空の論理配列を作るだけにする
% (空セルとの strcmpi は次元が揃わずエラーになり得るため分けている)
if isempty(pktLog)
    isCandidate = false(1, 0);
else
    allPhyFormat = {pktLog.phyFormat};
    isFmtOK = false(1, numel(pktLog));
    for k = 1:numel(targetFormats)
        isFmtOK = isFmtOK | strcmpi(allPhyFormat, targetFormats{k});
    end

    isMcsOK = true(1, numel(pktLog));
    if ~isempty(targetMCS)
        isMcsOK = ([pktLog.mcs] == targetMCS);
    end

    isCandidate = isFmtOK & isMcsOK;
end

if isempty(targetTxAddress)
    % 自動選択: 候補パケットを送信元ごとに数え、SSID を持たないものを優先
    bestAddr = '';
    bestScore = -1;
    for k = 1:numel(txCensus)
        addr = txCensus(k).address;
        n = sum(isCandidate & strcmp(allBssid, addr));
        if n == 0
            continue;
        end
        % FCS が 1 件も通らず件数も僅かなアドレスは、ビット誤りで MAC ヘッダ
        % が化けた偽の送信元とみなして候補から外す。keepPHYOnlyCSI = true なら
        % 同じパケット群は擬似アドレス '(PHY:...)' 側にまとまっており、
        % そちらが正しい選択肢になる。
        if keepPHYOnlyCSI && txCensus(k).nFCSOK == 0 && ...
                txCensus(k).count < suspiciousMaxCount && ~strncmp(addr, '(PHY:', 5)
            continue;
        end
        % Beacon を出している = AP なので注入源ではない。件数が同じなら
        % SSID を持たない方を優先する。
        score = n * 2 + double(isempty(txCensus(k).ssid));

        % 送信間隔が --delay と一致していれば注入源である強い証拠なので
        % 大きく優先する。偶然の一致を避けるため件数が少ないものは除く。
        if expectedIntervalUs > 0 && n >= 10 && ~isnan(txCensus(k).medianIntervalUs)
            relErr = abs(txCensus(k).medianIntervalUs - expectedIntervalUs) / expectedIntervalUs;
            if relErr < 0.10
                score = score * 10;
            end
        end

        if score > bestScore
            bestScore = score;
            bestAddr  = addr;
        end
    end
    targetTxAddress = bestAddr;

    if isempty(targetTxAddress)
        fprintf('\n自動選択: 条件に合う送信元が見つかりませんでした。\n');
    else
        fprintf('\n自動選択した送信元: %s\n', targetTxAddress);
        fprintf(['  ※意図と違う場合は targetTxAddress に上の一覧から\n', ...
                 '    正しいアドレスを指定して再実行してください。\n']);
    end
end

matched = pktLog([]);   % 空の構造体配列

if isempty(targetTxAddress)
    warning('decodeIQ_VHT_Pico:noTargetFound', ...
        ['対象とするトラフィックを特定できませんでした。\n', ...
         '  対象フォーマット: %s\n', ...
         '  対象 MCS        : %s\n', ...
         '送信側が送信中か、captureIQ.m の中心周波数が PicoScenes の\n', ...
         'チャネル (例 5180) と一致しているかを確認してください。\n', ...
         'targetMCS を [] にする、targetFormats を広げる、pktDetThreshold を\n', ...
         '下げる (例 0.3) なども試してください。'], ...
        strjoin(targetFormats, ','), ...
        ternary(isempty(targetMCS), '(指定なし)', sprintf('%d', targetMCS)));
else
    % 送信元・宛先のどちらかが対象アドレスなら採用する
    isAddrOK = strcmpi(allBssid, targetTxAddress) | strcmpi(allAddr1, targetTxAddress);
    matched  = pktLog(isAddrOK & isCandidate);
    fprintf('\n対象送信元 %s のパケット数: %d\n', targetTxAddress, numel(matched));

    % --- 送信周期の照合 (対象が正しいかの裏付け) ---
    if ~isempty(matched) && numel(matched) >= 2
        tm = sort([matched.timeSec]);
        medUs = median(diff(tm)) * 1e6;
        fprintf('  パケット間隔の中央値: %.1f us (%.1f packets/s)\n', ...
            medUs, 1e6 / medUs);
        if expectedIntervalUs > 0
            errPct = abs(medUs - expectedIntervalUs) / expectedIntervalUs * 100;
            if errPct < 10
                fprintf('  → 期待値 %.0f us と一致 (誤差 %.1f%%)。対象は正しいと判断できる。\n', ...
                    expectedIntervalUs, errPct);
            else
                fprintf(['  → 期待値 %.0f us と乖離 (誤差 %.1f%%)。\n', ...
                         '     取りこぼしがある (検出漏れ) か、対象の選択が誤っている\n', ...
                         '     可能性がある。上の一覧を確認してください。\n'], ...
                    expectedIntervalUs, errPct);
            end
        end
    end
end

% --- フォーマット別に [パケット数 x サブキャリア数] の行列へまとめる ---
% Non-HT/HT は 52 本、VHT(20MHz) は 56 本とサブキャリア数が異なるため、
% 1 つの行列には混ぜられない。フォーマットごとに分けて保存する。
if isempty(matched)
    allFormats   = {};
    allTimeSec   = [];
    allFrameType = {};
    allFcs       = logical([]);
    allMacOK     = logical([]);
else
    allFormats   = {matched.phyFormat}.';
    allTimeSec   = [matched.timeSec].';
    allFrameType = {matched.frameType}.';
    allFcs       = logical([matched.fcsVerified]).';
    allMacOK     = logical([matched.macDecoded]).';
end

isNonHT = strcmpi(allFormats, 'Non-HT');
isHT    = strcmpi(allFormats, 'HT');
isVHT   = strcmpi(allFormats, 'VHT');

csiNonHT       = stackCSI(matched(isNonHT));
timeSecNonHT   = allTimeSec(isNonHT);
frameTypeNonHT = allFrameType(isNonHT);
fcsNonHT       = allFcs(isNonHT);
macDecodedNonHT = allMacOK(isNonHT);

csiHT       = stackCSI(matched(isHT));
timeSecHT   = allTimeSec(isHT);
frameTypeHT = allFrameType(isHT);
fcsHT       = allFcs(isHT);
macDecodedHT = allMacOK(isHT);

csiVHT       = stackCSI(matched(isVHT));
timeSecVHT   = allTimeSec(isVHT);
frameTypeVHT = allFrameType(isVHT);
fcsVHT       = allFcs(isVHT);
macDecodedVHT = allMacOK(isVHT);

if ~isempty(matched)
    fprintf('  内訳: Non-HT=%d, HT=%d, VHT=%d\n', ...
        size(csiNonHT, 1), size(csiHT, 1), size(csiVHT, 1));
    totalMPDU = sum([matched.mpduCount]);
    fprintf('  PPDU(電波上の送信単位)数=%d に対し、集約されたMPDU(データ単位)の合計=%d\n', ...
        numel(matched), totalMPDU);
    nPhyOnly    = sum(~allMacOK);
    nUnverified = sum(~allFcs & allMacOK);
    if nUnverified > 0
        fprintf(['  ※うち %d 件は FCS 未検証 (ペイロードにビット誤りがあり、\n', ...
                 '    MAC ヘッダのみから送信元を判定したもの)。\n'], nUnverified);
    end
    if nPhyOnly > 0
        fprintf(['  ※うち %d 件は MAC を復号できず、PHY 設定 (フォーマット/MCS/\n', ...
                 '    20MHz/NSTS=1) と送信間隔だけで注入トラフィックと判断したもの。\n'], ...
                 nPhyOnly);
    end
    if nUnverified > 0 || nPhyOnly > 0
        fprintf(['    いずれも CSI 自体はプリアンブル (VHT-LTF) から算出しており\n', ...
                 '    影響を受けません。\n']);
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
macSets    = {macDecodedNonHT, macDecodedHT, macDecodedVHT};

primaryFormat     = formatNames{bestIdx};
csi               = csiSets{bestIdx};
subcarrierIndices = subcSets{bestIdx};
timeSec           = timeSets{bestIdx};
frameType         = frameSets{bestIdx};
fcsVerified       = fcsSets{bestIdx};
macDecoded        = macSets{bestIdx};
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
csiMeta.description      = ['CSI filtered by target transmitter / PHY parameters ', ...
    '(Non-HT / HT(NSS=1,20MHz) / SU-VHT(NSTS=1,20MHz))'];
csiMeta.decoder          = 'decodeIQ_VHT_Pico.m';   % どの復号器で作ったか
csiMeta.targetTxAddress  = targetTxAddress;   % 選別に使った送信元
csiMeta.targetFormats    = targetFormats;
csiMeta.targetMCS        = targetMCS;
csiMeta.expectedIntervalUs = expectedIntervalUs;
csiMeta.keepPHYOnlyCSI   = keepPHYOnlyCSI;   % MAC 不明でも CSI を残したか
csiMeta.foldedBogusAddrs = nFoldedBogus;     % 化けたアドレスから統合した件数
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

% VHT データ部の受信品質 (FCS が通らない原因の切り分け用。診断専用)
if nVhtQual > 0
    csiMeta.vhtQuality = struct( ...
        'nPackets',    nVhtQual, ...
        'snrdBMedian', median(vhtQual.snrdB(1:nVhtQual),  'omitnan'), ...
        'evmPctMedian', median(vhtQual.evmPct(1:nVhtQual), 'omitnan'), ...
        'peakAmpMedian', median(vhtQual.peak(1:nVhtQual),  'omitnan'), ...
        'peakAmpOver095Pct', mean(vhtQual.peak(1:nVhtQual) > 0.95) * 100, ...
        'mcsMode',     mode(vhtQual.mcs(1:nVhtQual)), ...
        'requiredSNRdB', vhtRequiredSNRdB(mode(vhtQual.mcs(1:nVhtQual))), ...
        'cpeDegMedian', median(vhtQual.cpeDeg(1:nVhtQual), 'omitnan'));
else
    csiMeta.vhtQuality = struct([]);
end
csiMeta.captureDatetime  = timestamp;         % キャプチャ時刻
csiMeta.decodeDatetime   = datestr(now, 'yyyymmddHHMM');   % 復号を行った時刻
csiMeta.sourceRawFile    = inputRawFile;      % どの生IQから作られたか
csiMeta.captureMeta      = meta;              % キャプチャ時の全メタデータ
csiMeta.matlabVersion    = version;

% 旧形式 (SSID で絞る版) との互換のため空で出力しておく
targetSSIDOut     = '';               %#ok<NASGU>
targetTxAddressOut = targetTxAddress; %#ok<NASGU>
targetBSSIDOut = '';          %#ok<NASGU>

saveVars = { ...
    'csi', 'subcarrierIndices', 'packetStartIndex', 'csiMeta', ...
    'phyFormat', 'fcsVerified', 'macDecoded', 'timeSec', 'frameType', ...
    'csiNonHT', 'timeSecNonHT', 'frameTypeNonHT', 'fcsNonHT', 'macDecodedNonHT', ...
    'csiHT', 'timeSecHT', 'frameTypeHT', 'fcsHT', 'macDecodedHT', ...
    'csiVHT', 'timeSecVHT', 'frameTypeVHT', 'fcsVHT', 'macDecodedVHT', ...
    'subcarrierIndicesNonHT', 'subcarrierIndicesHT20', 'subcarrierIndicesVHT20', ...
    'targetSSIDOut', 'targetBSSIDOut', 'targetTxAddressOut', ...
    'seenNetworks', 'txCensus'};

fprintf('\n');
nSaved = 0;
for k = 1:numel(outMatFiles)
    try
        save(outMatFiles{k}, saveVars{:}, '-v7.3');
        fprintf('結果を保存しました: %s\n', outMatFiles{k});
        nSaved = nSaved + 1;
    catch ME
        % 片方 (USB の抜き差し等) が失敗しても、もう片方は残す
        warning('decodeIQ_VHT_Pico:saveFailed', ...
            '保存に失敗しました (%s): %s', outMatFiles{k}, ME.message);
    end
end

if nSaved == 0
    error('decodeIQ_VHT_Pico:allSavesFailed', ...
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

function nSamples = ppduSamplesFromLSIG(pkt, chanEst, noiseEst, chanBW)
    % L-SIG から PPDU 全体のサンプル長を求める。復号に失敗しても探索位置を
    % パケット全長分だけ進められるようにするためのもの。
    % HT-MF / VHT では L-SIG LENGTH は
    %   RXTIME = 4*ceil((L_LENGTH+3)/3) + 20 [us]
    % が PPDU 長になるよう設定されるため、フォーマットに依らず使える。
    % L-SIG が読めない場合は 0 を返す (呼び出し側が既定値を使う)。
    nSamples = 0;
    try
        [lsigBits, lsigFail] = wlanLSIGRecover(pkt(321:400), chanEst, noiseEst, chanBW);
        if lsigFail
            return;
        end
        [~, lsigLen] = decodeLSIGBits(lsigBits);
        if lsigLen > 0
            nSamples = (4 * ceil((lsigLen + 3) / 3) + 20) * 20;   % 20 samples/us
        end
    catch
        nSamples = 0;
    end
end

function varargout = htVhtDataRecover(recoverFcn, rxData, chanEst, noiseEst, cfg)
    % wlanHTDataRecover / wlanVHTDataRecover を LDPC 早期終了つきで呼ぶ。
    %
    % 出力は呼び出し側が要求した数だけそのまま中継する。2番目以降の意味は
    % 関数によって異なる (wlanVHTDataRecover なら 等化後信号点・共通位相誤差)。
    %
    % 早期終了はパリティ検査が全て通った時点で反復を打ち切る。その時点で
    % 既に有効な符号語になっているため、追加の反復は出力を変えない。
    % したがって復号結果はビット単位で既定動作と一致する。
    % (復号方式 'LDPCDecodingMethod' は結果が変わり得るので変更しない)
    %
    % 名前と値のペアの対応は MATLAB のバージョンにより異なるため、
    % 受け付けられない場合は既定の呼び出し方へ退避する。
    persistent useEarlyTermination
    if isempty(useEarlyTermination)
        useEarlyTermination = true;
    end

    nOut = max(nargout, 1);
    if useEarlyTermination
        try
            [varargout{1:nOut}] = recoverFcn(rxData, chanEst, noiseEst, cfg, ...
                'EarlyTermination', true);
            return;
        catch
            % このバージョンでは指定できないので以後は使わない
            useEarlyTermination = false;
        end
    end
    [varargout{1:nOut}] = recoverFcn(rxData, chanEst, noiseEst, cfg);
end

function [rxPSDU, eqSym, cpe] = vhtDataRecoverWithDiag(rxData, chanEst, noiseEst, cfg)
    % wlanVHTDataRecover を診断用の追加出力つきで呼ぶ。
    % 実際の出力は [bits, eqSym, cpe] の順であり (MATLAB WLAN Toolbox の
    % 仕様。以前の版では2番目の出力を「VHT-SIG-B CRC」と誤って解釈しており、
    % 実際には存在しないその出力の代わりに eqSym (等化後の信号点) を、
    % 3番目の cpe (Common Phase Error、シンボルごとの共通位相誤差 [rad]) を
    % 誤って eqSym として EVM 計算に渡していた。この誤りにより、EVM は
    % 実際にはシンボルではなく位相誤差の値を QAM 信号点と比較しており、
    % 受信SNR によらず一定の値 (約25〜30%) に張り付いていた。
    %   2番目: 等化後の信号点 eqSym (EVM の算出に使う)
    %   3番目: シンボルごとの共通位相誤差 cpe [rad] (残留CFO/SFOの指標)
    % これらを返さない MATLAB バージョンでも動くよう、一度だけ試して結果を
    % 憶えておき、以後は余計な呼び出しをしない。
    persistent hasDiagOutputs
    if isempty(hasDiagOutputs)
        % まず宣言されている出力数で判定し、それが取れない場合は
        % 実際に呼んでみて確かめる (下の try/catch)。
        try
            nOutDeclared  = nargout('wlanVHTDataRecover');
            hasDiagOutputs = (nOutDeclared < 0) || (nOutDeclared >= 3);
        catch
            hasDiagOutputs = true;
        end
    end

    eqSym = [];
    cpe   = [];
    if hasDiagOutputs
        try
            [rxPSDU, eqSym, cpe] = htVhtDataRecover(@wlanVHTDataRecover, ...
                rxData, chanEst, noiseEst, cfg);
            return;
        catch
            hasDiagOutputs = false;
            eqSym = [];
            cpe   = [];
        end
    end
    rxPSDU = htVhtDataRecover(@wlanVHTDataRecover, rxData, chanEst, noiseEst, cfg);
end

function info = measureVHTQuality(info, pktSeg, lltfChanEst, noiseEst, eqSym, cpe, mcs)
    % VHT パケットの受信品質を測る (診断専用。復号結果には影響しない)。
    %
    %   snrdB   : L-LTF から求めた受信SNR。プリアンブルから見た品質。
    %   evmPct  : 等化後の信号点と理想信号点の誤差 (EVM)。SNR が高いのに
    %             EVM が悪ければ、雑音ではなく歪みが原因 (ADC 飽和・
    %             送信側の非線形・強い隣接波など) だと分かる。
    %   peakAmp : パケット区間の振幅ピーク。comm.SDRuReceiver の double 出力
    %             は ±1.0 がフルスケールなので、1.0 付近に張り付いていれば
    %             飽和している。
    %   cpeDeg  : シンボルごとの共通位相誤差 (Common Phase Error) の実効値
    %             [度]。残留 CFO や SFO (サンプリング周波数オフセット) が
    %             大きいと、パケット内で位相がシンボルごとにずれていき
    %             値が大きくなる。大きいのに EVM も悪ければ、雑音ではなく
    %             周波数・タイミング推定側の問題を疑う材料になる。
    % 各項目を個別の try/catch にしているのは、どれか1つ (特に eqSym/cpe の
    % 形式は MATLAB バージョンや構成によって変わりうる) が例外を出しても、
    % 他の項目 (特に snrdB。ここが取れないと [VHT データ部の品質診断] の
    % セクション自体が丸ごと表示されなくなる) を道連れにしないため。
    % 以前の版はこれを1つの try/catch にまとめており、EVM 計算だけが失敗
    % した場合でも hasQuality が立たず診断全体が消えるという不具合があった。
    persistent evmWarned cpeWarned
    if isempty(evmWarned), evmWarned = false; end
    if isempty(cpeWarned), cpeWarned = false; end

    try
        if ~isempty(lltfChanEst) && noiseEst > 0
            info.snrdB = 10 * log10(mean(abs(lltfChanEst(:)).^2) / noiseEst);
        end
    catch
    end
    try
        if ~isempty(pktSeg)
            info.peakAmp = max(max(abs(real(pktSeg))), max(abs(imag(pktSeg))));
        end
    catch
    end
    try
        if ~isempty(eqSym)
            info.evmPct = evmFromEqSym(eqSym, mcs);
        end
    catch ME
        if ~evmWarned
            evmWarned = true;
            fprintf('    <警告> EVM計算に失敗 (以後は表示のみ抑止): %s\n', ME.message);
        end
    end
    try
        if ~isempty(cpe)
            info.cpeDeg = rad2deg(sqrt(mean(double(cpe(:)).^2)));
        end
    catch ME
        if ~cpeWarned
            cpeWarned = true;
            fprintf('    <警告> CPE計算に失敗 (以後は表示のみ抑止): %s\n', ME.message);
        end
    end
    info.hasQuality = ~isnan(info.snrdB) || ~isnan(info.evmPct);
end

function evmCeil = evmCeilingPct(mcs)
    % 「最も近い理想信号点」を基準にした EVM の上限 [%]。
    %
    % 受信信号点が判定領域内で一様に散らばる (= 事実上ランダム) とき、
    % 最近傍点との距離の二乗平均は d^2/6 になる (d は最小信号点間距離)。
    % 単位平均電力の方形QAMでは d = sqrt(6/(M-1)) なので、EVM の上限は
    %   100 * d / sqrt(6) = 100 / sqrt(M-1)  [%]
    % となる。測定値がこれに近ければ、EVM の値そのものには意味が無く
    % 「データ部が復調できていない」ことだけを示している。
    evmCeil = NaN;
    modOrders = [2 4 4 16 16 64 64 64 256 256];   % VHT MCS0..MCS9
    if isnan(mcs) || mcs < 0 || mcs > 9
        return;
    end
    M = modOrders(mcs + 1);
    if M < 4
        M = 4;   % BPSK は判定が1次元だが、目安として QPSK と同じ値を使う
    end
    evmCeil = 100 / sqrt(M - 1);
end

function snrdB = vhtRequiredSNRdB(mcs)
    % VHT 20MHz / 1空間ストリームで、そのMCSのデータ部を誤りなく復号する
    % のに必要な受信SNRの目安 [dB] (AWGN, PER 10%% 程度)。
    % 厳密な閾値ではなく「足りているか」を判断するための目安として使う。
    %   MCS0 BPSK 1/2 / MCS1 QPSK 1/2 / MCS2 QPSK 3/4 / MCS3 16QAM 1/2 /
    %   MCS4 16QAM 3/4 / MCS5 64QAM 2/3 / MCS6 64QAM 3/4 / MCS7 64QAM 5/6 /
    %   MCS8 256QAM 3/4 / MCS9 256QAM 5/6
    table = [5 8 10 13 17 21 22 24 28 30];
    snrdB = NaN;
    if isnan(mcs) || mcs < 0 || mcs > 9
        return;
    end
    snrdB = table(mcs + 1);
end

function evmPct = evmFromEqSym(eqSym, mcs)
    % 等化後の信号点から EVM [%] を求める。
    % 参照点は「最も近い理想信号点」とする (判定誤りを含むので EVM が非常に
    % 悪い領域では過小評価になるが、良否の判断には十分)。
    evmPct = NaN;
    modOrders = [2 4 4 16 16 64 64 64 256 256];   % VHT MCS0..MCS9 の変調多値数
    if isempty(eqSym) || mcs < 0 || mcs > 9
        return;
    end
    M = modOrders(mcs + 1);
    if M == 2
        ref = [-1; 1];
    else
        ref = qammod((0:M-1).', M, 'UnitAveragePower', true);
    end
    sym = eqSym(:);
    if numel(sym) > 4096
        sym = sym(1:4096);   % 診断用なので先頭だけで十分
    end
    [~, idx] = min(abs(sym - ref.'), [], 2);
    pErr = mean(abs(sym - ref(idx)).^2);
    pRef = mean(abs(ref).^2);   % UnitAveragePower なので 1
    if pRef > 0
        evmPct = 100 * sqrt(pErr / pRef);
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

    % 未対応サブタイプの警告は頻出するが、抑止は呼び出し側でループ外に
    % まとめて行っている (状態の保存・復元が高コストなため)
    [cfgMAC, payload, mpduStatus] = wlanMPDUDecode(rxPSDU, cfgNonHT, 'DataFormat', 'bits');

    if ~strcmpi(string(mpduStatus), "Success")
        status = 0;
        return;
    end

    entry = buildEntry(cfgMAC, payload, 'Non-HT', chanEst, mcs);
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

    % [高速化2] L-SIG から PPDU 全長を先に求めておく。
    %   以降どの段階で失敗しても、探索位置をパケット全長分だけ進められる。
    %   旧版は失敗時に 560 サンプル(28us)しか進めず、同じパケットの本体を
    %   再走査して偽のプリアンブル検出を大量に生んでいた。
    %   HT-MF では L-SIG LENGTH は
    %     RXTIME = 4*ceil((L_LENGTH+3)/3) + 20 [us]
    %   が PPDU 長になるよう設定される。
    consumed = max(consumed, ppduSamplesFromLSIG(pkt, lltfChanEst, noiseEst, chanBW));

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

    % [高速化4] LDPC は早期終了を有効にする。パリティ検査が全て通った時点で
    %   反復を打ち切るが、その時点で既に有効な符号語なので出力は変わらない。
    %   (復号方式そのものは結果が変わり得るため既定のまま)
    rxPSDU = htVhtDataRecover(@wlanHTDataRecover, ...
        pkt(ind.HTData(1):ind.HTData(2)), htChanEst, noiseEst, cfgHT);

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

    entry = buildEntry(cfgMAC, payload, 'HT', htChanEst, ht.mcs);
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

function [status, consumed, entry, info] = processVHT(pkt, lltfChanEst, noiseEst, chanBW, keepPHYOnly)
    % SU-VHT (802.11ac, NSTS=1, 20MHz) パケットを復号する。
    %   status: 1=成功(entryあり), 0=失敗/非対応, -1=サンプル不足
    %   info:   診断用 (棄却理由・観測された帯域幅/NSTS/MCS・受信品質)
    %
    % keepPHYOnly = true のとき、MPDU (MAC) を復号できなくても PHY の構成が
    % 対応範囲内 (SU / 20MHz / NSTS=1) であれば CSI を残す。CSI は VHT-LTF
    % から求めており MPDU 復号の成否とは無関係なため、「SIG は読めるのに
    % FCS が通らない」状態でも CSI そのものは有効である。
    if nargin < 5
        keepPHYOnly = false;
    end
    entry = [];
    consumed = 560;   % L-STF..VHT-SIG-A
    info = struct('reason', '', 'bwMHz', 0, 'nsts', 0, 'mcs', 0, ...
        'reservedOK', false, 'gi', 0, 'coding', 0, 'lsigLen', 0, 'nsym', 0, ...
        'apepFromLSIG', 0, 'apepFromSIGB', 0, 'apepUsed', 0, ...
        'mpduCount', 0, 'deagStatus', '', ...
        'nonBinary', false, 'dataClass', '', 'dataRange', [0 0], ...
        'csiKept', false, 'hasQuality', false, 'snrdB', NaN, 'evmPct', NaN, ...
        'peakAmp', NaN, 'cpeDeg', NaN);

    % [高速化2] L-SIG を先に復号し、PPDU 全長を求めておく。
    %   NSTS>=2 などで棄却する場合も含め、どの段階で失敗しても探索位置を
    %   パケット全長分だけ進められる。旧版は失敗時に 560 サンプル(28us)しか
    %   進めず、同じパケットの本体を再走査して偽検出を大量に生んでいた。
    %   ここで得た lsigLen は後段のパケット長算出でもそのまま使う
    %   (旧版は L-SIG を2回復号していた)。
    [lsigBits, lsigFail] = wlanLSIGRecover(pkt(321:400), lltfChanEst, noiseEst, chanBW);
    if ~lsigFail
        [~, info.lsigLen] = decodeLSIGBits(lsigBits);
    end
    if info.lsigLen > 0
        rxTimeUs = 4 * ceil((info.lsigLen + 3) / 3) + 20;
        consumed = max(consumed, rxTimeUs * 20);   % 20 samples/us
    end

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
    % (info.lsigLen は関数の先頭で既に取得済み)

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

    % --- APEPLength の候補を両方とも実際に復号して試す ---
    % L-SIG 由来 (RXTIME の丸めを含む概算値) と VHT-SIG-B 由来 (SU-VHT では
    % 本来こちらが正確な値のはずだが、SIG-B はデータ部と同じ MCS で送られる
    % ため低SNRでは化けうる) のどちらが正しいかは、実際に復号して MPDU の
    % FCS が通るかを見るまで分からない。旧版は「バッファに収まる方を機械的に
    % 採用する」だけで、どちらが正しいかを検証していなかった。
    % CSI (vhtChanEst) は VHT-LTF から求めており、この候補選択より前に
    % 確定しているため、複数回試しても CSI には影響しない。
    apepCandidates = unique([info.apepFromLSIG, info.apepFromSIGB], 'stable');
    apepCandidates = apepCandidates(apepCandidates > 0);
    if isempty(apepCandidates)
        info.reason = 'パケット長を決定できず';
        status = 0;
        return;
    end

    triedAny = false;
    for c = 1:numel(apepCandidates)
        try
            cfgTry = wlanVHTConfig('ChannelBandwidth', chanBW, ...
                'NumTransmitAntennas', 1, 'NumSpaceTimeStreams', 1, ...
                'MCS', vhtA.mcs, 'APEPLength', apepCandidates(c), ...
                'ChannelCoding', codingStr, 'GuardInterval', giStr);
            indTry = wlanFieldIndices(cfgTry);
        catch
            continue;   % この候補は不正。次の候補へ
        end
        if numel(pkt) < indTry.VHTData(2)
            continue;   % データがまだ揃っていない
        end
        triedAny = true;
        info.apepUsed = apepCandidates(c);

        % [高速化4] LDPC は早期終了を有効にする (出力は変わらない。詳細は
        %   htVhtDataRecover のコメントを参照)
        [rxPSDU, eqSym, cpe] = vhtDataRecoverWithDiag( ...
            pkt(indTry.VHTData(1):indTry.VHTData(2)), vhtChanEst, noiseEst, cfgTry);

        % --- 受信品質の測定 (SIG は読めるのに FCS が通らない原因の切り分け) ---
        info = measureVHTQuality(info, pkt(1:indTry.VHTData(2)), lltfChanEst, noiseEst, ...
            eqSym, cpe, vhtA.mcs);

        % --- VHT の PSDU は A-MPDU なので分解してから MPDU を復号 ---
        [cfgMAC, payload, ok, deagInfo] = decodeAggregatedPSDU(rxPSDU, cfgTry);
        cfgVHT = cfgTry;
        ind    = indTry;
        info.mpduCount  = deagInfo.mpduCount;
        info.deagStatus = deagInfo.status;
        info.nonBinary  = deagInfo.nonBinary;
        info.dataClass  = deagInfo.dataClass;
        info.dataRange  = deagInfo.dataRange;

        if ok
            break;   % FCS が通った。以降の候補は試さない
        end
    end

    if ~triedAny
        info.reason = sprintf('APEPLength不正/長さ不足(LSIG=%d,SIGB=%d)', ...
            info.apepFromLSIG, info.apepFromSIGB);
        status = -1;   % データがまだ揃っていない可能性もある
        return;
    end

    consumed = double(ind.VHTData(2));

    if ~ok
        info.reason = sprintf('MPDU復号失敗(分解=%s,MPDU数=%d)', ...
            deagInfo.status, deagInfo.mpduCount);
        if ~keepPHYOnly
            status = 0;
            return;
        end
        % MAC は読めなかったが PHY 構成は対応範囲内。CSI は VHT-LTF から
        % 求めており MPDU 復号の成否と無関係なので、擬似アドレスの下に残す。
        info.csiKept = true;
        entry = buildPHYOnlyEntry('VHT', vhtChanEst, vhtA.mcs);
        entry.mpduCount = max(deagInfo.mpduCount, 1);
        status = 1;
        return;
    end

    entry = buildEntry(cfgMAC, payload, 'VHT', vhtChanEst, vhtA.mcs);
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
    %
    % 警告の抑止は呼び出し側でループ外にまとめて行っている
    % (状態の保存・復元はコストが高く、MPDU ごとだと回数が多すぎるため)。
    for m = 1:numel(mpduList)
        dfmt = mpduDataFormat(mpduList{m});
        if isempty(dfmt)
            continue;   % 判別できない形式
        end
        try
            [c, p, st] = wlanMPDUDecode(mpduList{m}, cfgFormat, 'DataFormat', dfmt);
            if strcmpi(string(st), "Success")
                cfgMAC = c; payload = p; ok = true;
                return;
            end
        catch
            % この MPDU は読み飛ばす
        end
    end

    % A-MPDU として分解できなかった場合は単一 MPDU として試す
    if isempty(mpduList)
        dfmt = mpduDataFormat(rxPSDU);
        if ~isempty(dfmt)
            try
                [c, p, st] = wlanMPDUDecode(rxPSDU, cfgFormat, 'DataFormat', dfmt);
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

function entry = buildEntry(cfgMAC, payload, phyFormat, chanEst, mcs)
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
        'mcs',         mcs, ...          % 注入トラフィックの選別に使う
        'fcsVerified', true, ...         % HT/VHT のヘッダのみ復号時は呼び出し側で false
        'macDecoded',  true, ...         % MAC ヘッダまで読めた
        'mpduCount',   1, ...            % 集約されている場合は呼び出し側で上書き
        'csi',         chanEst(:).');
end

function entry = buildPHYOnlyEntry(phyFormat, chanEst, mcs)
    % MAC を復号できなかったパケット用の記録を作る。
    %
    % CSI (chanEst) は VHT-LTF から求めたものであり、MPDU 復号の成否とは
    % 無関係に有効である。送信元 MAC は分からないので、PHY パラメータから
    % 作った擬似アドレスを bssid に入れて「同じ PHY 設定で送っている送信元」
    % としてまとめる。PicoScenes の injector は PHY 設定を固定して送るため、
    % このグループがそのまま注入トラフィックに対応する。
    %
    % 擬似アドレスに MCS を含めるのは、周囲の AP の VHT トラフィック
    % (MCS がばらつく) と注入分 (MCS 固定) を分けるため。
    entry = struct( ...
        'timeSec',     0, ...
        'bssid',       phyOnlyAddress(phyFormat, mcs), ...
        'addr1',       '', ...
        'frameType',   '(MAC復号不可)', ...
        'ssid',        '', ...
        'phyFormat',   phyFormat, ...
        'mcs',         mcs, ...
        'fcsVerified', false, ...
        'macDecoded',  false, ...
        'mpduCount',   1, ...
        'csi',         chanEst(:).');
end

function addr = phyOnlyAddress(phyFormat, mcs)
    % MAC 不明のパケットをまとめる擬似アドレス。
    % targetTxAddress にこの文字列をそのまま指定すれば、明示的に
    % 「MAC は読めないが PHY が一致するパケット」だけを選べる。
    addr = sprintf('(PHY:%s-MCS%d)', phyFormat, mcs);
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

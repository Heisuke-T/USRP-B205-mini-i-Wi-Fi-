function info = slimCSI(inFile, outFile, varargin)
%SLIMCSI  デコード済み CSI の .mat を、解析に必要な変数だけに絞って縮小する。
%
%   info = SLIMCSI(inFile, outFile)
%   info = SLIMCSI(inFile, outFile, 'Format', 'HE', 'MaxPackets', 8000)
%
%   decodeIQ_HE.m / decodeIQ_VHT.m の出力は、生の測定 30 秒で 50MB を超える。
%   内訳を調べると次の3つが大半を占める。
%
%     csi<FMT>   … 本体
%     csi        … 主データの複製 (ResultCSI.m 互換のために同じ内容を二重保存)
%     csiNonHT   … Non-HT パケット (SHARP の処理には使わない)
%
%   本関数は「使う形式ひとつ分」だけを残し、CSI を single 精度に落とす。
%   複素チャネル推定値に double の精度は不要なため、情報はほぼ失われない。
%   実測では 55.8MB → 約 8MB になる。
%
%   用途:
%     * GitHub に置ける大きさの標本を作る (50MB/100MB 制限の回避)
%     * 長時間キャプチャから解析対象の区間だけ抜き出す
%
%   出力は SHARP-USRP の手順0 (usrp_to_sharp.py / usrpCSItoSHARP.m) が
%   そのまま読める変数名を保つ。
%
%   名前と値の引数:
%     'Format'     : 'auto' (既定) | 'VHT' | 'HE' | 'NonHT'
%                    auto はパケット数が最多の形式を選ぶ。
%     'MaxPackets' : 残す最大パケット数 (既定 Inf)。
%     'StartPacket': 何番目のパケットから残すか (既定 1)。
%     'Precision'  : 'single' (既定) | 'double'
%     'FCSOnly'    : false (既定) | true   FCS 検証済みのみ残す
%
%   例:
%     % 30秒キャプチャを丸ごと縮小
%     slimCSI('D:\IQ_csi\09272233_WAX202_CSI.mat', 'CSI_ax\09272233_sample.mat');
%
%     % 長時間キャプチャの先頭 8000 パケットだけ
%     slimCSI('D:\IQ_csi\09272233_WAX202_CSI.mat', 'CSI_ax\09272233_sample.mat', ...
%             'MaxPackets', 8000);
%
%   Copyright (C) 2026 Heisuke Takeda
%   Released under the GNU GPL v3 (see LICENSE).

p = inputParser;
addParameter(p, 'Format', 'auto', @(x) any(strcmpi(x, {'auto','VHT','HE','NonHT'})));
addParameter(p, 'MaxPackets', Inf, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(p, 'StartPacket', 1, @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(p, 'Precision', 'single', @(x) any(strcmpi(x, {'single','double'})));
addParameter(p, 'FCSOnly', false, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});
fmtOpt     = lower(p.Results.Format);
maxPackets = p.Results.MaxPackets;
startPkt   = round(p.Results.StartPacket);
precision  = lower(p.Results.Precision);
fcsOnly    = logical(p.Results.FCSOnly);

% 形式ごとの変数名 (デコーダの出力に合わせる)
V = struct( ...
    'VHT',   struct('csi','csiVHT',  'sub','subcarrierIndicesVHT20', ...
                    'time','timeSecVHT',  'fcs','fcsVHT',  'frame','frameTypeVHT'), ...
    'HE',    struct('csi','csiHE',   'sub','subcarrierIndicesHE20', ...
                    'time','timeSecHE',   'fcs','fcsHE',   'frame','frameTypeHE'), ...
    'NonHT', struct('csi','csiNonHT','sub','subcarrierIndicesNonHT', ...
                    'time','timeSecNonHT','fcs','fcsNonHT','frame','frameTypeNonHT'));

fprintf('入力: %s\n', inFile);
d = dir(inFile);
if isempty(d)
    error('slimCSI:notFound', 'ファイルが見つかりません: %s', inFile);
end
inBytes = d.bytes;
fprintf('  元サイズ: %.1f MB\n', inBytes / 1e6);

S = load(inFile);

%% 形式を決める
% auto の候補に NonHT は入れない。ビーコン等の Non-HT フレームは数だけ多く、
% パケット数最多で選ぶと解析対象の HE / VHT を取り逃がすため
% (実測: HE 4956 に対し NonHT 10505)。NonHT は明示指定でのみ選べる。
cands = {'HE', 'VHT'};
if strcmp(fmtOpt, 'auto')
    fmt = ''; best = 0;
    for ii = 1:numel(cands)
        v = V.(cands{ii}).csi;
        if isfield(S, v) && ~isempty(S.(v)) && size(S.(v), 1) > best
            best = size(S.(v), 1);
            fmt  = cands{ii};
        end
    end
    if isempty(fmt)
        error('slimCSI:noCSI', ...
            'csiHE / csiVHT / csiNonHT のいずれも見つかりません。');
    end
else
    switch fmtOpt
        case 'vht',   fmt = 'VHT';
        case 'he',    fmt = 'HE';
        case 'nonht', fmt = 'NonHT';
    end
    if ~isfield(S, V.(fmt).csi) || isempty(S.(V.(fmt).csi))
        error('slimCSI:formatMissing', ...
            '指定された形式 %s のデータがありません。', fmt);
    end
end
vn = V.(fmt);
fprintf('  形式: %s\n', fmt);

%% 取り出し
csi = S.(vn.csi);                       % [パケット数 x サブキャリア数]
nAll = size(csi, 1);

getVec = @(name) localGetVec(S, name, nAll);
timeSec = getVec(vn.time);
fcs     = getVec(vn.fcs);

frameType = [];
if isfield(S, vn.frame) && numel(S.(vn.frame)) == nAll
    frameType = S.(vn.frame);
end

%% パケットの選別
keep = true(nAll, 1);

bad = (sum(abs(csi), 2) == 0) | ~all(isfinite(csi), 2);
if any(bad)
    fprintf('  除外: 全ゼロ/非有限 %d パケット\n', sum(bad));
end
keep = keep & ~bad;

if fcsOnly
    if isempty(fcs)
        warning('slimCSI:noFCS', 'FCS 情報が無いため FCSOnly を無視します。');
    else
        fprintf('  除外: FCS 未検証 %d パケット\n', sum(keep & ~logical(fcs(:))));
        keep = keep & logical(fcs(:));
    end
end

idx = find(keep);
if startPkt > 1
    idx = idx(min(startPkt, numel(idx)+1):end);
end
if numel(idx) > maxPackets
    idx = idx(1:maxPackets);
    fprintf('  先頭 %d パケットに制限\n', maxPackets);
end
if isempty(idx)
    error('slimCSI:noPackets', 'パケットが残りませんでした。');
end

csi = csi(idx, :);
if ~isempty(timeSec),   timeSec   = timeSec(idx);   end
if ~isempty(fcs),       fcs       = fcs(idx);       end
if ~isempty(frameType), frameType = frameType(idx); end

fprintf('  採用: %d / %d パケット x %d サブキャリア\n', ...
    size(csi, 1), nAll, size(csi, 2));

%% 精度を落とす
if strcmp(precision, 'single')
    csi = complex(single(real(csi)), single(imag(csi)));
end

%% csiMeta は嵩む項目を落として引き継ぐ
csiMeta = struct();
if isfield(S, 'csiMeta') && isstruct(S.csiMeta)
    src = S.csiMeta;
    % 解析に要る軽いフィールドだけ残す (captureMeta / decodeStats は大きい)
    wanted = {'centerFrequency','sampleRate','gain','wifiChannel', ...
              'captureDuration','captureDatetime','decodeDatetime', ...
              'primaryFormat','platform','serialNum','description','decodedBy'};
    for ii = 1:numel(wanted)
        if isfield(src, wanted{ii})
            csiMeta.(wanted{ii}) = src.(wanted{ii});
        end
    end
end
csiMeta.slimmedBy      = 'slimCSI.m';
csiMeta.slimmedFormat  = fmt;
csiMeta.slimmedFrom    = inFile;
csiMeta.slimPrecision  = precision;
csiMeta.originalPackets = nAll;

%% 保存 (デコーダと同じ変数名を保つ)
out = struct();
out.(vn.csi)  = csi;
out.(vn.sub)  = double(S.(vn.sub));
if ~isempty(timeSec),   out.(vn.time)  = timeSec(:).';  end
if ~isempty(fcs),       out.(vn.fcs)   = fcs(:).';      end
if ~isempty(frameType), out.(vn.frame) = frameType;     end
out.csiMeta = csiMeta;

% 別名の csi / subcarrierIndices / timeSec は保存しない。元ファイルで容量の
% 3分の1を占めていた重複がこれで、SHARP-USRP の手順0 は csiHE / csiVHT を
% 直接見るため不要。ResultCSI.m に掛けたい場合は元ファイルを使うこと。

outDir = fileparts(outFile);
if ~isempty(outDir) && ~isfolder(outDir)
    mkdir(outDir);
end
% -v7 で保存する。縮小後は数 MB なので v7 の容量制限に当たらず、圧縮も効き、
% Python 側が h5py 無しで (scipy.io.loadmat だけで) 読めるようになる。
save(outFile, '-struct', 'out', '-v7');

d2 = dir(outFile);
fprintf('  新サイズ: %.1f MB (%.0f%% 削減)\n', ...
    d2.bytes / 1e6, 100 * (1 - d2.bytes / inBytes));
fprintf('出力: %s\n', outFile);

info = struct('format', fmt, 'nPackets', size(csi, 1), ...
              'inBytes', inBytes, 'outBytes', d2.bytes, 'outFile', outFile);
end

% -----------------------------------------------------------------------
function v = localGetVec(S, name, nExpected)
%LOCALGETVEC  長さが合う数値ベクトルなら取り出す (無ければ空)。
v = [];
if isfield(S, name)
    raw = S.(name);
    if numel(raw) == nExpected
        v = raw(:);
    end
end
end

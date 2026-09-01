function info = usrpCSItoSHARP(inFile, outFile, varargin)
%USRPCSITOSHARP  USRP のデコード済み CSI を SHARP 位相サニタイゼーションの
%                入力形式へ変換する (Python 版 usrp_to_sharp.py と同一仕様)。
%
%   info = USRPCSITOSHARP(inFile, outFile)
%   info = USRPCSITOSHARP(inFile, outFile, 'Format', 'VHT', 'FCSOnly', true)
%
%   入力  inFile  : decodeIQ_VHT.m / decodeIQ_HE.m が出力した .mat
%   出力  outFile : SHARP-USRP の Python 側が読む .mat
%
%   名前と値の引数:
%     'Format'  : 'auto' (既定) | 'VHT' | 'HE'
%                 auto はパケット数が多いほうを選ぶ。
%     'FCSOnly' : false (既定) | true
%                 true なら FCS 検証済みパケットのみ使う。
%
%   出力ファイルに入る主な変数:
%     csi_buff           [パケット数 x FFT長] complex, FFT 順
%     sharp_config       'VHT20' または 'HE20'
%     subcarrier_indices 使用サブキャリア番号
%     time_sec           パケット取得時刻 [s]
%
%   重要: 保存は必ず '-v7' 形式で行う。Python の scipy.io.loadmat は
%   MATLAB v7.3 (HDF5) を読めないため。
%
%   Copyright (C) 2026 Heisuke Takeda
%   Based on SHARP (C) 2022 Francesca Meneghello, GNU GPL v3.
%
%   This program is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 3 of the License, or
%   (at your option) any later version.  See <https://www.gnu.org/licenses/>.

p = inputParser;
addParameter(p, 'Format', 'auto', @(x) any(strcmpi(x, {'auto', 'VHT', 'HE'})));
addParameter(p, 'FCSOnly', false, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});
fmtOpt  = lower(p.Results.Format);
fcsOnly = logical(p.Results.FCSOnly);

%% ------------------------------------------------------------------
%  規格ごとの OFDM 構成 (Python 側 wifi_config.py と同じ定義)
%  ------------------------------------------------------------------
cfgTable = struct( ...
    'VHT20', struct('standard', 'VHT', 'fftSize', 64,  'deltaF', 312.5e3, ...
                    'occupied', [-28:-1, 1:28], 'bandwidthMHz', 20), ...
    'HE20',  struct('standard', 'HE',  'fftSize', 256, 'deltaF', 78.125e3, ...
                    'occupied', [-122:-2, 2:122], 'bandwidthMHz', 20));

fprintf('入力: %s\n', inFile);
S = load(inFile);

%% ------------------------------------------------------------------
%  使用する PHY 形式を決める
%  ------------------------------------------------------------------
candidates = {'VHT', 'HE'};
varNames = struct( ...
    'VHT', struct('csi', 'csiVHT', 'sub', 'subcarrierIndicesVHT20', ...
                  'time', 'timeSecVHT', 'fcs', 'fcsVHT', 'cfg', 'VHT20'), ...
    'HE',  struct('csi', 'csiHE',  'sub', 'subcarrierIndicesHE20', ...
                  'time', 'timeSecHE',  'fcs', 'fcsHE',  'cfg', 'HE20'));

if strcmp(fmtOpt, 'auto')
    bestFmt = ''; bestN = 0;
    for ii = 1:numel(candidates)
        f = candidates{ii};
        v = varNames.(f).csi;
        if isfield(S, v) && ~isempty(S.(v)) && size(S.(v), 1) > bestN
            bestN = size(S.(v), 1);
            bestFmt = f;
        end
    end
    if isempty(bestFmt)
        error('usrpCSItoSHARP:noCSI', ...
            ['csiVHT / csiHE が見つかりません。decodeIQ_VHT.m または ', ...
             'decodeIQ_HE.m の出力を指定してください。']);
    end
    fmt = bestFmt;
else
    fmt = upper(fmtOpt);
    if ~isfield(S, varNames.(fmt).csi) || isempty(S.(varNames.(fmt).csi))
        error('usrpCSItoSHARP:formatMissing', ...
            '指定された形式 %s のデータがファイルにありません。', fmt);
    end
end

vn  = varNames.(fmt);
cfg = cfgTable.(vn.cfg);
fprintf('形式: %s -> 設定 %s (%d MHz, FFT長 %d)\n', ...
    fmt, vn.cfg, cfg.bandwidthMHz, cfg.fftSize);

%% ------------------------------------------------------------------
%  CSI・サブキャリア番号・時刻・FCS の取り出し
%  ------------------------------------------------------------------
csi = S.(vn.csi);                       % [パケット数 x サブキャリア数]
if ~ismatrix(csi) || isempty(csi)
    error('usrpCSItoSHARP:badCSI', '%s が空か 2 次元ではありません。', vn.csi);
end
fprintf('  読み込み: %d パケット x %d サブキャリア\n', size(csi, 1), size(csi, 2));

if ~isfield(S, vn.sub)
    error('usrpCSItoSHARP:noSubcarriers', ...
        'サブキャリア番号 %s がファイルにありません。', vn.sub);
end
subRaw = S.(vn.sub);
sub = double(subRaw(:)).';

timeSec = [];
if isfield(S, vn.time)
    timeRaw = S.(vn.time);
    if numel(timeRaw) == size(csi, 1)
        timeSec = double(timeRaw(:)).';
    end
end

fcs = [];
if isfield(S, vn.fcs)
    fcsRaw = S.(vn.fcs);
    if numel(fcsRaw) == size(csi, 1)
        fcs = logical(fcsRaw(:)).';
    end
end

%% ------------------------------------------------------------------
%  無効パケットの除去
%  ------------------------------------------------------------------
keep = true(1, size(csi, 1));

bad = (sum(abs(csi), 2).' == 0) | ~all(isfinite(csi), 2).';
if any(bad)
    fprintf('  除外: 全ゼロ/非有限の CSI %d パケット\n', sum(bad));
end
keep = keep & ~bad;

if fcsOnly
    if isempty(fcs)
        warning('usrpCSItoSHARP:noFCS', ...
            'FCS 情報が無いため FCSOnly を無視します。');
    else
        dropped = sum(keep & ~fcs);
        if dropped > 0
            fprintf('  除外: FCS 未検証 %d パケット\n', dropped);
        end
        keep = keep & fcs;
    end
end

csi = csi(keep, :);
if ~isempty(timeSec)
    timeSec = timeSec(keep);
end
fprintf('  採用パケット数: %d\n', size(csi, 1));
if isempty(csi)
    error('usrpCSItoSHARP:noPackets', '有効なパケットが残りませんでした。');
end

%% ------------------------------------------------------------------
%  サブキャリア構成の検証
%  ------------------------------------------------------------------
if numel(sub) ~= size(csi, 2)
    error('usrpCSItoSHARP:sizeMismatch', ...
        ['サブキャリア番号の数 (%d) と CSI の列数 (%d) が一致しません。'], ...
        numel(sub), size(csi, 2));
end
if ~isequal(sort(sub(:)).', sort(cfg.occupied(:)).')
    error('usrpCSItoSHARP:subcarrierMismatch', ...
        ['%s の使用サブキャリア構成と一致しません ', ...
         '(期待 %d 本, 実際 %d 本)。20MHz / SISO のデータか確認してください。'], ...
        vn.cfg, numel(cfg.occupied), numel(sub));
end

%% ------------------------------------------------------------------
%  csi_buff の構築 (中心寄せに並べてから FFT 順へ)
%  ------------------------------------------------------------------
half = cfg.fftSize / 2;
centered = zeros(size(csi, 1), cfg.fftSize);
centered(:, sub + half + 1) = csi;      % MATLAB は 1 始まりのため +1

% 中心寄せ -> FFT 順 (Nexmon と同じ並び。Python 側の fftshift で戻る)
csi_buff = ifftshift(centered, 2); %#ok<NASGU>

%% ------------------------------------------------------------------
%  取得レートの統計 (Doppler 解析時の判断材料)
%  ------------------------------------------------------------------
stats = struct();
if numel(timeSec) >= 2
    dt = diff(sort(timeSec));
    dt = dt(dt > 0);
    if ~isempty(dt)
        stats.meanRate         = 1 / mean(dt);
        stats.medianIntervalMs = median(dt) * 1e3;
        stats.minIntervalMs    = min(dt) * 1e3;
        stats.maxIntervalMs    = max(dt) * 1e3;
        stats.cv               = std(dt) / mean(dt);
        fprintf(['  取得レート: 平均 %.1f pkt/s ', ...
                 '(間隔 中央値 %.2f ms, %.2f..%.2f ms, 変動係数 %.2f)\n'], ...
            stats.meanRate, stats.medianIntervalMs, ...
            stats.minIntervalMs, stats.maxIntervalMs, stats.cv);
        if stats.cv > 0.2
            fprintf(['  注意: パケット間隔のばらつきが大きく、等間隔サンプリングを\n', ...
                     '        前提とする Doppler 解析にはリサンプリングが必要です。\n', ...
                     '        (位相サニタイゼーション自体はパケット単位処理のため影響なし)\n']);
        end
    end
end

%% ------------------------------------------------------------------
%  保存 (必ず -v7。Python の scipy.io.loadmat は v7.3 を読めない)
%  ------------------------------------------------------------------
sharp_config       = vn.cfg;             %#ok<NASGU>
standard           = cfg.standard;       %#ok<NASGU>
bandwidth_mhz      = cfg.bandwidthMHz;   %#ok<NASGU>
fft_size           = cfg.fftSize;        %#ok<NASGU>
delta_f            = cfg.deltaF;         %#ok<NASGU>
subcarrier_indices = double(cfg.occupied);  %#ok<NASGU>
n_ss               = 1;                  %#ok<NASGU>
n_core             = 1;                  %#ok<NASGU>
[~, srcName, srcExt] = fileparts(inFile);
source_file        = [srcName srcExt];   %#ok<NASGU>
time_sec           = timeSec;            %#ok<NASGU>

outDir = fileparts(outFile);
if ~isempty(outDir) && ~isfolder(outDir)
    mkdir(outDir);
end

saveVars = {'csi_buff', 'sharp_config', 'standard', 'bandwidth_mhz', ...
            'fft_size', 'delta_f', 'subcarrier_indices', ...
            'n_ss', 'n_core', 'source_file'};
if ~isempty(time_sec)
    saveVars{end+1} = 'time_sec';
end

save(outFile, saveVars{:}, '-v7');

fprintf('  csi_buff: [%d x %d] (パケット x FFT長, FFT順)\n', ...
    size(csi_buff, 1), size(csi_buff, 2));
fprintf('出力: %s\n', outFile);

info = struct('config', vn.cfg, 'format', fmt, ...
              'nPackets', size(csi_buff, 1), 'stats', stats, ...
              'outFile', outFile);
end

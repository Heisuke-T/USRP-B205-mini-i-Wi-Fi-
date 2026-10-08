%% decodeIQ_HE_batch.m
% =========================================================================
%  IQ_raw フォルダにある生IQをまとめて decodeIQ_HE.m で復号し、
%  結果をすべて IQ_csi フォルダに保存する
% -------------------------------------------------------------------------
%  概要:
%    hddInputPath にある生IQファイルを1つずつ decodeIQ_HE.m に渡して復号する。
%    復号の中身は decodeIQ_HE.m とまったく同じ (このファイルは呼び出し役)。
%    復号のパラメータ (pktDetThreshold、useBSSColorFallback など) を
%    変えたいときは decodeIQ_HE.m 側を編集すること。
%
%    途中のファイルで失敗しても止まらずに次へ進み、最後に一覧で報告する。
%    夜間などに放置して回す用途を想定している。
%
%  対象になるファイル:
%    *_raw.mat … captureIQ.m / captureIQ10.m / captureIQ15.m の出力
%    *_raw.bin … captureIQ_single.m の出力 (同名の *_rawmeta.mat が必要)
%    *_segNN_raw.mat … captureIQ_single.m で分割したセグメント。
%                      ただし元の *_raw.bin が残っている場合は対象外
%                      (.bin を丸ごと復号すれば済むので二重に処理しない)。
%
%  出力ファイル名:
%    入力ファイル名から決める。
%      10081530_32275DE_raw.mat  ->  10081530_32275DE_<SSID>_CSI.mat
%      09272233_raw.bin          ->  09272233_<SSID>_CSI.mat
%      09272233_seg03_raw.mat    ->  09272233_seg03_<SSID>_CSI.mat
%    分割セグメントは mergeCSI.m でそのまま結合できる名前になる。
%
%  すでに出力があるファイル:
%    skipExisting = true (既定) なら飛ばす。復号し直したいときは、
%    該当の *_CSI.mat を消すか、skipExisting = false にする。
%
%  ログ:
%    hddSavePath に次のものを残す。
%      <出力名>_log.txt          … 生IQ ファイルごとの復号結果。decodeIQ_HE.m を
%                                  単体で実行したときに画面に出る内容と同じ
%                                  (読み込んだ生IQ・復号サマリ・ネットワーク
%                                  一覧など)。*_CSI.mat の隣にできる。
%      batch_<日時>.log          … 一括処理全体の進行 (計画・進捗・最終結果)
%      batch_<日時>_summary.csv  … ファイルごとの成否・所要時間・件数
%    showEachPacket = false (既定) なら、パケット 1 個ごとの行は出さないので
%    ファイルごとのログは百数十行程度に収まる。
%
%  途中で止めたいとき:
%    Ctrl+C で止めてよい。処理中のファイルの出力は作られない (次回
%    skipExisting = true で実行すれば、そのファイルから再開される)。
% =========================================================================

clear; clc;

%% ------------------------------------------------------------------------
%  1. ユーザ設定パラメータ
%  ------------------------------------------------------------------------
cfg = struct();

% --- 入力元 / 出力先 ----------------------------------------------------
cfg.hddInputPath = 'D:\IQ_raw';
cfg.hddSavePath  = 'D:\IQ_csi';
cfg.usbSavePath  = '';            % USB にも保存するならドライブを書く (例 'F:\IQ')

% --- 抽出したい Wi-Fi の SSID --------------------------------------------
cfg.targetSSID = 'WAX202';

% --- すでに出力があるファイルを飛ばすか ----------------------------------
cfg.skipExisting = true;

% --- 復号できたパケットを 1 行ずつ表示するか ------------------------------
%     false にすると、ファイルごとのログ (*_CSI_log.txt) が復号結果の
%     サマリ中心の読みやすい分量 (百数十行) になる。true だと 1 ファイル
%     あたり数千行になるが、パケット単位で追いかけたいときに使う。
cfg.showEachPacket = false;

% --- 呼び出す復号スクリプト (通常は変更不要) -----------------------------
%     このファイルと同じフォルダの decodeIQ_HE.m を使う。
cfg.decoderScript = fullfile(fileparts(mfilename('fullpath')), 'decodeIQ_HE.m');

%% ------------------------------------------------------------------------
%  2. 実行
%  ------------------------------------------------------------------------
runBatch(cfg);

%% ------------------------------------------------------------------------
%  ローカル関数
%  ------------------------------------------------------------------------
function runBatch(cfg)
    % 一括処理の本体。関数にしてあるのは、Ctrl+C で止めたときにも
    % onCleanup でログの記録 (diary) を確実に閉じるため。

    if ~isfile(cfg.decoderScript)
        error('decodeIQ_HE_batch:noDecoder', ...
            '復号スクリプトが見つかりません: %s', cfg.decoderScript);
    end
    if ~exist(cfg.hddInputPath, 'dir')
        error('decodeIQ_HE_batch:noInputDir', ...
            '入力フォルダがありません: %s', cfg.hddInputPath);
    end
    if ~exist(cfg.hddSavePath, 'dir')
        [ok, msg] = mkdir(cfg.hddSavePath);
        if ~ok
            error('decodeIQ_HE_batch:mkdirFailed', ...
                '出力フォルダを作成できませんでした (%s): %s', cfg.hddSavePath, msg);
        end
    end

    % 前回の一括処理が異常終了していた場合に備え、受け渡し領域を空にしておく
    clearBatchOverride();

    % --- ログ (画面の内容をすべてファイルにも残す) ---
    runTag  = datestr(now, 'yyyymmdd_HHMMSS');
    logFile = fullfile(cfg.hddSavePath, ['batch_' runTag '.log']);
    csvFile = fullfile(cfg.hddSavePath, ['batch_' runTag '_summary.csv']);
    diary(logFile);
    diaryCleanup = onCleanup(@() diary('off')); %#ok<NASGU>

    % --- 対象ファイルの洗い出しと計画 ---
    jobs = planJobs(cfg);
    if isempty(jobs)
        fprintf('復号対象のファイルがありません: %s\n', cfg.hddInputPath);
        return;
    end
    printPlan(jobs, cfg);

    todo = find(~[jobs.skip]);
    if isempty(todo)
        fprintf('\nすべて復号済みです。やり直す場合は skipExisting = false にしてください。\n');
        writeSummaryCsv(csvFile, jobs);
        return;
    end

    % --- 1ファイルずつ復号 ---
    batchTic   = tic;
    doneCapSec = 0;     % 処理済みのキャプチャ秒数 (残り時間の推定に使う)
    totalCapSec = sum([jobs(todo).captureSec], 'omitnan');

    for t = 1:numel(todo)
        j = todo(t);
        fprintf('\n');
        fprintf('#########################################################################\n');
        fprintf('# [%d/%d] %s\n', t, numel(todo), jobs(j).name);
        fprintf('#   出力: %s\n', jobs(j).outName);
        fprintf('#   ログ: %s\n', jobs(j).logName);
        fprintf('#########################################################################\n');

        fileTic = tic;
        [ok, errMsg] = decodeOneFile(jobs(j), cfg);
        jobs(j).elapsedSec = toc(fileTic);

        if ok
            jobs(j).status = 'OK';
            jobs(j).counts = readOutputCounts(fullfile(cfg.hddSavePath, jobs(j).outName));
        else
            jobs(j).status = 'FAIL';
            jobs(j).errMsg = errMsg;
            fprintf(2, '\n[一括処理] 失敗しました: %s\n  %s\n', jobs(j).name, errMsg);
        end

        % 残り時間の推定 (復号時間はキャプチャ長にほぼ比例する)
        if ~isnan(jobs(j).captureSec)
            doneCapSec = doneCapSec + jobs(j).captureSec;
        end
        elapsedAll = toc(batchTic);
        fprintf('\n[一括処理] %d/%d 件目が終了 (%s, %.0f 秒)', ...
            t, numel(todo), jobs(j).status, jobs(j).elapsedSec);
        if doneCapSec > 0 && totalCapSec > doneCapSec
            remainSec = elapsedAll / doneCapSec * (totalCapSec - doneCapSec);
            fprintf('  残り推定 %s', formatDuration(remainSec));
        end
        fprintf('\n');

        fprintf('[一括処理] このファイルの復号結果: %s\n', ...
            fullfile(cfg.hddSavePath, jobs(j).logName));

        % 途中経過も毎回書き出しておく (途中で止めても結果が残るように)
        writeSummaryCsv(csvFile, jobs);
    end

    % --- 最終報告 ---
    printSummary(jobs, toc(batchTic));
    writeSummaryCsv(csvFile, jobs);
    fprintf('\n一括処理のログ       : %s\n', logFile);
    fprintf('結果の一覧 (CSV)     : %s\n', csvFile);
    fprintf('ファイルごとの復号結果: %s\\*_CSI_log.txt\n', cfg.hddSavePath);
end

function jobs = planJobs(cfg)
    % 入力フォルダから復号対象を集め、出力名とスキップの要否を決める。
    binList = dir(fullfile(cfg.hddInputPath, '*_raw.bin'));
    matList = dir(fullfile(cfg.hddInputPath, '*_raw.mat'));
    allList = [binList; matList];

    % 分割セグメントは、元の .bin が残っていれば対象外にする
    binBases = regexprep({binList.name}, '_raw\.bin$', '');
    keep = true(numel(allList), 1);
    for k = 1:numel(allList)
        tok = regexp(allList(k).name, '^(.*)_seg\d+_raw\.mat$', 'tokens', 'once');
        if ~isempty(tok) && any(strcmp(binBases, tok{1}))
            keep(k) = false;
        end
    end
    allList = allList(keep);

    % ファイル名の日時順に並べる
    [~, order] = sort({allList.name});
    allList = allList(order);

    ssidSafe = regexprep(cfg.targetSSID, '[^A-Za-z0-9_-]', '_');
    jobs = struct('name', {}, 'path', {}, 'ext', {}, 'outName', {}, 'logName', {}, ...
        'captureSec', {}, 'skip', {}, 'status', {}, 'elapsedSec', {}, ...
        'errMsg', {}, 'counts', {});
    for k = 1:numel(allList)
        [~, base, ext] = fileparts(allList(k).name);
        jb = struct();
        jb.name       = allList(k).name;
        jb.path       = fullfile(allList(k).folder, allList(k).name);
        jb.ext        = ext;
        jb.outName    = [regexprep(base, '_raw$', '') '_' ssidSafe '_CSI.mat'];
        jb.logName    = '';    % 出力名が確定してから決める (下)
        jb.captureSec = readCaptureSec(jb.path, ext);
        jb.skip       = false;
        jb.status     = '';
        jb.elapsedSec = NaN;
        jb.errMsg     = '';
        jb.counts     = [];
        jobs(end+1) = jb; %#ok<AGROW>
    end

    % 同じ出力名になる組 (例: 同じ名前の .mat と .bin) は拡張子で区別する。
    % そのままだと後から復号した方が先の結果を上書きしてしまう。
    outNames = {jobs.outName};
    [uniq, ~, idx] = unique(outNames);
    for u = 1:numel(uniq)
        dup = find(idx == u);
        if numel(dup) > 1
            for d = dup(:).'
                tag = strrep(jobs(d).ext, '.', '');
                jobs(d).outName = regexprep(jobs(d).outName, '_CSI\.mat$', ...
                    ['_' tag '_CSI.mat']);
            end
        end
    end

    % ファイルごとのログ名 (decodeIQ_HE.m が出力名から同じ規則で作る)
    for k = 1:numel(jobs)
        jobs(k).logName = regexprep(jobs(k).outName, '\.mat$', '_log.txt');
    end

    % すでに出力があるものを飛ばす
    for k = 1:numel(jobs)
        if cfg.skipExisting && isfile(fullfile(cfg.hddSavePath, jobs(k).outName))
            jobs(k).skip   = true;
            jobs(k).status = 'SKIP';
        end
    end
end

function sec = readCaptureSec(path, ext)
    % 残り時間の推定用に、キャプチャの長さ [s] をメタデータだけから読む。
    % (IQ 本体は読まないので速い。読めなければ NaN)
    sec = NaN;
    try
        if strcmpi(ext, '.bin')
            [folder, base] = fileparts(path);
            metaFile = fullfile(folder, [regexprep(base, '_raw$', '') '_rawmeta.mat']);
            M = load(metaFile, 'meta');
        else
            M = load(path, 'meta');
        end
        sec = double(M.meta.totalSamples) / double(M.meta.sampleRate);
    catch
    end
end

function [ok, errMsg] = decodeOneFile(job, cfg)
    % decodeIQ_HE.m を 1 ファイル分実行する。
    %
    % decodeIQ_HE.m は冒頭で clear するので、入力ファイル等は clear で消えない
    % MATLAB 全体の共有領域 (appdata) で渡す。受け渡しが終わったら、エラーや
    % Ctrl+C で抜けた場合も含めて必ず消す (残っていると、後で decodeIQ_HE.m を
    % 単体で実行したときに古い指定が効いてしまうため)。
    ovr = struct();
    ovr.inputRawFile = job.path;
    ovr.hddInputPath = cfg.hddInputPath;
    ovr.hddSavePath  = cfg.hddSavePath;
    ovr.usbSavePath  = cfg.usbSavePath;
    ovr.targetSSID   = cfg.targetSSID;
    ovr.outFileName  = job.outName;
    ovr.showEachPacket = cfg.showEachPacket;
    setappdata(0, 'decodeIQ_HE_batch', ovr);
    ovrCleanup = onCleanup(@() clearBatchOverride()); %#ok<NASGU>

    % decodeIQ_HE.m は内部で一時的に警告を止めている。途中でエラーになると
    % 止めたままになるので、ファイルごとに元へ戻す。
    warnState   = warning;
    warnCleanup = onCleanup(@() warning(warnState)); %#ok<NASGU>

    % decodeIQ_HE.m は画面の内容を *_CSI_log.txt に残すために diary の
    % 出力先を一時的に切り替える。途中でエラーになると切り替わったままに
    % なるので、一括処理のログ (batch_*.log) に必ず戻す。
    prevDiaryFile = get(0, 'DiaryFile');
    prevDiaryOn   = strcmp(get(0, 'Diary'), 'on');
    diaryCleanup  = onCleanup(@() restoreDiary(prevDiaryFile, prevDiaryOn)); %#ok<NASGU>

    try
        runDecoder(cfg.decoderScript);
        ok = true;
        errMsg = '';
    catch ME
        ok = false;
        errMsg = ME.message;
        if ~isempty(ME.stack)
            errMsg = sprintf('%s  (%s, %d 行目)', errMsg, ME.stack(1).name, ME.stack(1).line);
        end
        % この時点ではまだ diary がこのファイルのログ (*_CSI_log.txt) を
        % 向いているので、失敗の理由をそちらにも残しておく。
        fprintf(2, '\n[一括処理] 復号中にエラーが発生しました:\n  %s\n', errMsg);
    end
end

function runDecoder(scriptPath)
    % 復号スクリプトを、この関数の中だけで実行する。
    % decodeIQ_HE.m の clear はこの関数の変数だけを消すので、
    % 一括処理側の状態 (ファイル一覧や進捗) は影響を受けない。
    run(scriptPath);
end

function restoreDiary(prevFile, prevOn)
    diary off;
    if prevOn && ~isempty(prevFile)
        diary(prevFile);
    end
end

function clearBatchOverride()
    if isappdata(0, 'decodeIQ_HE_batch')
        rmappdata(0, 'decodeIQ_HE_batch');
    end
end

function counts = readOutputCounts(outPath)
    % 出力ファイルに入った CSI の件数を、中身を読まずに確認する。
    counts = struct('HE', 0, 'NonHT', 0, 'HT', 0, 'VHT', 0);
    try
        w = whos('-file', outPath);
        names = {'csiHE', 'csiNonHT', 'csiHT', 'csiVHT'};
        keysOut = {'HE', 'NonHT', 'HT', 'VHT'};
        for k = 1:numel(names)
            hit = strcmp({w.name}, names{k});
            if any(hit)
                sz = w(hit).size;
                if ~isempty(sz) && prod(sz) > 0
                    counts.(keysOut{k}) = sz(1);
                end
            end
        end
    catch
    end
end

function printPlan(jobs, cfg)
    nSkip = sum([jobs.skip]);
    nTodo = numel(jobs) - nSkip;
    capTodo = sum([jobs(~[jobs.skip]).captureSec], 'omitnan');
    fprintf('一括復号の計画\n');
    fprintf('  入力フォルダ : %s\n', cfg.hddInputPath);
    fprintf('  出力フォルダ : %s\n', cfg.hddSavePath);
    fprintf('  対象 SSID    : %s\n', cfg.targetSSID);
    fprintf('  見つかった数 : %d 件 (うち復号済みで飛ばす %d 件)\n', numel(jobs), nSkip);
    fprintf('  これから復号 : %d 件, キャプチャ合計 %.0f 秒\n', nTodo, capTodo);
    fprintf('  ※5 秒のキャプチャで約 3〜5 分かかる (電波の混雑度に依存)。\n');
    fprintf('    目安としてキャプチャ長の 40〜60 倍程度。\n\n');
    for k = 1:numel(jobs)
        if jobs(k).skip
            mark = '済';
        else
            mark = '  ';
        end
        if isnan(jobs(k).captureSec)
            capStr = '   ? s';
        else
            capStr = sprintf('%5.1f s', jobs(k).captureSec);
        end
        fprintf('  %s %-40s %s -> %s\n', mark, jobs(k).name, capStr, jobs(k).outName);
    end
end

function printSummary(jobs, totalSec)
    fprintf('\n');
    fprintf('=========================================================================\n');
    fprintf(' 一括復号の結果 (所要 %s)\n', formatDuration(totalSec));
    fprintf('=========================================================================\n');
    for k = 1:numel(jobs)
        j = jobs(k);
        switch j.status
            case 'OK'
                c = j.counts;
                if isempty(c)
                    cntStr = '';
                else
                    cntStr = sprintf('HE=%d, Non-HT=%d, HT=%d, VHT=%d', ...
                        c.HE, c.NonHT, c.HT, c.VHT);
                end
                fprintf('  OK    %-40s %7.0f 秒  %s\n', j.name, j.elapsedSec, cntStr);
            case 'FAIL'
                fprintf('  FAIL  %-40s %7.0f 秒  %s\n', j.name, j.elapsedSec, j.errMsg);
            case 'SKIP'
                fprintf('  SKIP  %-40s (復号済み)\n', j.name);
            otherwise
                fprintf('  ----  %-40s (未処理)\n', j.name);
        end
    end
    nOK   = sum(strcmp({jobs.status}, 'OK'));
    nFail = sum(strcmp({jobs.status}, 'FAIL'));
    nSkip = sum(strcmp({jobs.status}, 'SKIP'));
    fprintf('-------------------------------------------------------------------------\n');
    fprintf('  成功 %d 件, 失敗 %d 件, 復号済みで飛ばした %d 件\n', nOK, nFail, nSkip);
    if nFail > 0
        fprintf(['  ※失敗したファイルは、decodeIQ_HE.m の inputRawFile に指定して\n', ...
                 '    単体で実行すると原因を詳しく確認できます。\n']);
    end
    zeroHE = arrayfun(@(j) strcmp(j.status, 'OK') && ~isempty(j.counts) && j.counts.HE == 0, jobs);
    if any(zeroHE)
        fprintf(['  ※成功したが HE の CSI が 0 件のファイルが %d 件あります。対象 SSID の\n', ...
                 '    ビーコンを捕捉できていない可能性があります (ログを確認)。\n'], sum(zeroHE));
    end
end

function writeSummaryCsv(csvFile, jobs)
    % ファイルごとの結果を CSV に書く (Excel で開ける)
    fid = fopen(csvFile, 'w', 'n', 'UTF-8');
    if fid == -1
        warning('decodeIQ_HE_batch:csvFailed', '結果の一覧を書き込めませんでした: %s', csvFile);
        return;
    end
    fprintf(fid, '%s', char(65279));   % Excel 向けの BOM (U+FEFF, 日本語の文字化け防止)
    fprintf(fid, 'file,status,elapsed_s,capture_s,output,log,HE,NonHT,HT,VHT,error\n');
    for k = 1:numel(jobs)
        j = jobs(k);
        if isempty(j.counts)
            c = struct('HE', NaN, 'NonHT', NaN, 'HT', NaN, 'VHT', NaN);
        else
            c = j.counts;
        end
        err = strrep(j.errMsg, '"', '""');
        fprintf(fid, '%s,%s,%.1f,%.1f,%s,%s,%g,%g,%g,%g,"%s"\n', ...
            j.name, j.status, j.elapsedSec, j.captureSec, j.outName, j.logName, ...
            c.HE, c.NonHT, c.HT, c.VHT, err);
    end
    fclose(fid);
end

function s = formatDuration(sec)
    if isnan(sec) || sec < 0
        s = '?';
    elseif sec < 120
        s = sprintf('%.0f 秒', sec);
    elseif sec < 7200
        s = sprintf('%.0f 分', sec / 60);
    else
        s = sprintf('%.1f 時間', sec / 3600);
    end
end

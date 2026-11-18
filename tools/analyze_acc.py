#!/usr/bin/env python3
"""
Lab7 加速度 CSV 分析工具

用法:
    python analyze_acc.py capture.csv            # 只印數字
    python analyze_acc.py capture.csv --plot     # 另外存兩張 PNG

輸入格式（直接貼終端機 dump 的那段，標頭那行要留著）:
    idx,t_ms,raw_mg,lp2_mg,lp5_mg,lp10_mg,lp20_mg
或舊格式:
    idx,t_ms,raw_mg,filtered_mg

標頭前後多餘的文字（press any key / clipped / cutoff 表格）會自動略過。
"""
import sys
import numpy as np

FS = 100.0          # 取樣率 (Hz)，和韌體的 ACC_FS_HZ 一致
SKIP = 28           # 丟掉的啟動暫態筆數 = NUM_TAPS - 1
FULL_SCALE = 2000.0 # 感測器滿量程 (mg)


def load(path):
    names, rows = None, []
    for ln in open(path, encoding="utf-8", errors="ignore"):
        p = [x.strip() for x in ln.strip().split(",")]
        if len(p) >= 3 and p[0] == "idx":
            names = p
            continue
        if names and len(p) == len(names):
            try:
                rows.append([float(x) for x in p])
            except ValueError:
                pass
    if not rows:
        sys.exit("找不到資料行，確認 CSV 標頭那行 (idx,t_ms,raw_mg,...) 有複製到")
    return names, np.array(rows)


def spectrum(x, fs=FS):
    """去 DC + Hanning 窗 + rfft，回傳 (頻率, 正規化強度)"""
    w = np.hanning(len(x))
    sp = np.abs(np.fft.rfft((x - x.mean()) * w))
    fr = np.fft.rfftfreq(len(x), 1.0 / fs)
    return fr, sp / sp.max()


def main():
    path = sys.argv[1]
    do_plot = "--plot" in sys.argv
    names, d = load(path)
    cols = {n: d[:, i] for i, n in enumerate(names)}
    raw = cols["raw_mg"]
    filt_names = [n for n in names if n not in ("idx", "t_ms", "raw_mg")]

    print("檔案: %s" % path)
    print("樣本數 %d, fs = %g Hz, 時長 %.2f s" % (len(raw), FS, len(raw) / FS))
    print()

    # ---- 飽和檢查 ----
    hi = int((raw >= FULL_SCALE - 5).sum())
    lo = int((raw <= -FULL_SCALE + 5).sum())
    print("--- 飽和檢查 (±%.0f mg) ---" % FULL_SCALE)
    print("  raw max/min : %.0f / %.0f mg" % (raw.max(), raw.min()))
    print("  削平樣本    : %d (%.1f%%)" % (hi + lo, 100.0 * (hi + lo) / len(raw)))
    if hi + lo:
        print("  >>> 資料已失真，搖輕一點或把量程開到 ±8 g")
    print()

    # ---- 頻譜 ----
    seg = raw[SKIP:]
    fr, sp = spectrum(seg)
    print("--- raw 頻譜 (丟掉前 %d 筆暫態，解析度 %.3f Hz) ---" % (SKIP, FS / len(seg)))
    for i in sorted(np.argsort(sp)[::-1][:5]):
        print("  %6.2f Hz   相對強度 %.3f" % (fr[i], sp[i]))
    print()
    tot = (sp ** 2).sum()
    print("--- 頻帶能量佔比 ---")
    for a, b in [(0, 2), (2, 5), (5, 10), (10, 20), (20, FS / 2)]:
        m = (fr >= a) & (fr < b)
        print("  %4.0f - %4.0f Hz : %5.1f%%" % (a, b, 100.0 * (sp[m] ** 2).sum() / tot))
    print()

    # ---- 各 cutoff 的削除率 ----
    sd_raw = seg.std()
    print("--- 濾波效果 (同一筆資料，丟掉暫態後統計) ---")
    print("  %-10s %10s %12s %12s" % ("訊號", "std(mg)", "削掉", "DC 平均(mg)"))
    print("  %-10s %10.1f %12s %12.1f" % ("raw", sd_raw, "-", seg.mean()))
    for n in filt_names:
        y = cols[n][SKIP:]
        print("  %-10s %10.1f %11.1f%% %12.1f"
              % (n, y.std(), 100.0 * (1 - y.std() / sd_raw), y.mean()))
    print()

    if not do_plot:
        print("加 --plot 可以另外存出兩張圖")
        return

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.arange(len(raw)) / FS
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(t, raw, lw=0.7, color="0.6", label="raw")
    for n in filt_names:
        ax.plot(t, cols[n], lw=1.3, label=n.replace("_mg", ""))
    ax.set_xlabel("time (s)"); ax.set_ylabel("acceleration (mg)")
    ax.set_title("FIR lowpass, different cutoffs, same input")
    ax.legend(ncol=len(filt_names) + 1, fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig("acc_time.png", dpi=130)
    print("存出 acc_time.png")

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.semilogy(fr, np.maximum(sp, 1e-4), lw=0.9, color="C0")
    for fc, c in zip([2, 5, 10, 20], ["C1", "C2", "C3", "C4"]):
        ax.axvline(fc, color=c, ls="--", lw=1.1, label="cutoff %d Hz" % fc)
    ax.set_xlim(0, FS / 2); ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("relative magnitude")
    ax.set_title("raw spectrum vs cutoffs")
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which="both")
    fig.tight_layout(); fig.savefig("acc_spectrum.png", dpi=130)
    print("存出 acc_spectrum.png")


if __name__ == "__main__":
    main()

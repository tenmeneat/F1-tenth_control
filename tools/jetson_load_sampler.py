#!/usr/bin/env python3
"""젯슨 연산 부하 샘플러 — 주행 중 "무엇이 언제 멈췄나"를 bag과 같은 시계로 기록한다.

왜 이게 필요한가 (2026-08-18):
  `run_0818_134408`에서 MCL이 **1125 ms** 통째로 멈췄고 라이다 드라이버도 850 ms 멈췄는데,
  같은 시각 VESC 드라이버(50 Hz)와 컨트롤러(50 Hz)는 한 사이클도 안 빠졌다. 정지 직전까지
  MCL 간격은 완벽한 25.0 ms였다 — **서서히 밀린 게 아니라 한 번에 구멍이 났다.**
  bag만으로는 CPU·메모리·I/O 중 무엇인지 못 가른다. 그걸 가르는 게 이 스크립트다.

설계 요점:
  1) 🔑 **PSI(`/proc/pressure/*`)의 `total` 누적값을 diff해서 쓴다.** `avg10`은 10초 창이라
     1.1초 사건이 10배 희석돼 안 보인다. total delta는 "이 구간에서 실제로 몇 µs를
     기다렸나"라 사건 길이와 1:1로 대응한다. PSI가 곧 답이다 —
     cpu=런큐 대기 / io=디스크 대기 / memory=페이지 회수 대기.
  2) 시스템 지표는 20 Hz, 프로세스별은 5 Hz. 프로세스 목록은 5초마다 갱신한다
     (매 틱 /proc 전체 스캔은 그 자체가 부하라 안 한다 — 관측이 대상을 바꾸면 안 된다).
  3) **epoch 초(time.time())로 찍는다.** bag도 젯슨에서 같은 시계로 녹화되므로
     (jetson_rec.sh) 후처리에서 그냥 붙는다. ⚠️ 젯슨 RTC는 배터리가 없어 전원을 끊으면
     1970으로 돌아간다 — f1time.sh가 ssh마다 밀어넣지만, 값이 이상하면 그것부터 의심할 것.
  4) tegrastats가 있으면 같이 돌린다(EMC 대역폭·온도·클럭 스로틀링). 없으면 조용히 생략.

출력: <outdir>/sys.csv, proc.csv, tegra.log, meta.txt, dmesg_{before,after}.txt
"""
import os, sys, time, subprocess, signal, glob

CLK = os.sysconf("SC_CLK_TCK")
SYS_HZ = 20.0        # 시스템 지표 샘플레이트
PROC_EVERY = 4       # 프로세스 지표는 4틱마다(=5 Hz)
RESCAN_EVERY = 100   # 프로세스 목록 갱신 = 5초마다

# 관심 프로세스 — cmdline에 이 문자열이 들어가면 개별 추적한다.
WATCH = ["control_map_node", "particle_filter", "monte_carlo", "mcl", "urg_node",
         "vesc_driver", "ackermann", "drive_source", "ros2 bag", "rosbag", "sector_learner",
         "planner", "state_machine", "frenet", "obstacle", "wpnt", "joy"]
# 셸·래퍼는 제외한다. "cmdline에 노드 이름이 들어있다"는 이유만으로 잡히기 때문이다
# (`ros2 run f1tenth_control control_map_node`를 띄운 zsh가 그대로 걸린다).
SKIP_COMM = {"zsh", "bash", "sh", "dash", "ssh", "sshd", "sudo", "env", "timeout",
             "tmux", "tmux: server", "grep", "awk", "sed", "source", "run"}
TOPN = 8   # WATCH에 없어도 CPU를 많이 쓰는 프로세스는 자동 추적 — 범인이 미지일 수 있다


def read(p, default=""):
    try:
        with open(p) as f: return f.read()
    except Exception: return default


def psi(kind):
    """/proc/pressure/<kind> → (some_total_us, full_total_us). 없으면 (None,None)."""
    t = read(f"/proc/pressure/{kind}")
    if not t: return (None, None)
    some = full = None
    for line in t.splitlines():
        for tok in line.split():
            if tok.startswith("total="):
                v = int(tok[6:])
                if line.startswith("some"): some = v
                elif line.startswith("full"): full = v
    return (some, full)


def net_ifaces():
    """라이다가 붙은 이더넷 인터페이스 후보. 무선/루프백/도커는 제외한다."""
    out = []
    try:
        for n in sorted(os.listdir("/sys/class/net")):
            if n == "lo" or n.startswith(("wl", "docker", "veth", "br-", "can", "usb")):
                continue
            if os.path.exists(f"/sys/class/net/{n}/carrier"):
                out.append(n)
    except OSError:
        pass
    return out


def carrier(iface):
    """1 = 링크 살아있음, 0 = 끊김, -1 = 읽기 실패(보통 인터페이스 down)."""
    v = read(f"/sys/class/net/{iface}/carrier").strip()
    return int(v) if v in ("0", "1") else -1


def cpu_jiffies():
    """/proc/stat 첫 줄 → (busy, total)."""
    line = read("/proc/stat").split("\n")[0].split()
    if len(line) < 8: return (0, 0)
    v = [int(x) for x in line[1:8]]
    idle = v[3] + v[4]                 # idle + iowait
    return (sum(v) - idle, sum(v))


def label(pid, comm, cl):
    """읽을 수 있는 이름. comm이 우선이고, 범용 런처면 cmdline에서 실제 대상을 찾는다."""
    if comm and comm not in ("python3", "python", "ros2", "component_container"):
        return comm[:28]
    for tok in cl.split()[1:]:
        b = tok.split("/")[-1]
        if b and not b.startswith("-") and not b.endswith(".yaml"):
            return b[:28]
    return (comm or "?")[:28]


def procs(prev_tot=None):
    """추적 대상 pid → 라벨.
    ① WATCH에 걸리는 것  ② 그와 무관하게 CPU 상위 TOPN (범인이 미지의 프로세스일 수 있다).
    prev_tot(pid→jiffies)을 주면 그 사이의 CPU 증분으로 상위를 고른다."""
    out, tot = {}, {}
    for d in glob.glob("/proc/[0-9]*"):
        pid = d.split("/")[-1]
        comm = read(f"{d}/comm").strip()
        if not comm or comm in SKIP_COMM: continue
        st = pstat(pid)
        if st is None: continue
        tot[pid] = st[0]
        cl = read(f"{d}/cmdline").replace("\0", " ").strip()
        if not cl: continue                      # 커널 스레드
        hay = comm + " " + cl
        if any(w in hay for w in WATCH):
            out[pid] = label(pid, comm, cl)
    if prev_tot:
        delta = sorted(((tot[p] - prev_tot.get(p, tot[p]), p) for p in tot),
                       key=lambda x: -x[0])[:TOPN]
        for dj, pid in delta:
            if dj <= 0 or pid in out: continue
            cl = read(f"/proc/{pid}/cmdline").replace("\0", " ").strip()
            out[pid] = label(pid, read(f"/proc/{pid}/comm").strip(), cl)
    return out, tot


def pstat(pid):
    """(utime+stime jiffies, majflt, threads, rss_kb) 또는 None."""
    s = read(f"/proc/{pid}/stat")
    if not s: return None
    r = s[s.rfind(")") + 2:].split()
    try:
        return (int(r[11]) + int(r[12]), int(r[9]), int(r[17]), int(r[21]) * 4)
    except Exception: return None


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/jetson_load")
    os.makedirs(outdir, exist_ok=True)

    with open(f"{outdir}/meta.txt", "w") as f:
        f.write(f"start_epoch\t{time.time():.6f}\n")
        f.write(f"start_local\t{time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"hostname\t{read('/proc/sys/kernel/hostname').strip()}\n")
        f.write(f"ncpu\t{os.cpu_count()}\n")
        f.write(f"clk_tck\t{CLK}\n")
        f.write(f"psi\t{'yes' if psi('cpu')[0] is not None else 'NO(커널 CONFIG_PSI 없음)'}\n")
        f.write(f"uptime\t{read('/proc/uptime').split()[0] if read('/proc/uptime') else '?'}\n")
        f.write(f"model\t{read('/proc/device-tree/model','?').strip(chr(0))}\n")
        f.write(f"cmdline\t{' '.join(sys.argv)}\n")
    # dmesg는 kernel.dmesg_restrict=1이면 비루트가 못 읽는다 — 조용히 빈 파일이 남는 대신
    # meta에 남겨 두고, 필요하면 사용자가 한 번만 풀도록 안내한다(sudo를 이 스크립트가 쥐지 않는다).
    subprocess.run(f"dmesg -T > {outdir}/dmesg_before.txt 2>/dev/null", shell=True)
    dmesg_ok = os.path.getsize(f"{outdir}/dmesg_before.txt") > 0
    with open(f"{outdir}/meta.txt", "a") as f:
        f.write(f"dmesg\t{'yes' if dmesg_ok else 'NO(권한). 한 번만: sudo sysctl -w kernel.dmesg_restrict=0'}\n")
        f.write(f"tegrastats\t{'yes' if subprocess.run('command -v tegrastats', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0 else 'NO(젯슨이 아니거나 미설치)'}\n")
    if not dmesg_ok:
        print("[jetson_load] ⚠️ dmesg를 못 읽습니다(권한). USB 리셋·OOM 흔적을 보려면 한 번만:", flush=True)
        print("[jetson_load]    sudo sysctl -w kernel.dmesg_restrict=0", flush=True)

    # tegrastats (있으면). EMC 대역폭·온도·클럭 스로틀은 여기서만 나온다.
    tegra = None
    if subprocess.run("command -v tegrastats", shell=True,
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        tf = open(f"{outdir}/tegra.log", "w")
        tegra = subprocess.Popen(["tegrastats", "--interval", "200"],
                                 stdout=subprocess.PIPE, text=True, bufsize=1)
        import threading
        def pump():
            for line in tegra.stdout:
                tf.write(f"{time.time():.3f}\t{line}"); tf.flush()
        threading.Thread(target=pump, daemon=True).start()

    ncpu = os.cpu_count() or 1
    sysf = open(f"{outdir}/sys.csv", "w", buffering=1)
    # 🔑 라이다는 이더넷이다(192.168.0.10:10940). 0819에 8.04초 링크 두절로 벽에 박았는데
    #    bag만으로는 "라이다가 죽었다"까지만 알고 원인(링크/드라이버)을 못 갈랐다.
    #    carrier를 20 Hz로 같이 찍으면 그 순간 링크가 내려갔는지가 확정된다.
    ifaces = net_ifaces()
    link_cols = "".join(f",link_{n}" for n in ifaces)
    sysf.write("t,cpu_pct,psi_cpu_some_us,psi_io_some_us,psi_io_full_us,"
               "psi_mem_some_us,psi_mem_full_us,load1,nr_running,ctxt_per_s,mem_avail_kb,tick_slip_ms"
               + link_cols + "\n")
    procf = open(f"{outdir}/proc.csv", "w", buffering=1)
    procf.write("t,name,pid,cpu_pct,majflt_delta,threads,rss_kb\n")

    stop = {"v": False}
    signal.signal(signal.SIGINT,  lambda *a: stop.__setitem__("v", True))
    signal.signal(signal.SIGTERM, lambda *a: stop.__setitem__("v", True))

    plist, all_tot = procs()
    prev_cpu = cpu_jiffies()
    prev_psi = {k: psi(k) for k in ("cpu", "io", "memory")}
    prev_p = {pid: pstat(pid) for pid in plist}
    prev_ctxt = 0
    for line in read("/proc/stat").splitlines():
        if line.startswith("ctxt"): prev_ctxt = int(line.split()[1])
    link_prev = {n: None for n in ifaces}
    link_events = []
    prev_t = time.time()
    tick = 0
    period = 1.0 / SYS_HZ
    next_t = prev_t + period

    print(f"[jetson_load] 샘플링 시작 → {outdir}  (Ctrl+C 로 종료)", flush=True)
    while not stop["v"]:
        now = time.time()
        if now < next_t:
            time.sleep(min(next_t - now, 0.05)); continue
        slip = (now - next_t) * 1000.0      # 이 샘플러 자신이 밀린 정도(= 시스템 전체 스톨의 증거)
        next_t += period
        if next_t < now - 1.0: next_t = now + period   # 크게 밀리면 리셋(따라잡기 폭주 방지)

        dt = now - prev_t
        if dt <= 0: continue
        c = cpu_jiffies()
        dbusy, dtot = c[0] - prev_cpu[0], c[1] - prev_cpu[1]
        cpu_pct = 100.0 * dbusy / dtot if dtot > 0 else 0.0
        prev_cpu = c

        row = {}
        for k in ("cpu", "io", "memory"):
            cur = psi(k)
            for i, tag in enumerate(("some", "full")):
                d = (cur[i] - prev_psi[k][i]) if (cur[i] is not None and prev_psi[k][i] is not None) else -1
                row[f"{k}_{tag}"] = d
            prev_psi[k] = cur

        la = read("/proc/loadavg").split()
        load1 = la[0] if la else "0"
        nr_run = la[3].split("/")[0] if len(la) > 3 else "0"
        ctxt = prev_ctxt
        for line in read("/proc/stat").splitlines():
            if line.startswith("ctxt"): ctxt = int(line.split()[1])
        dctxt = int((ctxt - prev_ctxt) / dt); prev_ctxt = ctxt
        mem_avail = 0
        for line in read("/proc/meminfo").splitlines():
            if line.startswith("MemAvailable"): mem_avail = int(line.split()[1]); break

        links = [carrier(n) for n in ifaces]
        for n, cur in zip(ifaces, links):
            if link_prev.get(n) is not None and cur != link_prev[n]:
                # 전이는 stdout에도 즉시 남긴다 — csv를 안 열어봐도 보이게.
                print(f"[jetson_load] {time.strftime('%H:%M:%S')} 링크 {n}: "
                      f"{link_prev[n]} -> {cur}  (0=끊김)", flush=True)
                link_events.append((now, n, link_prev[n], cur))
            link_prev[n] = cur
        sysf.write(f"{now:.3f},{cpu_pct:.1f},{row['cpu_some']},{row['io_some']},{row['io_full']},"
                   f"{row['memory_some']},{row['memory_full']},{load1},{nr_run},{dctxt},{mem_avail},{slip:.1f}"
                   + "".join(f",{v}" for v in links) + "\n")

        if tick % PROC_EVERY == 0:
            for pid, name in list(plist.items()):
                st = pstat(pid)
                if st is None: continue
                pv = prev_p.get(pid)
                prev_p[pid] = st
                if pv is None: continue
                pct = 100.0 * (st[0] - pv[0]) / CLK / dt
                procf.write(f"{now:.3f},{name},{pid},{pct:.1f},{st[1]-pv[1]},{st[2]},{st[3]}\n")
        if tick % RESCAN_EVERY == 0 and tick:
            plist, all_tot = procs(all_tot)
            for pid in plist:
                if pid not in prev_p: prev_p[pid] = pstat(pid)
        prev_t = now; tick += 1

    if tegra:
        tegra.terminate()
    subprocess.run(f"dmesg -T > {outdir}/dmesg_after.txt 2>/dev/null", shell=True)
    with open(f"{outdir}/meta.txt", "a") as f:
        f.write(f"end_epoch\t{time.time():.6f}\n")
        f.write(f"samples\t{tick}\n")
        f.write(f"net_ifaces\t{','.join(ifaces) if ifaces else 'NONE'}\n")
        f.write(f"link_events\t{len(link_events)}\n")
        for t_, n_, a_, b_ in link_events:
            f.write(f"link_event\t{t_:.3f}\t{n_}\t{a_}->{b_}\n")
    print(f"\n[jetson_load] 종료 — {tick} 샘플, {outdir}", flush=True)


if __name__ == "__main__":
    main()

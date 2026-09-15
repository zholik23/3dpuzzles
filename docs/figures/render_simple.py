import io, json, os, re, subprocess, shutil
CHROME = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
TMP = r"C:\Users\Admin\.claude\jobs\d0af9bbd\tmp"
OUT = r"C:\Users\Admin\Documents\IRIT_to_Gcode-main\docs\images"
CSS = io.open(os.path.join(TMP, "vs_1d.html"), encoding="utf-8").read().split("</style>")[0] + "</style>"

figs = json.load(io.open(os.path.join(TMP, "voxel_simple.json"), encoding="utf-8"))
for key, svg in figs.items():
    w, h = [float(v) for v in re.search(r'viewBox="0 0 ([\d.]+) ([\d.]+)"', svg).groups()]
    W = 1800
    H = int(round(W * h / w))
    html = os.path.join(TMP, f"vs_{key}.html")
    png  = os.path.join(TMP, f"vs_{key}.png")
    io.open(html, "w", encoding="utf-8").write(CSS + svg)
    if os.path.exists(png):
        os.remove(png)
    subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--no-sandbox",
                    "--hide-scrollbars", f"--window-size={W},{H}",
                    f"--screenshot={png}", "--virtual-time-budget=4000",
                    "file:///" + html.replace("\\", "/")],
                   check=True, capture_output=True)
    shutil.copyfile(png, os.path.join(OUT, f"voxel_simple_{key}.png"))
    shutil.copyfile(png, os.path.join(r"C:\Users\Admin\Documents\IRIT_to_Gcode-main", f"ps_{key}.png"))
    print(f"{key:5s} {W}x{H}  {os.path.getsize(png):,} bytes")

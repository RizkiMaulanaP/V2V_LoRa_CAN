#!/usr/bin/env python3
"""
Path visualization for processed UKF CSV logs.

Plots the fused vehicle trajectory, coloured by speed, with heading arrows,
start/end markers, and any non-NORMAL "condition" points highlighted.
A secondary panel shows the speed profile over time.

Received V2V data from other vehicles is read from the sibling raw_*.csv
(LORA_RX frames) and overlaid in the same coordinate frame: each remote node
gets its own coloured track, latest-position marker, heading arrows, alert
markers, and a speed trace. Implausibly far fixes (corrupted LoRa packets)
are clipped — see --lora-clip / --no-lora.

The processed CSV schema has varied between firmware/app versions, so this
tool detects the available columns rather than assuming a fixed layout:

  position : pos_east_m / pos_north_m   (preferred, local ENU metres)
             else lat / lon             (projected to local metres)
  speed    : speed_kmh  or  speed_ms
  heading  : yaw_deg    or  heading_deg
  time     : mcu_ts_ms  or  wall_time
  status   : condition  (optional; non-NORMAL rows are marked)

Usage:
    python plot_path.py                      # newest non-empty processed_*.csv
    python plot_path.py processed_xxx.csv    # a specific file
    python plot_path.py --geo                # plot in lat/lon instead of metres
    python plot_path.py -o path.png --no-show
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

_EARTH_R = 6378137.0  # WGS-84 equatorial radius (m)


def find_latest_csv(folder: Path) -> Path | None:
    """Most recently modified processed_*.csv that has at least one data row."""
    cands = sorted(folder.glob("processed_*.csv"), key=lambda p: p.stat().st_mtime,
                   reverse=True)
    for p in cands:
        try:
            # header + >=2 data rows is the minimum for a path
            with p.open() as f:
                if sum(1 for _ in f) >= 3:
                    return p
        except OSError:
            continue
    return cands[0] if cands else None


def first_present(df: pd.DataFrame, *names: str) -> str | None:
    for n in names:
        if n in df.columns:
            return n
    return None


def project_to_enu(lat: np.ndarray, lon: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Equirectangular projection to local east/north metres about the mean point."""
    lat0 = np.nanmean(lat)
    lon0 = np.nanmean(lon)
    east = np.radians(lon - lon0) * _EARTH_R * np.cos(np.radians(lat0))
    north = np.radians(lat - lat0) * _EARTH_R
    return east, north


def load(path: Path, geo: bool):
    df = pd.read_csv(path)
    if df.empty:
        raise SystemExit(f"{path.name}: no data rows.")

    # --- speed (km/h) ---
    s_col = first_present(df, "speed_kmh", "speed_ms")
    if s_col == "speed_ms":
        speed = pd.to_numeric(df["speed_ms"], errors="coerce").to_numpy() * 3.6
    elif s_col:
        speed = pd.to_numeric(df["speed_kmh"], errors="coerce").to_numpy()
    else:
        speed = np.zeros(len(df))

    # --- heading (deg, 0 = North, CW) ---
    h_col = first_present(df, "yaw_deg", "heading_deg")
    heading = pd.to_numeric(df[h_col], errors="coerce").to_numpy() if h_col else None

    # --- time (s, relative to start) ---
    t_col = first_present(df, "mcu_ts_ms", "wall_time")
    t0_ms = None  # ego start on the MCU clock, used to align remote LoRa data
    if t_col == "mcu_ts_ms":
        t_ms = pd.to_numeric(df["mcu_ts_ms"], errors="coerce").to_numpy()
        t0_ms = float(np.nanmin(t_ms))
        t = (t_ms - t0_ms) / 1000.0
    elif t_col == "wall_time":
        wt = pd.to_datetime(df["wall_time"], errors="coerce")
        t = (wt - wt.iloc[0]).dt.total_seconds().to_numpy()
    else:
        t = np.arange(len(df), dtype=float)

    # --- position ---
    has_xy = {"pos_east_m", "pos_north_m"}.issubset(df.columns)
    has_ll = {"lat", "lon"}.issubset(df.columns)
    origin = None  # (lat0, lon0) of the local ENU frame, so remote tracks share it
    if geo:
        if not has_ll:
            raise SystemExit(f"{path.name}: --geo needs lat/lon columns.")
        x = pd.to_numeric(df["lon"], errors="coerce").to_numpy()
        y = pd.to_numeric(df["lat"], errors="coerce").to_numpy()
        xlabel, ylabel, mode = "Longitude (°)", "Latitude (°)", "geo"
    elif has_xy:
        x = pd.to_numeric(df["pos_east_m"], errors="coerce").to_numpy()
        y = pd.to_numeric(df["pos_north_m"], errors="coerce").to_numpy()
        xlabel, ylabel, mode = "East (m)", "North (m)", "enu"
        # Recover the ENU origin by inverting one ego row (east/north ⇄ lat/lon),
        # so remote vehicles can be projected into this exact frame.
        if has_ll:
            lat = pd.to_numeric(df["lat"], errors="coerce").to_numpy()
            lon = pd.to_numeric(df["lon"], errors="coerce").to_numpy()
            good = np.isfinite(x) & np.isfinite(y) & np.isfinite(lat) & np.isfinite(lon)
            if good.any():
                i = int(np.argmax(good))
                lat0 = lat[i] - np.degrees(y[i] / _EARTH_R)
                lon0 = lon[i] - np.degrees(x[i] / (_EARTH_R * np.cos(np.radians(lat0))))
                origin = (lat0, lon0)
    elif has_ll:
        lat = pd.to_numeric(df["lat"], errors="coerce").to_numpy()
        lon = pd.to_numeric(df["lon"], errors="coerce").to_numpy()
        x, y = project_to_enu(lat, lon)
        origin = (float(np.nanmean(lat)), float(np.nanmean(lon)))
        xlabel, ylabel, mode = "East (m)", "North (m)", "enu"
    else:
        raise SystemExit(f"{path.name}: no position columns "
                         "(need pos_east_m/pos_north_m or lat/lon).")

    # --- condition (optional) ---
    cond = (df["condition"].astype(str).str.upper().to_numpy()
            if "condition" in df.columns else None)

    # Drop rows with an invalid position so segments stay continuous.
    ok = np.isfinite(x) & np.isfinite(y)
    if not ok.any():
        raise SystemExit(f"{path.name}: position columns are all empty/NaN.")
    keep = {"x": x[ok], "y": y[ok], "speed": speed[ok], "t": t[ok],
            "heading": heading[ok] if heading is not None else None,
            "cond": cond[ok] if cond is not None else None,
            "xlabel": xlabel, "ylabel": ylabel, "mode": mode,
            "origin": origin, "t0_ms": t0_ms}
    return keep


def enu_from_latlon(lat: np.ndarray, lon: np.ndarray,
                    origin: tuple[float, float]) -> tuple[np.ndarray, np.ndarray]:
    """Project lat/lon to local ENU metres about a *given* origin."""
    lat0, lon0 = origin
    x = np.radians(lon - lon0) * _EARTH_R * np.cos(np.radians(lat0))
    y = np.radians(lat - lat0) * _EARTH_R
    return x, y


def find_raw_for(processed: Path) -> Path | None:
    """Sibling raw_*.csv that shares the processed file's timestamp suffix."""
    if processed.name.startswith("processed_"):
        cand = processed.with_name("raw_" + processed.name[len("processed_"):])
        if cand.exists():
            return cand
    return None


def _metres_from(cx: float, cy: float, x: np.ndarray, y: np.ndarray,
                 mode: str) -> np.ndarray:
    """Distance (m) of points (x,y) from a centre, for either plot mode."""
    if mode == "geo":  # x=lon, y=lat in degrees
        dx = np.radians(x - cx) * _EARTH_R * np.cos(np.radians(cy))
        dy = np.radians(y - cy) * _EARTH_R
    else:              # already ENU metres
        dx, dy = x - cx, y - cy
    return np.hypot(dx, dy)


def load_lora(raw_path: Path, mode: str, origin: tuple[float, float] | None,
              t0_ms: float | None, ego_xy: tuple[float, float] | None = None,
              clip_m: float = 0.0) -> list[dict]:
    """Per-remote-node tracks from LORA_RX frames in a raw CSV.

    Returns a list of dicts with x/y in the same frame as the ego path
    (ENU metres or lon/lat), plus speed, heading, alert flags and time.
    Points farther than ``clip_m`` from ``ego_xy`` (corrupted LoRa fixes)
    are dropped; each track records how many in ``dropped``.
    """
    try:
        df = pd.read_csv(raw_path)
    except (OSError, pd.errors.ParserError):
        return []
    if "frame_type" not in df.columns:
        return []
    rx = df[df["frame_type"].astype(str) == "LORA_RX"]
    if rx.empty or "lora_lat" not in rx.columns:
        return []

    node_col = "lora_node" if "lora_node" in rx.columns else None
    tracks: list[dict] = []
    groups = rx.groupby(node_col) if node_col else [(None, rx)]
    for node, g in groups:
        try:  # node ids are integers; pandas reads them as float (NaN-capable)
            node = int(node) if node is not None and float(node).is_integer() else node
        except (TypeError, ValueError):
            pass
        lat = pd.to_numeric(g["lora_lat"], errors="coerce").to_numpy()
        lon = pd.to_numeric(g["lora_lon"], errors="coerce").to_numpy()
        # A 0,0 fix means "no position yet" — drop those rows.
        ok = np.isfinite(lat) & np.isfinite(lon) & (lat != 0.0) & (lon != 0.0)
        if not ok.any():
            continue
        lat, lon = lat[ok], lon[ok]

        def col(name):
            return (pd.to_numeric(g[name], errors="coerce").to_numpy()[ok]
                    if name in g.columns else None)

        speed = col("lora_speed_kmh")
        heading = col("lora_heading_deg")
        alert = (g["lora_alert"].astype(str).to_numpy()[ok]
                 if "lora_alert" in g.columns else None)
        if "mcu_ts_ms" in g.columns and t0_ms is not None:
            ts = pd.to_numeric(g["mcu_ts_ms"], errors="coerce").to_numpy()[ok]
            tt = (ts - t0_ms) / 1000.0
        else:
            tt = None

        if mode == "geo":
            x, y = lon, lat
        elif origin is not None:
            x, y = enu_from_latlon(lat, lon, origin)
        else:
            # No shared origin (ego had no lat/lon) — can't place in metres.
            continue

        # Drop corrupted fixes sitting implausibly far from the ego path.
        dropped = 0
        if clip_m > 0 and ego_xy is not None:
            near = _metres_from(ego_xy[0], ego_xy[1], x, y, mode) <= clip_m
            dropped = int((~near).sum())
            if not near.any():
                continue
            x, y = x[near], y[near]
            speed = speed[near] if speed is not None else None
            heading = heading[near] if heading is not None else None
            alert = alert[near] if alert is not None else None
            tt = tt[near] if tt is not None else None
        tracks.append({"node": node, "x": x, "y": y, "speed": speed,
                       "heading": heading, "alert": alert, "t": tt,
                       "dropped": dropped})
    return tracks


def path_length(x: np.ndarray, y: np.ndarray, mode: str) -> float:
    """Cumulative travelled distance in metres (geo mode projects first)."""
    if mode == "geo":
        ex, ny = project_to_enu(y, x)  # y=lat, x=lon
    else:
        ex, ny = x, y
    return float(np.nansum(np.hypot(np.diff(ex), np.diff(ny))))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="processed CSV (default: newest non-empty)")
    ap.add_argument("--geo", action="store_true", help="plot in lat/lon, not metres")
    ap.add_argument("--arrows", type=int, default=20,
                    help="number of heading arrows along the path (0 = none)")
    ap.add_argument("--lora", help="raw CSV with LORA_RX frames for the remote "
                    "vehicle(s) (default: sibling raw_*.csv)")
    ap.add_argument("--no-lora", action="store_true",
                    help="don't overlay received LoRa (other-vehicle) tracks")
    ap.add_argument("--lora-clip", type=float, default=2000.0,
                    help="drop remote LoRa fixes farther than this many metres "
                    "from the ego path (corrupted packets); 0 = keep all")
    ap.add_argument("-o", "--out", help="PNG output path (default: alongside CSV)")
    ap.add_argument("--no-show", action="store_true", help="save only, don't display")
    args = ap.parse_args()

    folder = Path(__file__).resolve().parent
    path = Path(args.csv) if args.csv else find_latest_csv(folder)
    if path is None or not path.exists():
        raise SystemExit("No processed CSV found. Pass one explicitly.")

    if args.no_show:
        matplotlib.use("Agg")

    d = load(path, args.geo)
    x, y, speed, t = d["x"], d["y"], d["speed"], d["t"]

    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[2.2, 1], height_ratios=[3, 1],
                          hspace=0.28, wspace=0.22)
    ax = fig.add_subplot(gs[:, 0])      # trajectory (spans both rows)
    ax_sp = fig.add_subplot(gs[0, 1])   # speed vs time
    ax_tx = fig.add_subplot(gs[1, 1])   # stats text
    ax_tx.axis("off")

    # ── Trajectory coloured by speed ──────────────────────────────────────────
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    sp_finite = speed[np.isfinite(speed)]
    vmin = float(sp_finite.min()) if sp_finite.size else 0.0
    vmax = float(sp_finite.max()) if sp_finite.size else 1.0
    lc = LineCollection(segs, cmap="viridis",
                        norm=plt.Normalize(vmin, max(vmax, vmin + 1e-6)))
    lc.set_array(speed[:-1])
    lc.set_linewidth(2.4)
    ax.add_collection(lc)
    cbar = fig.colorbar(lc, ax=ax, fraction=0.045, pad=0.02)
    cbar.set_label("Speed (km/h)")

    # ── Heading arrows ────────────────────────────────────────────────────────
    if d["heading"] is not None and args.arrows > 0 and len(x) > 2:
        idx = np.linspace(0, len(x) - 1, min(args.arrows, len(x))).astype(int)
        hd = np.radians(d["heading"][idx])
        # heading 0°=North(+y), CW; in geo mode east≈+lon so same convention holds
        u, v = np.sin(hd), np.cos(hd)
        # Size arrows to ~6% of the path extent so they read regardless of scale.
        span = max(np.nanmax(x) - np.nanmin(x), np.nanmax(y) - np.nanmin(y), 1e-9)
        alen = 0.06 * span
        ax.quiver(x[idx], y[idx], u, v, color="black", alpha=0.6,
                  angles="xy", scale_units="xy", scale=1.0 / alen,
                  width=0.004, zorder=4, label="heading")

    # ── Condition highlights ──────────────────────────────────────────────────
    if d["cond"] is not None:
        alert = ~np.isin(d["cond"], ("NORMAL", "NAN", "", "NONE"))
        if alert.any():
            ax.scatter(x[alert], y[alert], s=36, marker="x", color="red",
                       zorder=5, label="non-NORMAL")

    # ── Start / end ───────────────────────────────────────────────────────────
    ax.scatter(x[0], y[0], s=110, marker="o", color="lime",
               edgecolor="black", zorder=6, label="start")
    ax.scatter(x[-1], y[-1], s=120, marker="s", color="red",
               edgecolor="black", zorder=6, label="end")

    # ── Remote vehicles (received LoRa) ───────────────────────────────────────
    lora_tracks: list[dict] = []
    if not args.no_lora:
        raw = Path(args.lora) if args.lora else find_raw_for(path)
        if raw is not None and raw.exists():
            ego_xy = (float(np.nanmedian(x)), float(np.nanmedian(y)))
            lora_tracks = load_lora(raw, d["mode"], d["origin"], d["t0_ms"],
                                    ego_xy=ego_xy, clip_m=args.lora_clip)
            for tr in lora_tracks:
                tag = "" if tr["node"] is None else f" #{tr['node']}"
                msg = f"[plot] remote{tag}: {len(tr['x'])} fixes"
                if tr.get("dropped"):
                    msg += f" ({tr['dropped']} dropped as >{args.lora_clip:.0f} m outliers)"
                print(msg)
        elif args.lora:
            print(f"[plot] --lora file not found: {args.lora}")
        elif not args.no_lora:
            print(f"[plot] no sibling raw_*.csv for {path.name}; no remote tracks")
    _RCOLORS = ["darkorange", "magenta", "saddlebrown", "teal", "indigo"]
    for k, tr in enumerate(lora_tracks):
        c = _RCOLORS[k % len(_RCOLORS)]
        rx, ry = tr["x"], tr["y"]
        label = f"remote{'' if tr['node'] is None else ' #' + str(tr['node'])}"
        ax.plot(rx, ry, "-", color=c, lw=1.6, alpha=0.9, zorder=3)
        ax.scatter(rx, ry, s=12, color=c, alpha=0.7, zorder=3)
        # Latest received position of this remote node.
        ax.scatter(rx[-1], ry[-1], s=90, marker="D", color=c,
                   edgecolor="black", zorder=6, label=label)
        if tr["alert"] is not None:
            al = ~np.isin(np.char.upper(tr["alert"].astype(str)),
                          ("NORMAL", "NAN", "", "NONE", "0"))
            if al.any():
                ax.scatter(rx[al], ry[al], s=60, marker="*", color="red",
                           edgecolor=c, zorder=7)
        # Remote heading arrows.
        if tr["heading"] is not None and args.arrows > 0 and len(rx) > 1:
            ridx = np.linspace(0, len(rx) - 1,
                               min(args.arrows, len(rx))).astype(int)
            rhd = np.radians(tr["heading"][ridx])
            ru, rv = np.sin(rhd), np.cos(rhd)
            span = max(np.nanmax(x) - np.nanmin(x),
                       np.nanmax(y) - np.nanmin(y), 1e-9)
            alen = 0.06 * span
            ax.quiver(rx[ridx], ry[ridx], ru, rv, color=c, alpha=0.5,
                      angles="xy", scale_units="xy", scale=1.0 / alen,
                      width=0.003, zorder=4)

    ax.set_xlabel(d["xlabel"])
    ax.set_ylabel(d["ylabel"])
    ax.set_title(f"Trajectory — {path.name}")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    if d["mode"] == "enu":
        ax.set_aspect("equal", adjustable="datalim")

    # ── Speed vs time ─────────────────────────────────────────────────────────
    ax_sp.plot(t, speed, lw=1.0, color="tab:blue", label="ego")
    for k, tr in enumerate(lora_tracks):
        if tr["speed"] is None:
            continue
        c = _RCOLORS[k % len(_RCOLORS)]
        rt = tr["t"] if tr["t"] is not None else np.arange(len(tr["speed"]))
        label = f"remote{'' if tr['node'] is None else ' #' + str(tr['node'])}"
        ax_sp.plot(rt, tr["speed"], lw=1.0, color=c, alpha=0.8, label=label)
    ax_sp.set_xlabel("Time (s)")
    ax_sp.set_ylabel("Speed (km/h)")
    ax_sp.set_title("Speed profile", fontsize=10)
    ax_sp.grid(True, alpha=0.3)
    if lora_tracks:
        ax_sp.legend(loc="best", fontsize=7)

    # ── Stats box ─────────────────────────────────────────────────────────────
    dist = path_length(x, y, d["mode"])
    dur = float(t[-1] - t[0]) if len(t) > 1 else 0.0
    stats = [
        f"file      : {path.name}",
        f"points    : {len(x):,}",
        f"duration  : {dur:6.1f} s",
        f"distance  : {dist:7.1f} m",
        f"max speed : {vmax:6.1f} km/h",
        f"avg speed : {np.nanmean(speed):6.1f} km/h",
    ]
    ax_tx.text(0.0, 1.0, "\n".join(stats), va="top", ha="left",
               family="monospace", fontsize=9,
               transform=ax_tx.transAxes)

    out = Path(args.out) if args.out else path.with_suffix(".png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"[plot] {len(x):,} points, {dist:.1f} m over {dur:.1f} s  →  {out}")

    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())

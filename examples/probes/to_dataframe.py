#!/usr/bin/env python3
"""
merge_probe_logs.py
(Cf main.c for short tutorial)

Parse Pogobot probe logs, merge into a single pandas DataFrame, and export
as .csv or .feather/.ftr. If output already exists, append + deduplicate.

Accepted input lines (examples):
  PROBE 50: robot=33, t=1000, state=0
  [..] [PRINTF] PROBE 50: robot=33, t=1000, state=0
  PROBE 1: robot 33 t=1000 ms state=RUN (0)
  PROBE 1: robot=33, t=1000 ms, state=TUMBLE (1)

Output columns:
  robot (int), t_ms (int), state (int)
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from typing import Iterable, Optional, Dict, Any, List

import pandas as pd


PROBE_PREFIX_RE = re.compile(r"\bPROBE\s+(\d+)\s*:\s*(.*)$")


def parse_probe_line(line: str) -> Optional[Dict[str, Any]]:
    """
    Return {'robot': int, 't_ms': int, 'state': int} if line contains a valid PROBE entry,
    else None.
    """
    m = PROBE_PREFIX_RE.search(line)
    if not m:
        return None

    rest = m.group(2)

    # robot id (avoid capturing the probe id by searching after the colon)
    m_robot = re.search(r"\brobot\b\s*=?\s*(\d+)", rest)
    if not m_robot:
        return None

    # time in ms (accept "t=123", "t=123," "t=123 ms")
    m_t = re.search(r"\bt\b\s*=?\s*(\d+)", rest)
    if not m_t:
        return None

    # state: prefer direct numeric, else number in parentheses (e.g., RUN (0), TUMBLE (1))
    m_state = re.search(r"\bstate\b\s*=?\s*(\d+)\b", rest)
    state_val: Optional[int] = None
    if m_state:
        state_val = int(m_state.group(1))
    else:
        m_state_paren = re.search(r"\bstate\b.*?\((\d+)\)", rest)
        if m_state_paren:
            state_val = int(m_state_paren.group(1))
        else:
            # last resort: take the last (...) integer anywhere after "state"
            if "state" in rest:
                tail = rest.split("state", 1)[1]
                nums = re.findall(r"\((\d+)\)", tail)
                if nums:
                    state_val = int(nums[-1])

    if state_val is None:
        return None

    return {
        "robot": int(m_robot.group(1)),
        "t_ms": int(m_t.group(1)),
        "state": int(state_val),
    }


def iter_lines(paths: List[str]) -> Iterable[str]:
    if not paths or paths == ["-"]:
        for line in sys.stdin:
            yield line
        return

    for p in paths:
        if p == "-":
            for line in sys.stdin:
                yield line
            continue
        with open(p, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                yield line


def read_existing(out_path: str) -> Optional[pd.DataFrame]:
    if not os.path.exists(out_path):
        return None

    ext = os.path.splitext(out_path.lower())[1]
    if ext == ".csv":
        return pd.read_csv(out_path)
    if ext in (".feather", ".ftr"):
        return pd.read_feather(out_path)

    raise ValueError(f"Unsupported output extension for existing file: {ext}")


def write_output(df: pd.DataFrame, out_path: str) -> None:
    ext = os.path.splitext(out_path.lower())[1]
    if ext == ".csv":
        df.to_csv(out_path, index=False)
        return
    if ext in (".feather", ".ftr"):
        # Requires pyarrow installed
        df.to_feather(out_path)
        return
    raise ValueError(f"Unsupported output extension: {ext} (use .csv or .feather/.ftr)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-o", "--output", required=True,
        help="Output path (.csv or .feather/.ftr). If it exists, new rows are merged into it."
    )
    ap.add_argument(
        "inputs", nargs="*", default=["-"],
        help="Input log file(s). Use '-' or omit to read from stdin."
    )
    ap.add_argument(
        "--keep-duplicates", action="store_true",
        help="Do not drop duplicate (robot, t_ms, state) rows."
    )
    ap.add_argument(
        "--no-sort", action="store_true",
        help="Do not sort by (robot, t_ms)."
    )
    args = ap.parse_args()

    rows: List[Dict[str, Any]] = []
    for line in iter_lines(args.inputs):
        rec = parse_probe_line(line)
        if rec is not None:
            rows.append(rec)

    new_df = pd.DataFrame.from_records(rows, columns=["robot", "t_ms", "state"])
    if len(new_df) == 0:
        # Still allow merging/writing an existing file unchanged (or creating an empty file).
        existing = read_existing(args.output)
        if existing is not None:
            df = existing
        else:
            df = new_df
        write_output(df, args.output)
        return 0

    # Normalize dtypes
    new_df = new_df.astype({"robot": "int64", "t_ms": "int64", "state": "int64"})

    existing = read_existing(args.output)
    if existing is not None and len(existing) > 0:
        # Ensure same columns exist (ignore extras in existing; keep ours)
        existing = existing.copy()
        for col in ["robot", "t_ms", "state"]:
            if col not in existing.columns:
                raise ValueError(f"Existing output file is missing required column '{col}'")
        existing = existing[["robot", "t_ms", "state"]]
        # Coerce dtypes safely
        existing = existing.astype({"robot": "int64", "t_ms": "int64", "state": "int64"})
        df = pd.concat([existing, new_df], ignore_index=True)
    else:
        df = new_df

    if not args.keep_duplicates:
        df = df.drop_duplicates(subset=["robot", "t_ms", "state"], keep="first")

    if not args.no_sort:
        df = df.sort_values(["robot", "t_ms", "state"], kind="mergesort").reset_index(drop=True)

    write_output(df, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


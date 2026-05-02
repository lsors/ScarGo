#!/usr/bin/env python3
"""每次构建时自动递增固件版本号，生成 main/include/version.h。

版本格式：YYYY.MM.DD.TOTAL.DAILY
  TOTAL  — 从项目创建开始的累计构建次数
  DAILY  — 当天的构建次数，每天从 1 重新开始
状态持久化在 version_state.json（需提交到 git）。
"""
import json
import os
from datetime import datetime

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATE_FILE  = os.path.join(SCRIPT_DIR, "version_state.json")
VERSION_H   = os.path.join(SCRIPT_DIR, "main", "include", "version.h")

today = datetime.now().strftime("%Y.%m.%d")

# 读取上次状态
if os.path.exists(STATE_FILE):
    with open(STATE_FILE, "r") as f:
        state = json.load(f)
else:
    state = {"date": "", "total": 0, "daily": 0}

total = state.get("total", 0) + 1
daily = (state.get("daily", 0) + 1) if state.get("date") == today else 1

# 保存新状态
with open(STATE_FILE, "w") as f:
    json.dump({"date": today, "total": total, "daily": daily}, f, indent=2)

# 写出 version.h
version_str = f"{today}.{total}.{daily}"
with open(VERSION_H, "w") as f:
    f.write("/* 自动生成 — 勿手动修改，由 version_bump.py 在每次构建时更新。 */\n")
    f.write("#pragma once\n\n")
    f.write(f'#define SCARGO_FW_VERSION_STR  "{version_str}"\n')

print(f"[version_bump] {version_str}")

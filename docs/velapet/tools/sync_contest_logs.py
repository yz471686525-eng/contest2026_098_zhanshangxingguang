#!/usr/bin/env python3
"""把归集暂存区里的 AI Coding 会话日志完整同步进本仓库的 logs/。

为什么需要这个脚本
------------------
组委会的 `contest-snapshot`(tools/export-session.py) 对跨天会话会静默漏掉日志。

`copy_session()` 从 manifest 条目的 `file_path` 反推 staging 源文件路径,而该
字段只含**单一日期**(随归集运行被改写);staging 实际是**按事件发生日**分文件
存的,一个会话跨天就有多个文件。由此产生两种故障:

A. 日期对不上 —— 打印 `source jsonl not found`,`rc=1`。会被发现,但措辞会让人
   误以为该会话没被采集到,实际文件就在 staging 里。

B. 日期对上但会话跨天 —— `copy_session()` 每个会话只复制**一个**文件,于是
   只导出 manifest 指向的那一天,其余天**静默丢失**,而且报告 ✅、退出码 0。
   这一种才是真正危险的。

本仓库 2026-08-19 实测:14 个会话中 8 个受影响 —— 7 个属 A(完全导不出),
1 个属 B:会话 2911cf58 在 staging 有 08-10(478 事件)与 08-11(31 事件),
manifest 指向 08-11,官方工具报成功却只复制了 31 事件那份,丢掉 478 个。

附带影响:manifest 的 `event_count` 因此是**单文件**事件数而非会话总数,跨天
会话会低报。文件内容本身完整,统计请以 jsonl 行数为准。

做法
----
不重新实现导出逻辑,也不手写 manifest。改为遍历 staging 里**真实存在**的每个
`<date>/<tool>__<session_id>.jsonl`,为其构造正确的 `file_path` 后交给官方的
`copy_session()` 处理,manifest 仍由官方 `update_dest_manifest()` 生成。

按日期升序处理,使同一会话的 manifest 条目最终指向最后一天(与官方行为一致)。

与 contest-log-collector-multiday.patch 的关系
---------------------------------------------
同目录下的 `contest-log-collector-multiday.patch` 是给官方工具的**根治补丁**,
已实测两者产出的 manifest 与文件完全等价。若组委会接受该补丁(或你在本地打上
它),`contest-snapshot --all --confirm` 就能直接导全,本脚本即可不再需要。
在此之前,用本脚本。

用法
----
    python3 docs/velapet/tools/sync_contest_logs.py            # 同步
    python3 docs/velapet/tools/sync_contest_logs.py --dry-run  # 只看会做什么

在仓库根目录执行。同步完成后:

    python3 ../.claude/skills/contest-log-collector/tools/validate-log.py logs
    git add logs/ && git commit -s -m "logs: sync sessions" && git push

任何一个文件同步失败都会以退出码 1 结束,便于接进脚本或 CI。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

SKILL_REL = Path(".claude/skills/contest-log-collector/tools/export-session.py")


def find_repo_root() -> Path:
    """从本脚本位置上溯,找到含 logs/ 与 .git 的仓库根。"""
    for parent in Path(__file__).resolve().parents:
        if (parent / ".git").exists() and (parent / "logs").is_dir():
            return parent
    raise SystemExit("找不到仓库根目录(需同时含 .git 与 logs/)")


def find_export_tool(repo: Path) -> Path:
    """定位官方 export-session.py。

    优先仓库同级的 openvela 工作区(与官方文档的 `../.claude/...` 一致),
    其次用户目录下的常见位置。
    """
    candidates = [
        repo.parent / SKILL_REL,
        Path.home() / SKILL_REL,
        Path.home() / "openvela" / SKILL_REL,
    ]
    for c in candidates:
        if c.is_file():
            return c
    raise SystemExit(
        "找不到官方 export-session.py,试过:\n  "
        + "\n  ".join(str(c) for c in candidates)
    )


def load_module(path: Path):
    spec = importlib.util.spec_from_file_location("export_session", path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"无法加载 {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main() -> int:
    ap = argparse.ArgumentParser(
        description="完整同步 AI Coding 会话日志到 logs/(修正官方跨天漏导)"
    )
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印将要复制的文件,不写入")
    args = ap.parse_args()

    repo = find_repo_root()
    tool = find_export_tool(repo)
    es = load_module(tool)

    staging = es.staging_root()
    if not staging.is_dir():
        raise SystemExit(f"暂存区不存在:{staging}\n"
                         "归集钩子可能未安装,或还没有会话结束过。")

    # staging 布局:<staging_root>/<github_login>/{manifest.json,<date>/*.jsonl}

    logins = [d.name for d in staging.iterdir()
              if (d / "manifest.json").is_file()]
    if not logins:
        raise SystemExit(f"{staging} 下没有找到含 manifest.json 的用户目录")
    if len(logins) > 1:
        raise SystemExit(f"暂存区有多个用户目录,不确定用哪个:{logins}")
    login = logins[0]

    manifest_path = staging / login / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    by_sid = {s["session_id"]: s for s in manifest.get("sessions", [])}

    # 只看 staging 里真实存在的按天文件,按 (日期, 文件名) 升序

    files = sorted((staging / login).glob("*/*.jsonl"),
                   key=lambda p: (p.parent.name, p.name))

    print(f"仓库     {repo}")
    print(f"暂存区   {staging / login}")
    print(f"官方工具 {tool}")
    print(f"按天文件 {len(files)} 个,manifest 记录 {len(by_sid)} 个会话")
    if args.dry_run:
        print("--dry-run:不会写入任何文件")
    print()

    ok = 0
    failed: list[str] = []
    orphan: list[str] = []

    for f in files:
        date_dir, fname = f.parent.name, f.name
        sid = fname.rsplit("__", 1)[-1].removesuffix(".jsonl")
        entry = by_sid.get(sid)

        if entry is None:
            # staging 有文件但 manifest 没条目。不能凭空造条目(schema 里的
            # started_at/model 等字段无从得知),交给使用者决定。
            print(f"  !  {date_dir}/{sid[:8]}  manifest 无此会话条目,跳过")
            orphan.append(f"{date_dir}/{fname}")
            continue

        s = dict(entry)
        s["github_login"] = login
        s["file_path"] = f"logs/{login}/{date_dir}/{fname}"

        good, msg = es.copy_session(s, repo, dry_run=args.dry_run)
        print(f"  {'OK' if good else 'X '} {date_dir}/{sid[:8]}  {msg}")
        if good:
            ok += 1
        else:
            failed.append(f"{date_dir}/{fname}: {msg}")

    print()
    print(f"同步 {ok} 个,失败 {len(failed)} 个,无 manifest 条目 {len(orphan)} 个")

    if orphan:
        print("\n以下文件在暂存区但 manifest 没有对应条目,需要人工确认:")
        for o in orphan:
            print(f"  {o}")

    if failed:
        print("\n失败项:")
        for f_ in failed:
            print(f"  {f_}")
        return 1

    if not args.dry_run:
        print("\n下一步:")
        print("  python3 ../.claude/skills/contest-log-collector/tools/"
              "validate-log.py logs")
        print("  git add logs/ && git commit -s -m 'logs: sync sessions'")

    return 0


if __name__ == "__main__":
    sys.exit(main())

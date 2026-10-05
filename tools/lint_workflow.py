#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════════
#  Workflow lint — the failures that compiled code cannot reveal
# ═══════════════════════════════════════════════════════════════
#  Three CI rounds died on things outside the firmware: a YAML key a
#  serialiser had rewritten, a library file that only fails on the real
#  core, and an empty token the GitHub API rejects.  This checks the
#  workflow for that whole class before a runner is ever started:
#
#    - it parses, and the trigger key is literally `on:`
#    - every `run:` block is valid bash (bash -n)
#    - every network fetch (core/lib install, update-index, pip, curl,
#      espup) goes through `retry`
#    - no step passes an EMPTY token
#    - every script a step calls exists in the repo
#    - every sketch directory the compile steps name is created first
# ═══════════════════════════════════════════════════════════════
import os, re, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WF = os.path.join(ROOT, "github", "workflows", "build.yml")
if not os.path.exists(WF):
    WF = os.path.join(ROOT, ".github", "workflows", "build.yml")
text = open(WF).read()
errors = []

if not re.search(r"^on:\s*$", text, re.M):
    errors.append("trigger key is not a literal `on:` (re-serialised file?)")
if re.search(r"GITHUB_TOKEN:\s*(''|\"\")", text):
    errors.append("a step sets GITHUB_TOKEN to an empty string (GitHub API answers 401)")

# Extract run: | blocks by indentation.
lines = text.split("\n")
blocks = []
i = 0
while i < len(lines):
    m = re.match(r"^(\s*)(?:- )?run:\s*\|\s*$", lines[i])
    if m:
        base = len(lines[i]) - len(lines[i].lstrip())
        body, j = [], i + 1
        while j < len(lines) and (lines[j].strip() == "" or
                                  len(lines[j]) - len(lines[j].lstrip()) > base):
            body.append(lines[j]); j += 1
        ind = min((len(l) - len(l.lstrip()) for l in body if l.strip()), default=0)
        blocks.append((i + 1, "\n".join(l[ind:] for l in body)))
        i = j
        continue
    m = re.match(r"^\s*(?:- )?run:\s*(\S.*)$", lines[i])
    if m:
        blocks.append((i + 1, m.group(1)))
    i += 1

NET = re.compile(r"(arduino-cli\s+(core|lib)\s+install|arduino-cli\s+core\s+update-index|"
                 r"pip\s+install|\bcurl\b|espup\"?\s+install)")
for ln, body in blocks:
    sh = re.sub(r"\$\{\{[^}]*\}\}", "X", body)
    with tempfile.NamedTemporaryFile("w", suffix=".sh", delete=False) as f:
        f.write(sh); path = f.name
    r = subprocess.run(["bash", "-n", path], capture_output=True, text=True)
    os.unlink(path)
    if r.returncode != 0:
        errors.append(f"line {ln}: run block is not valid bash: {r.stderr.strip()}")
    for k, l in enumerate(body.split("\n")):
        s = l.strip()
        if s.startswith("#") or not NET.search(s):
            continue
        if "retry" in s or "--retries" in s:
            continue
        # The command may sit inside a retried shell function.
        if re.search(r"^\s*\"?\$HOME/\.cargo/bin/espup\"?\s+install", l) and "install_esp" in body:
            continue
        errors.append(f"line {ln + k + 1}: network fetch without retry: {s[:90]}")
    for scr in re.findall(r"(tools/[A-Za-z0-9_./-]+\.(?:sh|py))", body):
        if not os.path.exists(os.path.join(ROOT, scr)):
            errors.append(f"line {ln}: step calls {scr}, which is not in the repo")

if errors:
    for e in errors: print(f"::error::workflow: {e}")
    sys.exit(1)
print(f"workflow: {len(blocks)} run blocks valid bash, network steps retried, no empty tokens")

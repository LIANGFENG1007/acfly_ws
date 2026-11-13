#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="${repo_root}/docs/assets"

if ! command -v mmdc >/dev/null 2>&1; then
  echo "未找到 mmdc。请安装 Mermaid CLI，或用 VS Code Mermaid 插件 / Mermaid Live Editor 导出。" >&2
  exit 127
fi

mmdc -i "${source_dir}/architecture.mmd" -o "${source_dir}/architecture.svg" -b transparent
mmdc -i "${source_dir}/config-layers.mmd" -o "${source_dir}/config-layers.svg" -b transparent
printf '%s\n' "已更新 docs/assets/architecture.svg 和 docs/assets/config-layers.svg"

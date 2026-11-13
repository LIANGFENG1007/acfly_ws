# 配图源文件

本目录同时保存可编辑源文件和 GitHub 预览文件。后续改模块名称、接口关系或配置层次时，先编辑 `.mmd`，再重新导出同名 `.svg`。SVG 只作为提交后的预览，不能作为唯一编辑源。

| 图 | 可编辑源文件 | 预览文件 | 说明 |
| --- | --- | --- | --- |
| 模块架构 | [architecture.mmd](architecture.mmd) | [architecture.svg](architecture.svg) | 外部系统、主控、探索、感知、公共层和启动入口的边界 |
| 配置分层 | [config-layers.mmd](config-layers.mmd) | [config-layers.svg](config-layers.svg) | 根索引、分类 YAML、节点声明和运行时覆盖 |

## 编辑和导出

Mermaid 源文件是纯文本，适合 Git diff、代码审查和后续批量修改。安装了 Mermaid CLI 后，在仓库根目录运行：

```bash
tools/render_diagrams.sh
```

脚本会把两个 `.mmd` 导出为同名 `.svg`。没有安装 `mmdc` 时，脚本会给出安装提示；也可以使用 VS Code Mermaid 插件或 Mermaid Live Editor 打开 `.mmd`。导出的 SVG 仍应保留在仓库中，便于 GitHub 直接显示。

任务流程没有单独画成固定图。任务步骤会随现场需求变化，统一写在 README 和项目概览的文字及接口表中，避免流程图过期。

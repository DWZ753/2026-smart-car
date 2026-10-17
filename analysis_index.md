# 分析资料索引

**维护位置**: E:/ti/projects/2026_smart_car。
**更新日期**: 2026-10-15。

- [已实现的分级应用与联调说明](application/car_app.md)。
- [当前 PVC 低速控制方案](pvc_control_plan.md)。
- [PID 参数与使用问题](pid_parameter_review.md)。
- [main 基线分析](findings.md)，历史分支结论。
- [2026h 应用分析](2026h_application_analysis.md)，历史比赛应用结论。
- [分析计划](task_plan.md)与[执行记录](progress.md)，末尾为最新范围。

当前先忽略路障，用 IMU pitch 判断坡道，低速开环跟踪约 60 Hz 的
视觉 dev/k；不依赖里程或车模/赛道精确几何。

## 证据与历史脚本

Debug/pvc_analysis 保存题目原图、文字提取、帧样例诊断。
Debug/2026h_23d6e82、main_validation、smart_car_migration 和
software_tests 保存当时快照、验证脚本与产物。
其中旧脚本可能保留 ti_cup 绝对路径，不能直接执行迁移脚本或
当作当前工程构建流程。日常构建使用本工程 CCS 配置。
历史报告中的复现命令只记录当时验证过程。

ti_cup 原文件保持原样，后续分析在本工程维护。
复制清单及哈希见 references/analysis_copy_manifest.json。

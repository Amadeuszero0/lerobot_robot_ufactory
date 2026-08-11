# Piper 历史版本快照：`ed30ad0`

本目录完整保存了 Git 提交
`ed30ad05ca528c30a0191cea5a1c8a5304a5f2ee` 中的 `piper/` 子目录。该提交名为
`Expose joint-stream classes to LeRobot factory`，提交时间是
2026-08-09 00:32:55（UTC+8）。

## 选择这个版本的原因

- 它的父提交 `dd3a611e9c54c586418c37c0db176966ce0ee0c8`，就是用户描述的
  8 月 8 日附近的大规模修改：共修改 9 个文件，新增 829 行、删除 36 行，提交名为
  `Add verified PiPER-X joint-stream control`。
- `ed30ad0` 在上述版本基础上，又补充了最后 12 行类型注册代码，使 LeRobot 能够发现并创建
  joint-stream 机器人类，因此它是这套实验实现最后一个完整可解析的版本。
- 紧接着的提交 `b125e21` 在恢复旧版双臂控制行为时，删除了 1294 行实验性运行配置。

## 目录用途

历史源码保存在本目录的 `piper/` 下，没有覆盖项目当前使用的 `piper/` 目录。
这里仅作为工程历史、代码比较和问题追踪快照，不是当前活动的运行包。

## 安全警告

此历史版本中的 PiPER-X joint-stream 实机实验后来出现过严重抖动。未经重新检查控制架构、
关节限位、急停机制和实机安全流程，请勿使用其中的驱动配置控制真实机械臂。

项目当前支持的实现仍位于仓库顶层的 `piper/` 目录。

## Git 校验命令

可以使用以下命令独立核对相关历史：

```bash
git show --stat dd3a611
git show --stat ed30ad0
git diff ed30ad0..HEAD -- piper
```

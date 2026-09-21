# 第三方源码与许可

本工作空间的定位实现以 [MIT-SPARK/KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher)
为基础。根目录的 [LICENSE](LICENSE) 说明本项目自有代码采用 MIT 许可；
它不替代 `vendor/` 内各项目的版权声明和许可。修改或重新打包第三方源码时，
应继续保留相应的原始声明。

| 目录 | 上游项目 | 本工作空间保留的许可 |
| --- | --- | --- |
| `vendor/kiss_matcher/` | [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) | [MIT，MIT-SPARK 版权声明](vendor/kiss_matcher/LICENSE) |
| `vendor/robin/` | [ROBIN](https://github.com/MIT-SPARK/ROBIN) | [ROBIN 许可及随附第三方声明](vendor/robin/LICENSE.md) |
| `vendor/small_gicp/` | [small_gicp](https://github.com/koide3/small_gicp) | [MIT](vendor/small_gicp/LICENSE) |
| `vendor/pmc/` | [PMC](https://github.com/MIT-SPARK/pmc) | [GPL v3 或更高版本声明](vendor/pmc/LICENSE.md)，[GPL v3 全文](vendor/pmc/COPYING) |
| `vendor/xenium/` | [xenium](https://github.com/mpoeter/xenium) | [MIT](vendor/xenium/LICENSE) |

这些项目内部还包含其他第三方源码及其许可文件；相应文件保留在各自的
`vendor/` 子目录中。`vendor/kiss_matcher/` 的 MIT 文本取自与本工作空间
固定的算法源码同一版本的 KISS-Matcher 仓库（提交
`8b58b118e00c7a5a22448741f8d956d638a44ae2`）。

当前构建脚本将 PMC 编译为静态库，并通过 ROBIN 链接进定位程序。因此，
根目录的 MIT 许可不能用于宣称最终组合程序只有 MIT 许可。发布编译产物时，
还需按 PMC 的 GPL 条款处理组合程序及对应源码。

仓库不包含实际部署用的地图和调试 PCD；这两类文件由使用者另行提供。

mc_rtc new plugin template
==

This project is a template for a new plugin wihtin [mc_rtc]

It comes with:
- a CMake project that can build a plugin for [mc_rtc], the project can be put within [mc_rtc] source-tree for easier updates
- clang-format files
- automated GitHub Actions builds on three major platforms

Quick start
--

1. Renaming the controller from `TorqueResidual` to `MyPlugin`. In a shell (Git Bash on Windows, replace sed with gsed on macOS):

```bash
sed -i -e's/TorqueResidual/MyPlugin/g' `find . -type f`
git mv src/TorqueResidual.cpp src/MyPlugin.cpp
git mv src/TorqueResidual.h src/MyPlugin.h
git mv etc/TorqueResidual.in.yaml etc/MyPlugin.in.yaml
```

2. You can customize the project name in vcpkg.json as well, note that this must follow [vcpkg manifest rules](https://github.com/microsoft/vcpkg/blob/master/docs/users/manifests.md)

3. Build and install the project

4. Run using your [mc_rtc] interface of choice, add `MyPlugin` to the `Plugins` configuration entry or enable the autoload option

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/

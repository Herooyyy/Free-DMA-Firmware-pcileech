# News
随着众多ANTICHEATS都推出了IOMMU-DMA数据通道错误检测，传统DMA方案已逐渐成为历史，近两个月来我一直在开发新的方式，目前仍有一些性能问题，例如不经常的low speed卡顿，但这已经够用了。

The dual-machine memory converter has been completed.

Support DMA links for all software

Characteristics:

1: No DMA board is required, and data transmission is driven through the network card RDMA, and any Pcileech software connection is supported.

2: Support VT-D/SVM Mode/IOMMU enablement

3: It cannot be shielded by any AC, and the local network card is used as the DMA medium.

双机内存转换器已经完成。

支持所有软件的DMA链接

特征：

1：无需DMA板，数据传输通过网卡RDMA驱动，并支持任何Pcileech软件连接。

2：支持VT-D/SVM模式/IOMMU启用

3：它不能被任何交流电屏蔽，本地网卡用作DMA介质。



# Firmware
PCI1734 released

Bypass Faceit/Vanguard

Realized the method of bypassing Faceit anti-cheating without passing through MSI/MSI-X

https://DMA.WANG

ASMedia USB3.0/3.1/Intel NMD NVme Updated


# Discord
if you don't had any important things do NOT request to join DC server.

https://discord.gg/rDydQaePa3

# About Source

Don't ask me any published src. Try do by urself.

ASMedia 3.2 USB for ZDMA 100t

PCI ISA BRIDGE for FACEIT

SATA for ACE bypass

# PCI-1734
请查看bar_controller.sv，基于LSFR随机tlp，通过ACE检测，目前的问题是Delta Force反馈击杀后踢出游戏。源码已上传至release，同时，如果你有好的想法，请联系我

# MSI-X Interrupt teach

## ASMedia XHCI Controller

你可以直接通过此文件生成MSI-X中断/计数
If you dont know how to release the msi-x interrupt, check the Release https://github.com/Herooyyy/Free-DMA-Firmware-pcileech/releases/tag/MSIX Here is a ASMedia MSI-X release


一些人很喜欢上传一些无用的代码,如果你想学习,我建议从此开始

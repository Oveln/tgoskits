//! `/dev/rt_shm`：AMP 跨核共享内存设备（rt-async ↔ StarryOS IPC）。
//!
//! 本文件是该驱动的**权威设计文档**：协议、缓存一致性模型、ioctl ABI 与
//! 日志策略都在这里维护（不另建过程文档）。OS 无关的设备逻辑在
//! `drivers/ipc/rt-shm`（Driver Core），本文件是 StarryOS Glue：解析设备树、
//! 提供平台 IPI 后端（QEMU virt = CLINT MSIP，K3 = mailbox4）、实现
//! DeviceOps/ioctl/mmap。
//!
//! ## 拓扑与协议
//!
//! AP（大核，StarryOS，S-mode）与 RP（rcpu1，rt-async，M-mode）经一块共享
//! SRAM 窗口上的 ov-channels `SharedMemory<3>` 通信：ch0 = AP→RP 请求，
//! ch1 = RP→AP 回包（`CH_FROM_RT_ASYNC`）。通知面用门铃中断代替轮询：
//! K3 为 mailbox4（AP=USER0/APLIC，RP=USER1/PLIC），RP 每写完一条消息即向
//! AP 的 FIFO 投一条 new_msg。
//!
//! 用户态协议纪律：**send → NOTIFY → AWAIT → recv**。NOTIFY/AWAIT 是内核
//! 缓存同步点（见下），偏离该纪律（如两次同步点之间再写共享窗）会触发
//! 64B 行部分写回覆盖对端数据的经典非一致缓存风险。
//!
//! ## 缓存一致性模型（双模式，boot 期 PBMT 探针自动选择）
//!
//! 共享 SRAM 的 PMA 为 cacheable（真 RAM），能否用页表属性压成非缓存取决于
//! **PBMT 是否生效**：X100 硬件声称 RVA23（强制 Svpbmt）、厂商 DT 也声明
//! svpbmt，但 S/U 模式用 PBMT 需要 M-mode 固件先置 `menvcfg.PBMTE`——
//! OpenSBI 1.6 仅在传入 DT 声明 svpbmt 且 priv≥1.12 时置位（sbi_hart.c），
//! 板上固件状态随刷机版本浮动，且被忽略时**不报错**（2026-08-15 板测即踩此，
//! 彼时只能靠 CBO 兜底）。因此本驱动 boot 期跑一次**功能探针**（写穿透
//! 判定，见 PbmtProbe）决定运行时模式：
//!
//! * **PbmtNc**（探针通过）：ioremap 的 IO 别名与用户态 NC mmap 真实非缓存，
//!   读写直达 SRAM——NOTIFY/AWAIT/mmap 的逐操作 CBO 同步点全部跳过，时延
//!   最优（小消息场景比全窗 CBO 扫掠快约一个量级）。
//! * **Cbo**（探针失败或无法执行）：不信任任何"NC 映射"语义（读写实际都走
//!   缓存），保留四处显式 CBO 同步点（zicbom，经 HAL `dcache_range` 分发）：
//!
//! | 时机 | 操作 | 方向/目的 |
//! |---|---|---|
//! | 设备初始化 | invalidate | **两种模式都保留**：丢弃启动链（SPL/U-Boot cacheable 写）遗留脏行，杜绝迟到写回——与 PBMT 无关的启动链卫生 |
//! | mmap | invalidate | Cbo 模式：清内核早期访问留下的驻留行，用户态首写直达 SRAM |
//! | NOTIFY（门铃前） | clean+invalidate | Cbo 模式：for-device，把本核滞留写推到 SRAM 再通知对端 |
//! | AWAIT（每次就绪检查前 + 返回前） | invalidate / clean+invalidate | Cbo 模式：for-cpu，读到对端已写入 SRAM 的回包真值 |
//!
//! 兜底：启动期早于初始化作废的写回，由 RP 侧 magic 自愈（幂等 re-init）
//! 消化。固件侧排查线索：OpenSBI 启动日志的 `Boot HART ISA Extensions` 行
//! 有无 svpbmt（无 → 固件 DT 老；有而探针失败 → PBMTE 未置或硅忽略）。
//!
//! ## ioctl ABI
//!
//! * `RT_SHM_IOC_NOTIFY`：clean+invalidate 窗口后向 RP 发门铃（写完消息后调用）。
//! * `RT_SHM_IOC_AWAIT`：阻塞至 ch1 有数据。就绪判定经内核映射（每次检查前
//!   作废缓存）读环索引，**空门铃（对端 ping 无消息）只会空唤醒后重睡**，
//!   不会假返回；返回前 clean+invalidate，保证随后用户态读为 SRAM 真值。
//! * `RT_SHM_IOC_CLR_PENDING`：兼容占位（就绪状态由环索引推导，无需清除）。
//! * `RT_SHM_IOC_TEST_MBOX`：软件注入 new_msg 自测中断全链路（K3 专用）。
//!
//! ## 设备树驱动配置
//!
//! 设备仅在 FDT 含 `ov,rt-async-amp` 节点时注册；共享窗/通知机制/中断号
//! 全部来自设备树（`amp.toml` 只管构建期常量，不再是本驱动的运行时真值源）。
//!
//! K3 绑定（`its/rt-async-k3.dts`，AP 侧为其等价节点）：
//! ```text
//! rt-async@c0800000 {
//!     compatible = "ov,rt-async-amp";
//!     reg = <0x0 0xc0800000 0x0 0x19000>;    // ≥ ov-channels footprint 0x18700
//!     notifier {
//!         compatible = "ov,k3-mailbox-notifier";
//!         reg = <0x0 0xcac91000 0x0 0x400>;   // mailbox4（esos 侧 disabled，独占）
//!         tx-channel = <0>;                   // AP 发门铃用 ch0（RP=USER1 监听）
//!         rx-channel = <1>;                   // AP=USER0 监听 ch1（RP 回包门铃）
//!         interrupt-parent = <&aplic>; interrupts = <217>;  // mailbox_irq4[0]
//!     };
//! };
//! ```
//!
//! **mailbox 硬件约束**：写 `mbox_msg[ch]` 会同时置位**双方 user** 的 ch
//! NEW_MSG 位（见 M-mode 侧 `chip-k3-rt24/src/mailbox.rs` 同款说明）——若
//! tx 与 rx 配成同一通道，本端发门铃会自激触发自己的中断线，故收发**必须**
//! 分通道（上述 0/1 配置即为此约定，解析默认值同）。
//!
//! QEMU virt 绑定把 notifier 换成 `ov,clint-msip-notifier`（reg 指向 MSIP1，
//! `ov,ipi-irq` 为 SSI cause），其余同构。
//!
//! ## 日志策略
//!
//! 一次性启动信息（初始化/中断注册/自测）、有界诊断（IRQ 计数前 5 次）、
//! 罕见事件（注册前排空残留门铃 `drained N stale message(s)`）与失败路径
//! 打印；高频路径（NOTIFY/AWAIT/每次缓存维护）静默。
//!
//! ## 验证状态
//!
//! 2026-08-15 真板（K3 RT24）三轮 RPC 全绿：通知回显 + ADD 请求/响应断言
//! （中断驱动，`user-test-ipc`）。

use core::sync::atomic::{AtomicUsize, Ordering};

use ax_kspin::SpinNoIrq;
use ax_memory_addr::{PhysAddr, PhysAddrRange};
use axfs_ng_vfs::{DeviceId, NodeFlags, VfsError, VfsResult};
use axpoll::{IoEvents, Pollable};
use rt_shm::{
    ChannelId, DEFAULT_CHANNELS, Event, IpiIrqEndpoint, IpiSender, OvChannelsShmAccess, RtShmCore,
    SharedMemory,
};

use crate::pseudofs::{DeviceMmap, DeviceOps};

// ── ioctl constants ────────────────────────────────────────────────────────

pub const RT_SHM_IOC_NOTIFY: u32 = 0x7350_01;
pub const RT_SHM_IOC_AWAIT: u32 = 0x7350_02;
pub const RT_SHM_IOC_CLR_PENDING: u32 = 0x7350_03;
/// AP 端纯自测：软件注入一条 mailbox new_msg，验证中断全链路
/// （mailbox → APLIC → IMSIC → CPU → handler），无需对端参与。
pub const RT_SHM_IOC_TEST_MBOX: u32 = 0x7350_04;

/// 伪字符设备号（major=10 = misc，minor=200 本仓自选，注册处防碰撞）。
pub const RT_SHM_DEVICE_ID: DeviceId = DeviceId::new(10, 200);

// ── Channel constants (protocol ABI, shared with M-mode rt-async) ──────────

/// rt-async → StarryOS 通道号 (M-mode 写，S-mode 读 has_pending)。
const CH_FROM_RT_ASYNC: ChannelId = ChannelId::new(1);

// ── Device-tree configuration ──────────────────────────────────────────────

/// 从设备树 `rt-async` 节点解析出的运行时配置。
///
/// 这是 `/dev/rt_shm` 是否建设备以及如何建设的唯一依据；
/// `amp.toml` 在本文件中不再被读取运行时参数。
pub struct RtAsyncConfig {
    shm_phys_base: usize,
    shm_size: usize,
    notifier: NotifierConfig,
}

enum NotifierConfig {
    ClintMsip {
        msip_phys: usize,
        ipi_irq: u32,
    },
    K3Mailbox {
        mbox_phys: usize,
        tx_channel: u8,
        rx_channel: u8,
        rx_irq: ax_runtime::hal::irq::IrqId,
    },
}

/// 解析设备树，返回 `rt_shm` 所需的运行时配置。
///
/// 返回 `None` 表示当前平台没有 `ov,rt-async-amp` 节点（或被 disable），
/// 调用方据此跳过 `/dev/rt_shm` 的创建。
///
/// 这是 `is_available()` 的演进：不仅判断"是否启用"，还一并给出配置。
pub fn config() -> Option<RtAsyncConfig> {
    parse_rt_async_from_fdt()
}

fn parse_rt_async_from_fdt() -> Option<RtAsyncConfig> {
    let fdt = ax_runtime::hal::dtb::get_fdt()?;

    // 主节点：compatible = "ov,rt-async-amp"，且 status 非 "disabled"。
    let node = fdt
        .find_compatible(&["ov,rt-async-amp"])
        .find(|n| !matches!(n.find_property("status").map(|p| p.str()), Some("disabled")))?;

    // SHM 区来自标准 `reg` (address-cells/size-cells 感知)。
    let reg = node.reg()?.next()?;
    let shm_phys_base = reg.address as usize;
    let shm_size = reg.size? as usize;

    // 通知子节点：fdt-parser 的 Node 无 children() 枚举，用 find_compatible
    // 跨树定位 notifier（其 compatible 唯一）。
    //
    // 注意：rt-async 节点存在但缺少 notifier 子节点（或 notifier 缺 reg）属于
    // 配置不完整 —— 与"节点不存在"同等处理（跳过设备建设），而非 panic 内核。
    // 这样将来引入新通知机制（如 K3 mailbox）而 DTS 尚未更新时，不会让内核
    // 直接崩溃，而是优雅降级。
    let notifier = if let Some(n) = fdt.find_compatible(&["ov,clint-msip-notifier"]).next() {
        match n.reg().and_then(|mut r| r.next()) {
            Some(nr) => {
                let ipi_irq = n.find_property("ov,ipi-irq").map(|p| p.u32()).unwrap_or(1);
                NotifierConfig::ClintMsip {
                    msip_phys: nr.address as usize,
                    ipi_irq,
                }
            }
            None => {
                ax_println!("rt_shm: clint-msip-notifier missing reg, skipping /dev/rt_shm");
                return None;
            }
        }
    } else if let Some(n) = fdt.find_compatible(&["ov,k3-mailbox-notifier"]).next() {
        match n.reg().and_then(|mut r| r.next()) {
            Some(nr) => {
                let tx_channel = n
                    .find_property("tx-channel")
                    .map(|p| p.u32() as u8)
                    .unwrap_or(0);
                let rx_channel = n
                    .find_property("rx-channel")
                    .map(|p| p.u32() as u8)
                    .unwrap_or(1);
                // mailbox 中断线解析：notifier 节点用标准 interrupt-parent +
                // interrupts（mailbox_irq4[0] → APLIC source 217）。经 rdrive
                // phandle → DeviceId → intc.translate_fdt 得到**带正确 domain**
                // （K3 为 APLIC 动态 domain）的 IrqId，替代硬编码
                // RISCV_PLIC_DOMAIN（固定保留 id，K3 上不可注册）。
                //
                // 解析失败（缺 interrupt-parent / controller 未注册 / specifier
                // 非法）按配置不完整处理：跳过设备建设，而非 panic 内核。
                let rx_irq = match k3_notifier_irq() {
                    Some(irq) => irq,
                    None => {
                        ax_println!(
                            "rt_shm: k3-mailbox-notifier missing/invalid interrupt binding, \
                             skipping /dev/rt_shm"
                        );
                        return None;
                    }
                };
                NotifierConfig::K3Mailbox {
                    mbox_phys: nr.address as usize,
                    tx_channel,
                    rx_channel,
                    rx_irq,
                }
            }
            None => {
                ax_println!("rt_shm: k3-mailbox-notifier missing reg, skipping /dev/rt_shm");
                return None;
            }
        }
    } else {
        ax_println!("rt_shm: rt-async node has no notifier child, skipping /dev/rt_shm");
        return None;
    };

    Some(RtAsyncConfig {
        shm_phys_base,
        shm_size,
        notifier,
    })
}

/// 解析 `ov,k3-mailbox-notifier` 节点的标准 interrupt binding，返回带正确
/// interrupt domain（K3 上为 APLIC 动态 domain）的 `IrqId`。
///
/// 经 rdrive 全局 FDT 找节点 → interrupt-parent phandle → DeviceId →
/// `intc.translate_fdt` 的完整链路（与 kpu devfs 的中断解析一致），保证
/// IRQ 注册落到真实的中断控制器 domain 上，而非硬编码的保留 domain id。
fn k3_notifier_irq() -> Option<ax_runtime::hal::irq::IrqId> {
    let interrupt = rdrive::with_fdt(|fdt| {
        fdt.find_compatible(&["ov,k3-mailbox-notifier"])
            .first()
            .and_then(|node| node.interrupts().into_iter().next())
    })??;
    let controller = rdrive::fdt_phandle_to_device_id(interrupt.interrupt_parent)?;
    ax_runtime::irq::resolve_binding_irq(ax_driver::BindingIrq::fdt_interrupt_with_controller(
        controller,
        interrupt.specifier.clone(),
    ))
    .ok()
}

// ── StarryOS Glue: CLINT IPI sender ────────────────────────────────────────

struct ClintIpiSender {
    msip_ptr: *mut u32,
}

// SAFETY: The pointer points to a fixed MMIO register (CLINT MSIP1).
// It is valid for the entire lifetime of the system and accessed with
// volatile writes only. The SpinNoIrq wrapper ensures mutual exclusion.
unsafe impl Send for ClintIpiSender {}
unsafe impl Sync for ClintIpiSender {}

impl ClintIpiSender {
    /// # Panics
    /// 若 ioremap 失败则 panic（boot 阶段配置错误，应立刻暴露）。
    fn new(msip_phys: usize) -> Self {
        // CLINT MSIP 是 MMIO 区，不在内核线性 RAM 映射内，需显式 ioremap。
        let page_aligned = msip_phys & !0xFFF;
        let mmio = unsafe {
            mmio_api::ioremap_raw(page_aligned.into(), 0x1000)
                .expect("rt_shm: failed to ioremap CLINT MSIP")
        };
        let offset = msip_phys - page_aligned;
        Self {
            msip_ptr: unsafe { mmio.as_nonnull_ptr().as_ptr().add(offset) as *mut u32 },
        }
    }
}

impl IpiSender for ClintIpiSender {
    #[cfg(target_arch = "riscv64")]
    fn notify_peer(&mut self) {
        // iorw,iorw 全序栅栏（普通+IO 读写）：既覆盖 Cbo 模式的 CBO 后排序，
        // 也严格保证 PbmtNc 模式下 NC store 先于门铃 MMIO write 对总线可见
        // （fence rw,rw 对 IO 后继的排序在规范层面不足）。
        unsafe { core::arch::asm!("fence iorw, iorw", options(nostack, preserves_flags)) };
        unsafe {
            core::ptr::write_volatile(self.msip_ptr, 1);
        }
    }

    #[cfg(not(target_arch = "riscv64"))]
    fn notify_peer(&mut self) {}
}

// ── StarryOS Glue: K3 Mailbox ──────────────────────────────────────────────

// 寄存器布局出自 K3 TRM mailbox 章（与 M-mode 侧 chip-k3-rt24/src/mailbox.rs
// 的定义保持一致，两处改动需同步）。
const K3_MBOX_MSG_BASE: usize = 0x040;
const K3_MBOX_MSG_STATUS_BASE: usize = 0x0c0;
const K3_MBOX_IRQ_BASE: usize = 0x100;
const K3_MBOX_IRQ_STRIDE: usize = 0x10;
const K3_MBOX_IRQ_STATUS_OFF: usize = 0x00;
const K3_MBOX_IRQ_CLR_OFF: usize = 0x04;
const K3_MBOX_IRQ_EN_SET_OFF: usize = 0x08;

/// new_msg 中断掩码：通道 m 占 bit 2m（每个通道 2 个状态位，new_msg 在偶位）。
const fn k3_new_msg_mask(ch: u8) -> u32 {
    1u32 << (ch * 2)
}

const fn k3_irq_reg(user: u8, reg_off: usize) -> usize {
    K3_MBOX_IRQ_BASE + (user as usize) * K3_MBOX_IRQ_STRIDE + reg_off
}

struct K3MboxMmio {
    base: *mut u8,
    user_local: u8,
}

// SAFETY: Points to a fixed MMIO region (mailbox4 registers). Valid for the
// entire system lifetime, accessed with volatile only. 三类使用方并发存在：
// IpiSender 路径（SpinNoIrq on RtShmDevice 保护）、IRQ endpoint（非重入
// callback）、boot 自测（故意与 IRQ handler 并发触发中断）。MMIO 寄存器
// 各自独立、无跨寄存器不变量，volatile 访问下无数据竞争。
unsafe impl Send for K3MboxMmio {}
unsafe impl Sync for K3MboxMmio {}

impl K3MboxMmio {
    fn new(base: *mut u8, user_local: u8) -> Self {
        Self { base, user_local }
    }

    #[inline]
    fn read(&self, offset: usize) -> u32 {
        unsafe { core::ptr::read_volatile(self.base.add(offset) as *const u32) }
    }

    #[inline]
    fn write(&self, offset: usize, value: u32) {
        unsafe { core::ptr::write_volatile(self.base.add(offset) as *mut u32, value) }
    }

    fn signal_to_remote(&self, channel: u8, user_remote: u8) {
        let mask = k3_new_msg_mask(channel);
        self.write(k3_irq_reg(user_remote, K3_MBOX_IRQ_EN_SET_OFF), mask);
        self.write(K3_MBOX_MSG_BASE + (channel as usize) * 4, 1);
    }

    /// 应答并清 pending——按 M-mode 侧 `mbox_isr`
    /// （modules/chip-k3-rt24/src/mailbox.rs，与 esos 驱动同模式）的硬件语义：
    ///
    /// * NEW_MSG 是**锁存电平**：只要曾有待读消息就拉高并 latch；
    /// * CLR 写生效的前提是之前**至少一次 mbox_msg 读**作 acknowledge，且
    ///   FIFO 仍非空时写 CLR 只临时清 latch、硬件立即重新置位；
    /// * AP 侧 APLIC **MSI 模式**对电平源只在 setip 0→1 跳变时投一次 MSI，
    ///   不像 PLIC 会持续重投——若 burst 门铃撞上 CLR 写（FIFO 瞬时非空），
    ///   CLR 不生效、中断线永久挂高，之后所有门铃全部隐形（板上实锤
    ///   2026-08-15：count=2 后再无 IRQ，停靠的 AWAIT 永久挂死）。
    ///
    /// 故必须外层循环：排空 FIFO（读一次 ack + 按 num_msg 判空）→ RMW 清
    /// pending → 重读 `RAW & EN`，真正归零才返回；后续门铃才能产生新的
    /// setip 上升沿。上限防 IRQ 上下文活锁。
    fn ack_and_clear(&self, channel: u8) {
        let ch = channel as usize;
        let mask = k3_new_msg_mask(channel);
        let status = k3_irq_reg(self.user_local, K3_MBOX_IRQ_STATUS_OFF);
        let clr = k3_irq_reg(self.user_local, K3_MBOX_IRQ_CLR_OFF);
        let en = k3_irq_reg(self.user_local, K3_MBOX_IRQ_EN_SET_OFF);
        for _round in 0..32 {
            if self.read(status) & self.read(en) & mask == 0 {
                break;
            }
            // 排空 FIFO：先读一次 mbox_msg（acknowledge）再按 num_msg 判空，
            // 对齐 M-mode/esos 的 while(1){readl;check;break} 模式。
            for _ in 0..64 {
                let _ = self.read(K3_MBOX_MSG_BASE + ch * 4);
                if self.read(K3_MBOX_MSG_STATUS_BASE + ch * 4) & 0xF == 0 {
                    break;
                }
            }
            // 清 pending：RMW 保留其他通道位（esos 同款；CLR 可读）。
            let cur = self.read(clr);
            self.write(clr, cur | mask);
        }
    }

    fn enable_rx(&self, channel: u8) {
        self.write(
            k3_irq_reg(self.user_local, K3_MBOX_IRQ_EN_SET_OFF),
            k3_new_msg_mask(channel),
        );
    }

    /// 自测用：模拟对端发来一条 new_msg，触发本端（USER0）的 new_msg 中断线
    /// （mailbox_irq4[0] → APLIC → IMSIC → CPU → handler 全链路），不需要对端
    /// 参与。写入会占用 FIFO 一槽，由中断 handler 的 ack_and_clear 读走。
    fn inject_local_message(&self, channel: u8) {
        self.write(
            k3_irq_reg(self.user_local, K3_MBOX_IRQ_EN_SET_OFF),
            k3_new_msg_mask(channel),
        );
        self.write(K3_MBOX_MSG_BASE + (channel as usize) * 4, 1);
    }

    /// 调试：读本地 user 的 IRQSTATUS_RAW / IRQENABLE_SET 原始值。
    /// 自测失败时打印，区分"mailbox 未置位"与"置位了但 APLIC/IMSIC 未交付"。
    fn debug_irq_raw(&self) -> (u32, u32) {
        (
            self.read(k3_irq_reg(self.user_local, K3_MBOX_IRQ_STATUS_OFF)),
            self.read(k3_irq_reg(self.user_local, K3_MBOX_IRQ_EN_SET_OFF)),
        )
    }

    /// 复位接收通道：读空 FIFO 并清 pending，把 NEW_MSG 电平放下来。
    ///
    /// NEW_MSG 是"FIFO 非空"电平信号：若对端（rt-async 的门铃 ping）在本
    /// driver 注册 handler **之前**写过 FIFO，残留消息会让 IRQ 线提前挂起，
    /// 与 APLIC/IMSIC 的交付路径互锁（板上实测：RAW=0x04 恒置位、handler
    /// 从不触发）。注册前复位电平，自测与真实中断从干净状态开始。
    fn reset_rx(&self, channel: u8) {
        let mask = k3_new_msg_mask(channel);
        let mut drained = 0;
        while self.read(k3_irq_reg(self.user_local, K3_MBOX_IRQ_STATUS_OFF)) & mask != 0
            && drained < 32
        {
            let _ = self.read(K3_MBOX_MSG_BASE + (channel as usize) * 4);
            self.write(k3_irq_reg(self.user_local, K3_MBOX_IRQ_CLR_OFF), mask);
            drained += 1;
        }
        if drained > 0 {
            ax_println!(
                "rt_shm: mailbox rx ch{channel} drained {drained} stale message(s) before enable"
            );
        }
    }
}

struct K3MailboxIpiSender {
    mbox: K3MboxMmio,
    tx_channel: u8,
    user_remote: u8,
}

impl IpiSender for K3MailboxIpiSender {
    fn notify_peer(&mut self) {
        // 同 ClintIpiSender：iorw,iorw 全序栅栏，严格保证 NC/CBO 后的共享窗
        // 写先于 mailbox 门铃 MMIO write 总线可见（riscv64 专属指令，多架构
        // 编译门控）。
        #[cfg(target_arch = "riscv64")]
        unsafe {
            core::arch::asm!("fence iorw, iorw", options(nostack, preserves_flags))
        };
        self.mbox
            .signal_to_remote(self.tx_channel, self.user_remote);
    }
}

struct K3MailboxIrqEndpoint {
    mbox: K3MboxMmio,
    rx_channel: u8,
}

impl IpiIrqEndpoint for K3MailboxIrqEndpoint {
    fn handle_irq(&mut self) -> Event {
        self.mbox.ack_and_clear(self.rx_channel);
        Event::PeerNotify
    }
}

// ── Waker storage ──────────────────────────────────────────────────────────

/// AWAIT/Pollable 的单槽 waker。
///
/// 单等待者假设：当前无强制独占打开，若两个线程同时 AWAIT（或 epoll 与
/// AWAIT 并存），后注册者会覆盖先注册者、先注册者可能丢失唤醒——协议
/// 上由"每个进程一个 IPC 等待线程"的用法约定保证。
static IPC_WAKER: SpinNoIrq<Option<core::task::Waker>> = SpinNoIrq::new(None);
/// K3 mailbox new_msg 中断触发次数（自测用：ioctl TEST_MBOX 轮询此计数确认
/// mailbox → APLIC → CPU → handler 全链路贯通）。
static K3_MBOX_IRQ_COUNT: AtomicUsize = AtomicUsize::new(0);

// ── IPI handler (raw function pointer for irq-framework) ───────────────────

/// 唤醒 AWAIT/epoll 等待者（IRQ 上下文调用，两个 IPI 后端共用）。
fn wake_ipc_waiter() {
    if let Some(waker) = IPC_WAKER.lock().take() {
        waker.wake();
    }
}

#[cfg(target_arch = "riscv64")]
fn ipi_irq_handler(_ctx: ax_runtime::hal::irq::IrqContext) -> ax_runtime::hal::irq::IrqReturn {
    wake_ipc_waiter();
    ax_runtime::hal::irq::IrqReturn::Handled
}

// ── Shared-window cache maintenance ────────────────────────────────────────

/// 作废共享窗口在本核缓存中的陈旧行（丢弃，不写回）。
///
/// 三个调用场景（均为"for-cpu"同步点，详见模块文档「缓存一致性模型」）：
/// * 设备初始化：丢弃启动链（BootROM/SPL/U-Boot cacheable 写）遗留的脏行，
///   杜绝迟到写回覆盖 rt-async 数据（ch1/ch2 magic 被清零、SPL 指令字节
///   "复活"、RPC 请求被覆盖——均为该根因的板上实锤）；
/// * mmap：清掉内核早期 cacheable 访问可能留下的驻留行；
/// * AWAIT 每次就绪检查前：X100 上 PBMT 不生效（见模块文档），ioremap
///   "NC"别名的读也走缓存，首查取入的"空"状态会让重查恒 false——先作废
///   再读才能看到对端已写入 SRAM 的回包。
///
/// 缓存操作经 HAL 平台实现分发（K3 走 zicbom cbo.inval + fence；无 zicbom
/// 的平台退化为 DMA fence），本函数只作废缓存行、不改 SRAM 内容，对
/// rt-async 已写入的数据无损。早于初始化作废发生的写回由 rt-async 侧的
/// magic 自愈兜底（rt-async shm_ping：监视三通道 magic，丢失即幂等 re-init）。
///
/// 本函数刻意不打日志：AWAIT 每次（被唤醒后的）轮询都会调用，属高频路径。
fn invalidate_shm_window(vaddr: usize, size: usize) {
    ax_runtime::hal::mem::dcache_range(
        ax_runtime::hal::mem::DCacheOp::Invalidate,
        ax_memory_addr::VirtAddr::from(vaddr),
        size,
    );
}

/// 共享窗口 clean+invalidate（把本核缓存里可能滞留的写推到 SRAM 再作废）。
///
/// NOTIFY 前调用——DMA-to-device 同款所有权屏障：用户态 NC 写在个别缓存
/// 层（如 LLC）存在驻留行时会被"吸收"不达 SRAM（板上实锤：AP 回读
/// ch0.write=1 而 RP 读 0），clean 把滞留的新值推到 SRAM，invalidate 作废
/// 副本，然后才打门铃，保证对端看到全部写入。
fn flush_shm_window(vaddr: usize, size: usize) {
    ax_runtime::hal::mem::dcache_range(
        ax_runtime::hal::mem::DCacheOp::CleanInvalidate,
        ax_memory_addr::VirtAddr::from(vaddr),
        size,
    );
}

// ── Coherence mode & PBMT probe ────────────────────────────────────────────

/// 共享窗运行时一致性模式（boot 期由 PBMT 探针判定，见模块文档
/// 「缓存一致性模型」）。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum CoherenceMode {
    /// PBMT 生效：IO/NC 映射真实非缓存，读写直达 SRAM，逐操作 CBO
    /// 同步点全部跳过（小消息场景时延最优）。
    PbmtNc,
    /// PBMT 不生效或未知：不信任任何映射属性，保留全部 CBO 同步点。
    Cbo,
}

/// PBMT 探针 magic（A=prime 脏行值，B=写穿透判定值，互异且非全 0/1）。
const PBMT_PROBE_A: u64 = 0xA5A5_5A5A_A5A5_5A5A;
const PBMT_PROBE_B: u64 = 0x5A5A_A5A5_5A5A_A5A5;

/// PBMT 写穿透探针（boot 期一次性，跨 ioremap 分两段执行）。
///
/// 利用 ioremap 前后**同一虚地址**的属性变化（cacheable 线性映射 → IO/PBMT
/// 重写）构造判定序列：
///
/// 1. `prime`（ioremap 前）：经 cacheable 线性映射向 scratch 行写 A——制造
///    本核驻留脏行；
/// 2. 调用方执行 `ioremap_raw`（窗口 PTE 重写为 IO 属性，TLB 由映射层刷新）；
/// 3. `finish`（ioremap 后）：经（名义上的）IO 映射写 B。PBMT 生效 ⇒ B
///    绕过缓存直达 SRAM，脏行 A 不受影响；PBMT 被固件/硬件忽略 ⇒ 写命中
///    同一缓存行，B 只是覆盖 A，SRAM 仍是旧值；
/// 4. 作废 scratch 行后回读：== B ⇒ PBMT 生效；否则被忽略。
///
/// scratch 取窗口最后一行：ov-channels footprint（0x18700）之外、双端协议
/// 均不触碰的空闲区；探针后该行内容为垃圾（若未来窗口布局收缩到 footprint
/// 贴边需重审此假设）。窗口不足 64B 或尾行未按 CBO 块对齐时返回 None，
/// 放弃探针、保守走 [`CoherenceMode::Cbo`]。
struct PbmtProbe {
    /// scratch 行虚地址（窗口最后一行，64B 对齐）。
    scratch: usize,
}

impl PbmtProbe {
    /// 第一段：ioremap 前在尾行制造 cacheable 脏行。
    fn prime(lin_vaddr: usize, shm_size: usize) -> Option<Self> {
        let scratch = lin_vaddr.checked_add(shm_size)? - 64;
        if scratch & 0x3f != 0 {
            return None;
        }
        // SAFETY: 窗口内已映射的普通内存，对齐的单字 volatile 访问。
        unsafe { core::ptr::write_volatile(scratch as *mut u64, PBMT_PROBE_A) };
        Some(Self { scratch })
    }

    /// 第二段：ioremap 后执行写穿透判定（见类型文档步骤 3-4）。
    fn finish(self) -> bool {
        // SAFETY: 同一虚地址，PTE 已被 ioremap 重写为 IO 属性（PBMT=IO），
        // volatile 单字访问。
        unsafe { core::ptr::write_volatile(self.scratch as *mut u64, PBMT_PROBE_B) };
        invalidate_shm_window(self.scratch, 64);
        // SAFETY: 回读走（名义上的）IO 映射：PBMT 生效时为非缓存读，取
        // SRAM 真值；被忽略时为缓存读，作废后重取 SRAM 旧值。
        let readback = unsafe { core::ptr::read_volatile(self.scratch as *const u64) };
        // 尾行清理（best-effort）：生效路径下 prime 制造的脏行 A 可能仍驻留
        // （IO 映射上的 CBO 行为依实现而定），clean+invalidate 把可能的滞留写
        // 推出，杜绝迟到写回；判读在清理前完成，不受影响。
        flush_shm_window(self.scratch, 64);
        readback == PBMT_PROBE_B
    }
}

/// 诊断辅助：FDT cpu 节点是否声明 svpbmt。只影响启动日志、不参与判定——
/// 用于区分探针失败的两种形态：「DT 没声明 → 固件/DT 老」与「DT 声明了但
/// 探针失败 → menvcfg.PBMTE 未置或硅忽略」。fdt-parser 无子节点枚举，
/// 按 cpu@N 命名惯例探测前 8 个槽位。
fn fdt_declares_svpbmt() -> Option<bool> {
    const CPU_PATHS: [&str; 8] = [
        "/cpus/cpu@0",
        "/cpus/cpu@1",
        "/cpus/cpu@2",
        "/cpus/cpu@3",
        "/cpus/cpu@4",
        "/cpus/cpu@5",
        "/cpus/cpu@6",
        "/cpus/cpu@7",
    ];
    let fdt = ax_runtime::hal::dtb::get_fdt()?;
    for path in CPU_PATHS {
        if let Some(isa) = fdt
            .find_nodes(path)
            .next()
            .and_then(|n| n.find_property("riscv,isa"))
        {
            return Some(isa.str().contains("svpbmt"));
        }
    }
    None
}

// ── RtShmDevice ────────────────────────────────────────────────────────────

enum IpiBackend {
    Clint(ClintIpiSender),
    K3Mbox(K3MailboxIpiSender),
}

impl IpiSender for IpiBackend {
    fn notify_peer(&mut self) {
        match self {
            Self::Clint(s) => s.notify_peer(),
            Self::K3Mbox(s) => s.notify_peer(),
        }
    }
}

pub struct RtShmDevice {
    core: SpinNoIrq<RtShmCore<OvChannelsShmAccess<DEFAULT_CHANNELS>, IpiBackend>>,
    shm_phys_base: usize,
    shm_size: usize,
    /// 共享窗的内核线性映射虚地址（固定 PA 的确定值，new() 算一次）。
    /// Cbo 模式的缓存同步点（NOTIFY/AWAIT/mmap）都经它做 CBO。
    lin_vaddr: usize,
    /// 运行时一致性模式（boot 期 PBMT 探针判定）：决定逐操作 CBO 同步点
    /// 是否执行，见模块文档「缓存一致性模型」。
    mode: CoherenceMode,
    /// 共享内存的 ioremap NC 别名映射句柄（保持映射存活；访问经其指针进行）。
    _shm_nc: mmio_api::MmioRaw,
    /// K3 mailbox 自测句柄（mmio + DT 配置的 rx_channel）：仅 K3 分支构造时
    /// Some，其余平台 None。供 ioctl TEST_MBOX 软件注入 new_msg 验证中断
    /// 全链路。注意 mailbox 通道号与 ov-channels 通道号是两个不相干的
    /// 命名空间，自测必须用 DT 的 rx-channel 而非 `CH_FROM_RT_ASYNC`。
    test_mbox: Option<(K3MboxMmio, u8)>,
}

impl RtShmDevice {
    /// 从 DT 配置构建设备：缓存同步点（boot invalidate）、内核 NC 别名、
    /// 按 notifier 后端构造 IPI sender 并注册中断、boot 自测。
    ///
    /// # Panics
    ///
    /// 共享窗或 mailbox 的 ioremap 失败时 panic——二者来自 DT 保留区，映射
    /// 失败属于启动期配置错误，应立刻暴露而非带病运行（对比 DT 节点缺失/
    /// 不完整走优雅降级，见 [`config`]）。
    pub fn new(cfg: RtAsyncConfig) -> Self {
        let RtAsyncConfig {
            shm_phys_base,
            shm_size,
            notifier,
        } = cfg;

        // SAFETY: Address from DT reserved SHM region. Layout matches M-mode
        // SharedMemory::<DEFAULT_CHANNELS> with the same feature configuration.
        let lin_vaddr =
            ax_runtime::hal::mem::phys_to_virt(PhysAddr::from(shm_phys_base)).as_ptr() as usize;
        // K3：作废启动链（BootROM/SPL/U-Boot）遗留在缓存中的陈旧脏行，
        // 杜绝迟到写回覆盖 rt-async 数据（详见 invalidate_shm_window 注释）。
        // **两种模式都保留**——这是与 PBMT 无关的启动链卫生。
        invalidate_shm_window(lin_vaddr, shm_size);
        ax_println!("rt_shm: stale boot-chain cache lines over shm window invalidated");
        // PBMT 探针第一段：ioremap 重写窗口 PTE 前，先在尾行制造 cacheable
        // 脏行（完整序列见 PbmtProbe 文档）。
        let pbmt_probe = PbmtProbe::prime(lin_vaddr, shm_size);
        // 内核访问共享内存改走 ioremap 的 non-cacheable 别名（与 mailbox 同款），
        // 不经 cacheable 线性映射：内核读（is_valid/has_pending）永远新鲜，
        // 也不会在缓存里制造新的驻留行——驻留行会把用户态 NC 写"吸收"在
        // 缓存里不达 SRAM（板上实锤：AP 回读 ch0.write=1 而 RP 读 0）。
        // SAFETY: 地址来自 DT 保留区，与 M-mode 布局一致；NC 映射 lifetime
        // 与设备相同（指针全程有效）。
        let shm_nc = unsafe { mmio_api::ioremap_raw(shm_phys_base.into(), shm_size) }
            .expect("rt_shm: failed to ioremap shm window");
        // PBMT 探针第二段：经 IO 属性映射做写穿透判定，确定运行时一致性
        // 模式（探针不可执行或判失败时保守走 Cbo）。
        let mode = match pbmt_probe {
            Some(probe) => {
                if probe.finish() {
                    CoherenceMode::PbmtNc
                } else {
                    CoherenceMode::Cbo
                }
            }
            None => CoherenceMode::Cbo,
        };
        ax_println!(
            "rt_shm: PBMT probe: effective={} dt_svpbmt={:?} -> {:?} mode (Cbo=fallback with \
             per-op sync points)",
            mode == CoherenceMode::PbmtNc,
            fdt_declares_svpbmt(),
            mode
        );
        let vaddr = shm_nc.as_nonnull_ptr().as_ptr() as usize;
        let shm: &'static SharedMemory<DEFAULT_CHANNELS> = unsafe { SharedMemory::at(vaddr) };
        let access = unsafe { OvChannelsShmAccess::new(shm, CH_FROM_RT_ASYNC) };

        let mut test_mbox = None;
        let ipi: IpiBackend = match notifier {
            NotifierConfig::ClintMsip { msip_phys, ipi_irq } => {
                #[cfg(target_arch = "riscv64")]
                {
                    use ax_runtime::hal::irq::{
                        AutoEnable, CPU_LOCAL_IRQ_DOMAIN, CpuMask, HwIrq, IrqId, IrqRequest,
                        IrqScope, ShareMode, request_irq,
                    };
                    let irq = IrqId::new(CPU_LOCAL_IRQ_DOMAIN, HwIrq(ipi_irq as _));
                    let cpus = CpuMask::first_n(ax_runtime::hal::cpu_num());
                    let request = IrqRequest::new(ipi_irq_handler)
                        .share_mode(ShareMode::Shared)
                        .scope(IrqScope::PerCpu { cpus })
                        .auto_enable(AutoEnable::Yes);
                    match request_irq(irq, request) {
                        Ok(_) => {
                            ax_println!("rt_shm: IPI IRQ handler registered (irq={})", ipi_irq)
                        }
                        Err(e) => ax_println!("rt_shm: failed to register IPI IRQ handler: {e:?}"),
                    }
                }
                #[cfg(not(target_arch = "riscv64"))]
                let _ = ipi_irq;

                IpiBackend::Clint(ClintIpiSender::new(msip_phys))
            }
            NotifierConfig::K3Mailbox {
                mbox_phys,
                tx_channel,
                rx_channel,
                rx_irq,
            } => {
                let page_aligned = mbox_phys & !0xFFF;
                let mmio = unsafe {
                    mmio_api::ioremap_raw(page_aligned.into(), 0x1000)
                        .expect("rt_shm: failed to ioremap K3 mailbox")
                };
                let offset = mbox_phys - page_aligned;
                // SAFETY: mmio is a valid ioremap'd region; offset < 0x1000.
                let mbox_base = unsafe { mmio.as_nonnull_ptr().as_ptr().add(offset) };

                // starryos = USER0, rcpu1 = USER1.
                let mbox_rx = K3MboxMmio::new(mbox_base, 0);
                let mbox_tx = K3MboxMmio::new(mbox_base, 0);
                // 先复位电平再使能（见 reset_rx 注释：注册前对端残留的 FIFO
                // 消息会让 NEW_MSG 电平挂死交付路径）。
                mbox_rx.reset_rx(rx_channel);
                mbox_rx.enable_rx(rx_channel);
                test_mbox = Some((K3MboxMmio::new(mbox_base, 0), rx_channel));

                #[cfg(target_arch = "riscv64")]
                {
                    use ax_runtime::hal::irq::{AutoEnable, IrqRequest, ShareMode, request_irq};
                    let mut endpoint = K3MailboxIrqEndpoint {
                        mbox: mbox_rx,
                        rx_channel,
                    };
                    let request = IrqRequest::new(move |_ctx| {
                        let now = K3_MBOX_IRQ_COUNT.fetch_add(1, Ordering::AcqRel) + 1;
                        // 前 5 次打计数便于确认链路（boot 自测与 RP 多时段单点
                        // 门铃；超出的不打印，防 notification 洪水刷屏）。
                        if now <= 5 {
                            ax_println!("rt_shm: mailbox IRQ fired (count={now})");
                        }
                        let event = endpoint.handle_irq();
                        if event == Event::PeerNotify {
                            wake_ipc_waiter();
                        }
                        ax_runtime::hal::irq::IrqReturn::Handled
                    })
                    .share_mode(ShareMode::Shared)
                    .auto_enable(AutoEnable::Yes);
                    match request_irq(rx_irq, request) {
                        Ok(_) => ax_println!(
                            "rt_shm: K3 mailbox IRQ handler registered (domain={}, irq={})",
                            rx_irq.domain.0,
                            rx_irq.hwirq.0
                        ),
                        Err(e) => ax_println!("rt_shm: failed to register K3 mailbox IRQ: {e:?}"),
                    }
                }
                #[cfg(not(target_arch = "riscv64"))]
                {
                    let _ = (rx_irq, mbox_rx);
                }

                IpiBackend::K3Mbox(K3MailboxIpiSender {
                    mbox: mbox_tx,
                    tx_channel,
                    user_remote: 1,
                })
            }
        };

        let core = RtShmCore::new(access, ipi);

        let valid = shm.is_valid();
        ax_println!(
            "rt_shm: device initialized, phys base {:#x}, size {} bytes",
            shm_phys_base,
            shm_size
        );
        ax_println!(
            "rt_shm: ov-channels SharedMemory<{}> valid={} (expected false at boot; becomes true \
             after rt-async inits)",
            DEFAULT_CHANNELS,
            valid
        );

        let dev = Self {
            core: SpinNoIrq::new(core),
            shm_phys_base,
            shm_size,
            lin_vaddr,
            mode,
            _shm_nc: shm_nc,
            test_mbox,
        };

        // Boot 时自动执行一次 AP 端 mailbox 自测：软件注入 new_msg 验证
        // mailbox → APLIC → IMSIC → CPU → handler 全链路贯通（无需 rt-async）。
        // 结果直接进启动日志，便于真板免传文件快速验证。
        if dev.test_mbox.is_some() {
            match dev.test_k3_mailbox_irq() {
                Ok(n) => ax_println!(
                    "rt_shm: K3 mailbox selftest OK: IRQ handler fired {n} time(s) \
                     (mailbox→APLIC→CPU→handler 链路贯通)"
                ),
                Err(e) => ax_println!(
                    "rt_shm: K3 mailbox selftest FAILED: {e:?} (中断未触发——检查 notifier \
                     中断线使能与 IMSIC 配置，注册行上方有实际 domain/irq 号)"
                ),
            }
        }
        dev
    }

    /// AP 端纯自测：向本地 user 注入一条 new_msg，轮询等待 IRQ handler 触发。
    ///
    /// 验证 mailbox → APLIC → IMSIC → CPU → handler 全链路贯通，无需对端
    /// （rt-async）参与。返回 handler 触发次数；超时（中断链路断）返回
    /// [`VfsError::Io`]。
    ///
    /// 仅 K3 mailbox 后端可用；其余平台（test_mbox=None）返回
    /// [`VfsError::Unsupported`]。
    fn test_k3_mailbox_irq(&self) -> VfsResult<usize> {
        let Some((mbox, rx_channel)) = &self.test_mbox else {
            return Err(VfsError::Unsupported);
        };
        let before = K3_MBOX_IRQ_COUNT.load(Ordering::Acquire);
        // 3 次尝试（boot 期 APLIC/IMSIC 交付可能延迟），每次注入前重新使能；
        // 全败时逐次打印 IRQSTATUS_RAW/IRQENABLE 原始值——RAW 置位而未触发
        // = APLIC/IMSIC 交付问题；RAW=0 = mailbox 单元未置位（EN/FIFO 语义）。
        for attempt in 1..=3u32 {
            mbox.enable_rx(*rx_channel);
            mbox.inject_local_message(*rx_channel);
            // 忙等上限 10^5 次自旋（GHz 级核上约百 µs~ms 量级）：链路健康时
            // 中断近乎立即触发；这是快速判定，不做精确超时。
            for _ in 0..100_000 {
                let now = K3_MBOX_IRQ_COUNT.load(Ordering::Acquire);
                if now != before {
                    return Ok(now - before);
                }
                core::hint::spin_loop();
            }
            let (raw, en) = mbox.debug_irq_raw();
            ax_println!(
                "rt_shm: mailbox selftest attempt {attempt} 未触发 (IRQSTATUS_RAW={raw:#010x}, \
                 IRQENABLE={en:#010x})"
            );
        }
        Err(VfsError::Io)
    }
}

// ── DeviceOps ──────────────────────────────────────────────────────────────

impl DeviceOps for RtShmDevice {
    fn read_at(&self, _buf: &mut [u8], _offset: u64) -> VfsResult<usize> {
        Err(VfsError::Unsupported)
    }

    fn write_at(&self, _buf: &[u8], _offset: u64) -> VfsResult<usize> {
        Err(VfsError::Unsupported)
    }

    fn ioctl(&self, cmd: u32, _arg: usize) -> VfsResult<usize> {
        match cmd {
            RT_SHM_IOC_NOTIFY => {
                // Cbo 模式：通知前把窗口 clean+invalidate（DMA-to-device 屏障），
                // 把本核缓存可能滞留的写推到 SRAM 再打门铃（详见 flush_shm_window
                // 注释），保证对端看到全部写入。PbmtNc 模式：NC/IO 写本就直达
                // SRAM，无滞留可推，跳过。
                if self.mode == CoherenceMode::Cbo {
                    flush_shm_window(self.lin_vaddr, self.shm_size);
                }
                self.core.lock().notify_peer();
                Ok(0)
            }
            RT_SHM_IOC_AWAIT => {
                use core::{future::poll_fn, task::Poll};

                use ax_task::future::{block_on, interruptible};
                let lin_vaddr = self.lin_vaddr;
                let cbo_mode = self.mode == CoherenceMode::Cbo;
                let _: Result<usize, VfsError> = block_on(interruptible(poll_fn(|cx| {
                    // 以下作废均为 Cbo 模式专属（PbmtNc 模式下 NC/IO 读直达
                    // SRAM，天然新鲜）。
                    //
                    // Cbo 模式背景：SRAM 的 PMA 为 cacheable，PBMT 不生效时
                    // ioremap 的"NC"别名读同样走缓存（板上实锤：RP 回包已落
                    // SRAM，IRQ 唤醒后重查 has_pending 仍命中首查取入的陈旧行，
                    // AWAIT 永久挂死，仅 60/120/180s ping 反复空唤醒）。
                    // 每次就绪检查前先作废窗口行，保证读到 SRAM 真值；poll
                    // 仅由 IRQ/伪唤醒驱动，频率低，CBO 开销可接受。
                    if cbo_mode {
                        invalidate_shm_window(lin_vaddr, self.shm_size);
                    }
                    if self.core.lock().has_pending() {
                        return Poll::Ready(Ok(0usize));
                    }
                    let mut guard = IPC_WAKER.lock();
                    // 重查前必须**再作废一次**（锁内，SpinNoIrq 挡住中断）：
                    // 首查读会把"空"快照取入缓存；若 RP 回包恰在首查与注册
                    // waker 之间落 SRAM、且其门铃唤醒先于此处拿到锁（waker
                    // 未注册，wake 为 no-op），锁内重查会命中首查的陈旧行而
                    // 误判"无数据"→ 注册 → 永久挂死。板上实锤：第 2 轮起 RP
                    // 暖态回程 µs 级，稳定踩中该窗口（wget 的 eth0 中断流把
                    // handler 推迟到注册之后，故 wget 后跑全过）。
                    if cbo_mode {
                        invalidate_shm_window(lin_vaddr, self.shm_size);
                    }
                    if self.core.lock().has_pending() {
                        Poll::Ready(Ok(0usize))
                    } else {
                        *guard = Some(cx.waker().clone());
                        Poll::Pending
                    }
                })))?;
                // Cbo 模式的 RP→AP 就绪同步点（for-cpu）：has_pending 经内核
                // NC 别名判定，返回即 SRAM 里已有回包。此处 clean+invalidate
                // 窗口——clean 把用户态滞留的脏行推出（不丢写），invalidate
                // 作废陈旧驻留行（含 fetch 早于对端写的行），保证随后用户态读
                // ch1 直接取 SRAM 真值。板上实锤：用户态映射实际 cacheable 时，
                // 无此同步点回包在 SRAM 存在 5s 仍对用户态不可读（仅 NOTIFY 的
                // flush 能让它显形）。协议安全性：本操作前后用户态不写共享窗
                // （send→NOTIFY→AWAIT→recv 纪律），无部分行写回覆盖对端风险。
                // PbmtNc 模式：用户态 NC 读直达 SRAM，跳过。
                if cbo_mode {
                    flush_shm_window(lin_vaddr, self.shm_size);
                }
                Ok(0)
            }
            RT_SHM_IOC_CLR_PENDING => Ok(0),
            RT_SHM_IOC_TEST_MBOX => self.test_k3_mailbox_irq(),
            _ => Err(VfsError::InvalidInput),
        }
    }

    fn mmap(&self, _offset: u64, _length: u64) -> DeviceMmap {
        // Cbo 模式：用户态映射前再作废一次全窗缓存行：清掉任何驻留副本
        // （内核早期 cacheable 访问或残留），保证随后用户态的 NC 写直达
        // SRAM 而不是被驻留行"吸收"（板上实锤：AP 回读自写成功而 RP 读不到）。
        // PbmtNc 模式：boot 期 invalidate 后窗口再无 cacheable 访问路径，
        // 无驻留行可清，跳过。
        if self.mode == CoherenceMode::Cbo {
            invalidate_shm_window(self.lin_vaddr, self.shm_size);
        }
        DeviceMmap::Physical(
            PhysAddrRange::from_start_size(PhysAddr::from(self.shm_phys_base), self.shm_size),
            None,
        )
    }

    fn as_any(&self) -> &dyn core::any::Any {
        self
    }

    fn as_pollable(&self) -> Option<&dyn Pollable> {
        Some(self)
    }

    fn flags(&self) -> NodeFlags {
        // 请求非缓存映射（PTE PBMT=NC）。PbmtNc 模式下这是用户态直达 SRAM
        // 的实际机制；Cbo 模式下该属性不生效（见模块文档「缓存一致性模型」），
        // 一致性由 CBO 同步点保证，此标志仅对属性生效的平台有意义。
        NodeFlags::NON_CACHEABLE
    }
}

// ── Pollable ───────────────────────────────────────────────────────────────

impl Pollable for RtShmDevice {
    fn poll(&self) -> IoEvents {
        let mut events = IoEvents::OUT;
        if self.core.lock().has_pending() {
            events |= IoEvents::IN;
        }
        events
    }
    fn register(&self, context: &mut core::task::Context<'_>, _events: IoEvents) {
        *IPC_WAKER.lock() = Some(context.waker().clone());
    }
}

//! Shared-memory device for AMP inter-core communication.
//!
//! Exposes the ov-channels shared memory region as `/dev/rt_shm`.
//!
//! OS Glue layer: implements StarryOS-specific capability traits
//! (CLINT MSIP for QEMU virt) for the reusable `rt-shm` Driver Core,
//! and parses all runtime configuration from the device tree.
//!
//! ## Device-tree driven configuration
//!
//! The device is only registered when the FDT contains an `ov,rt-async-amp`
//! compatible node. All runtime parameters — shared-memory region, IPI
//! notification mechanism, IPI IRQ — are read from the node so that
//! `amp.toml` is no longer the source of truth for `rt_shm`'s runtime
//! behavior (it remains for build-time/M-side constants).
//!
//! DT binding (`its/qemu-virt-amp.dts`):
//! ```text
//! rt-async@88000000 {
//!     compatible = "ov,rt-async-amp";
//!     reg = <0x00 0x88000000 0x00 0x11000>;   // SHM region
//!     notifier {
//!         compatible = "ov,clint-msip-notifier";
//!         reg = <0x00 0x2000004 0x00 0x4>;    // MSIP1 phys addr
//!         ov,ipi-irq = <0x1>;                  // SSI cause
//!     };
//! };
//! ```
//!
//! On platforms without the node, the device is not created.

use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use ax_errno::AxError;
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
        core::sync::atomic::fence(Ordering::Release);
        unsafe {
            core::ptr::write_volatile(self.msip_ptr, 1);
        }
    }

    #[cfg(not(target_arch = "riscv64"))]
    fn notify_peer(&mut self) {}
}

// ── StarryOS Glue: K3 Mailbox ──────────────────────────────────────────────

const K3_MBOX_MSG_BASE: usize = 0x040;
const K3_MBOX_IRQ_BASE: usize = 0x100;
const K3_MBOX_IRQ_STRIDE: usize = 0x10;
const K3_MBOX_IRQ_CLR_OFF: usize = 0x04;
const K3_MBOX_IRQ_EN_SET_OFF: usize = 0x08;

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
// entire system lifetime, accessed with volatile only. IpiSender path is
// guarded by SpinNoIrq on RtShmDevice; IRQ endpoint runs in non-reentrant
// callback.
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

    fn ack_and_clear(&self, channel: u8) {
        let _ = self.read(K3_MBOX_MSG_BASE + (channel as usize) * 4);
        self.write(
            k3_irq_reg(self.user_local, K3_MBOX_IRQ_CLR_OFF),
            k3_new_msg_mask(channel),
        );
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
}

struct K3MailboxIpiSender {
    mbox: K3MboxMmio,
    tx_channel: u8,
    user_remote: u8,
}

impl IpiSender for K3MailboxIpiSender {
    fn notify_peer(&mut self) {
        core::sync::atomic::fence(Ordering::Release);
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

static OPENED: AtomicBool = AtomicBool::new(false);
static IPC_WAKER: SpinNoIrq<Option<core::task::Waker>> = SpinNoIrq::new(None);
/// K3 mailbox new_msg 中断触发次数（自测用：ioctl TEST_MBOX 轮询此计数确认
/// mailbox → APLIC → CPU → handler 全链路贯通）。
static K3_MBOX_IRQ_COUNT: AtomicUsize = AtomicUsize::new(0);

// ── IPI handler (raw function pointer for irq-framework) ───────────────────

#[cfg(target_arch = "riscv64")]
fn ipi_irq_handler(_ctx: ax_runtime::hal::irq::IrqContext) -> ax_runtime::hal::irq::IrqReturn {
    if let Some(waker) = IPC_WAKER.lock().take() {
        waker.wake();
    }
    ax_runtime::hal::irq::IrqReturn::Handled
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
    /// K3 mailbox 自测句柄：仅 K3 分支构造时 Some，其余平台 None。
    /// 供 ioctl TEST_MBOX 软件注入 new_msg，验证中断全链路。
    test_mbox: Option<K3MboxMmio>,
}

impl RtShmDevice {
    pub fn new(cfg: RtAsyncConfig) -> Self {
        let RtAsyncConfig {
            shm_phys_base,
            shm_size,
            notifier,
        } = cfg;

        // SAFETY: Address from DT reserved SHM region. Layout matches M-mode
        // SharedMemory::<DEFAULT_CHANNELS> with the same feature configuration.
        let vaddr =
            ax_runtime::hal::mem::phys_to_virt(PhysAddr::from(shm_phys_base)).as_ptr() as usize;
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
                mbox_rx.enable_rx(rx_channel);
                test_mbox = Some(K3MboxMmio::new(mbox_base, 0));

                #[cfg(target_arch = "riscv64")]
                {
                    use ax_runtime::hal::irq::{AutoEnable, IrqRequest, ShareMode, request_irq};
                    let mut endpoint = K3MailboxIrqEndpoint {
                        mbox: mbox_rx,
                        rx_channel,
                    };
                    let request = IrqRequest::new(move |_ctx| {
                        K3_MBOX_IRQ_COUNT.fetch_add(1, Ordering::AcqRel);
                        let event = endpoint.handle_irq();
                        if event == Event::PeerNotify {
                            if let Some(waker) = IPC_WAKER.lock().take() {
                                waker.wake();
                            }
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
                    "rt_shm: K3 mailbox selftest FAILED: {e:?} \
                     (中断未触发——检查 APLIC source 217 使能与 IMSIC 配置)"
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
        let Some(mbox) = &self.test_mbox else {
            return Err(VfsError::Unsupported);
        };
        let before = K3_MBOX_IRQ_COUNT.load(Ordering::Acquire);
        mbox.inject_local_message(CH_FROM_RT_ASYNC.get());
        // 中断应在微秒量级到达；轮询 1ms 仍无触发则判链路故障。
        for _ in 0..1000 {
            let now = K3_MBOX_IRQ_COUNT.load(Ordering::Acquire);
            if now != before {
                return Ok(now - before);
            }
            core::hint::spin_loop();
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
                self.core.lock().notify_peer();
                Ok(0)
            }
            RT_SHM_IOC_AWAIT => {
                use core::{future::poll_fn, task::Poll};

                use ax_task::future::{block_on, interruptible};
                block_on(interruptible(poll_fn(|cx| {
                    if self.core.lock().has_pending() {
                        return Poll::Ready(Ok(0usize));
                    }
                    let mut guard = IPC_WAKER.lock();
                    if self.core.lock().has_pending() {
                        Poll::Ready(Ok(0usize))
                    } else {
                        *guard = Some(cx.waker().clone());
                        Poll::Pending
                    }
                })))?
            }
            RT_SHM_IOC_CLR_PENDING => Ok(0),
            RT_SHM_IOC_TEST_MBOX => self.test_k3_mailbox_irq(),
            _ => Err(VfsError::InvalidInput),
        }
    }

    fn mmap(&self, _offset: u64, _length: u64) -> DeviceMmap {
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

// ── Lifecycle ──────────────────────────────────────────────────────────────

pub fn try_claim_device() -> bool {
    OPENED
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_ok()
}

pub fn release_device() {
    OPENED.store(false, Ordering::Release);
}

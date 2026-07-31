//! Reusable AMP shared-memory IPC driver core.
//!
//! This crate provides an OS-independent abstraction for inter-core
//! communication via shared memory + inter-processor interrupts (IPI).
//! It implements the *Driver Core* layer of the
//! [cross-kernel-driver][ckd] architecture:
//!
//! * **Driver Core** (this crate): pure pending-check logic, no OS deps.
//!   SHM access is built on [`ov_channels`] so the ring-buffer pointer
//!   offsets are determined by the `SharedMemory<N>` type itself — no
//!   hardcoded offsets (the previous `0x8400`/`0x8408` magic is gone).
//! * **Capability Boundary** (traits): [`IpiSender`] (notify the peer),
//!   [`ShmAccess`] (check pending), [`IpiIrqEndpoint`] (IRQ source probe).
//! * **OS Glue** (platform code): implements `IpiSender` for a specific
//!   platform (e.g. CLINT MSIP for QEMU virt, hardware Mailbox for K3),
//!   registers IRQ handlers, wires up VFS devices or task wakers.
//!
//! [ckd]: https://rcore-os.cn/tgoskits/docs/development/components#cross-kernel-driver
//!
//! # Portability
//!
//! The core logic is identical on any platform. To port to a new
//! IPI mechanism (e.g. CLINT → Mailbox), write a new `impl IpiSender`
//! — the [`RtShmCore`] struct needs no changes. SHM access is provided
//! generically by [`OvChannelsShmAccess`] for any `SharedMemory<N>`.
//!
//! # Example (QEMU virt, CLINT MSIP)
//!
//! See the StarryOS OS Glue in `os/StarryOS/kernel/src/pseudofs/dev/rt_shm.rs`.

#![no_std]

// ── Re-exports from ov-channels ──────────────────────────────────────────
//
// OS Glue 通过这些重导出访问 ov-channels 类型，避免 kernel 直接依赖
// ov-channels crate（rt_shm 的独有依赖应当封装在本 Driver Core 内）。
pub use ov_channels::{ChannelId, SharedMemory};

/// 默认通道数（双端协议 ABI 常量）。
///
/// M-mode 侧 (rt-async) 使用 `SharedMemory::<3>`，故 S-mode 侧必须一致。
/// 通道布局约定：
/// - CH0: StarryOS → rt-async
/// - CH1: rt-async → StarryOS
/// - CH2: 备用
pub const DEFAULT_CHANNELS: usize = 3;

/// Event returned by the IRQ endpoint after inspecting the interrupt source.
///
/// Per the cross-kernel-driver principle "interrupts synchronize state,
/// tasks advance flow" (`中断只同步状态，任务才推进流程`), the IRQ handler
/// should only extract an event — the OS Glue decides whether to wake a
/// thread, a future, schedule a worker, or set a pending flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// The peer core has sent us a notification. New messages may be
    /// available in the ring buffer.
    PeerNotify,
    /// The interrupt was not from our peer (spurious / other source).
    Spurious,
}

// ── Capability traits ────────────────────────────────────────────────────

/// Ability to send an inter-core notification (IPI) to the peer core.
///
/// The implementation is platform-specific:
/// * QEMU virt: write to CLINT MSIP1 register
/// * K3: trigger a hardware Mailbox channel
///
/// # Contract
///
/// Before `notify_peer` returns, the implementation MUST ensure that all
/// prior writes to shared memory are visible to the peer (typically via
/// a `Release` fence before the register write).
pub trait IpiSender {
    /// Trigger a notification to the peer core.
    fn notify_peer(&mut self);
}

/// Check whether the peer has written new data into the shared-memory
/// channel since we last read it.
///
/// This replaces the old `read_ptr`/`write_ptr` split: ring-buffer
/// pointer offsets are now an internal detail of [`OvChannelsShmAccess`],
/// so OS Glue no longer needs to hand-compute them.
pub trait ShmAccess {
    /// Returns `true` when the peer has written unread messages.
    ///
    /// Implementations read the channel's read/write pointers with
    /// `Acquire` ordering to synchronize with the peer's `Release` writes.
    fn has_pending(&self) -> bool;
}

/// IRQ endpoint owned by the OS IRQ handler callback.
///
/// Per the cross-kernel-driver "IRQ Callback Ownership" pattern, the
/// IRQ endpoint should be **moved into** the registered interrupt
/// callback (e.g. via a `Box<dyn FnMut>` closure), not shared through
/// `Arc<Mutex<_>>`. This gives the handler a single mutation site
/// without locking.
///
/// The endpoint only inspects the interrupt source and returns an
/// [`Event`]; it does **not** directly wake tasks, schedule workers,
/// or call OS notification APIs.
pub trait IpiIrqEndpoint {
    /// Inspect the interrupt source and return an event.
    ///
    /// Called in IRQ context (non-reentrant, short path, no blocking).
    fn handle_irq(&mut self) -> Event;
}

// ── Driver Core ──────────────────────────────────────────────────────────

/// OS-independent shared-memory IPC device state.
///
/// [`RtShmCore`] combines a [`ShmAccess`] implementation (for checking
/// pending messages) and an [`IpiSender`] (for notifying the peer).
/// It provides the core logic that is identical across all platforms.
///
/// # Generic parameters
///
/// * `A`: [`ShmAccess`] — how to check the shared-memory channel
/// * `S`: [`IpiSender`] — how to send notifications to the peer
pub struct RtShmCore<A: ShmAccess, S: IpiSender> {
    shm: A,
    ipi: S,
}

impl<A: ShmAccess, S: IpiSender> RtShmCore<A, S> {
    /// Create a new IPC device core.
    pub fn new(shm: A, ipi: S) -> Self {
        Self { shm, ipi }
    }

    /// Check whether the peer has written new data since we last read.
    pub fn has_pending(&self) -> bool {
        self.shm.has_pending()
    }

    /// Notify the peer core that we have written new data.
    ///
    /// The caller should have already written data to shared memory
    /// before calling this. The [`IpiSender`] implementation is
    /// responsible for the memory fence that makes those writes
    /// visible before the IPI is delivered.
    pub fn notify_peer(&mut self) {
        #[cfg(feature = "log")]
        log::trace!("rt_shm: notifying peer");
        self.ipi.notify_peer();
        #[cfg(feature = "log")]
        log::trace!("rt_shm: peer notified");
    }
}

// ── Generic ov-channels ShmAccess implementation ─────────────────────────

/// 基于 [`ov_channels::SharedMemory`] 的 [`ShmAccess`] 实现。
///
/// `N` 必须与对端一致（双端协议常量，见 [`DEFAULT_CHANNELS`]）。
/// OS Glue 负责把物理地址映射后，通过
/// [`SharedMemory::<N>::at(vaddr)`](ov_channels::SharedMemory::at) 得到
/// `&'static` 引用再传入本结构体。这样 ring-buffer 指针偏移完全由
/// ov-channels 的类型布局决定，不再需要在驱动里硬编码偏移常量。
pub struct OvChannelsShmAccess<const N: usize> {
    shm: &'static SharedMemory<N>,
    channel: ChannelId,
}

impl<const N: usize> OvChannelsShmAccess<N> {
    /// # Safety
    ///
    /// `shm` 必须指向一块有效、已映射、且与对端 ov-channels 布局一致
    /// （相同的 `N` 与 feature 配置，尤其是 `flags`）的共享内存区域。
    /// 引用必须在设备整个生命周期内保持有效。
    pub unsafe fn new(shm: &'static SharedMemory<N>, channel: ChannelId) -> Self {
        Self { shm, channel }
    }
}

impl<const N: usize> ShmAccess for OvChannelsShmAccess<N> {
    fn has_pending(&self) -> bool {
        // 只做只读的 pending 检查：receiver() 返回的 Receiver 仅读取
        // ring buffer 的 read/write 指针（Acquire），不修改任何状态。
        self.shm
            .receiver(self.channel)
            .map(|r| r.has_pending())
            .unwrap_or(false)
    }
}

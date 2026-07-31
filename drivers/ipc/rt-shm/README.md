# rt-shm — Reusable AMP Shared-Memory IPC Driver Core

`no_std`, OS-independent driver core for inter-core communication via
shared memory + inter-processor interrupts (IPI).

## Architecture

Follows the [cross-kernel-driver](https://rcore-os.cn/tgoskits/docs/development/components#cross-kernel-driver) layered architecture:

```
┌──────────────────────────────────────────────┐
│  Runtime (blocking / Future / worker)         │  ← OS-specific
├──────────────────────────────────────────────┤
│  OS Glue (VFS / IRQ registration / mmap)     │  ← per-kernel impl
├──────────────────────────────────────────────┤
│  Capability Boundary (traits in this crate)  │  ← IpiSender / ShmAccess
├──────────────────────────────────────────────┤
│  Driver Core (RtShmCore<A, S>)               │  ← this crate, all platforms
└──────────────────────────────────────────────┘
```

### Driver Core

[`RtShmCore<A, S>`] — pure ring-buffer logic, identical on any platform:

- `has_pending()` — check if the peer has written new data
- `notify_peer()` — signal the peer via the platform's IPI mechanism

### Capability Traits

Three traits abstract hardware-specific details:

| Trait | Purpose | QEMU virt impl | K3 impl |
|---|---|---|---|
| [`IpiSender`] | Send notification to peer | CLINT MSIP1 write | Mailbox trigger |
| [`ShmAccess`] | Access ring-buffer pointers in shared memory | `phys_to_virt` + offset | same |
| [`IpiIrqEndpoint`] | Extract event from IRQ source | SSI (cause 1) | External IRQ |

### IRQ Design Principle

> **Interrupts synchronize state; tasks advance flow.**
> (`中断只同步状态，任务才推进流程`)

The IRQ endpoint only extracts an [`Event`] — the OS Glue decides whether
to wake a thread, a future, schedule a worker, or set a pending flag.

## Usage

### Adding the reusable core to your OS

1. Add `rt-shm` to your crate's dependencies
2. Implement [`ShmAccess`] for your platform (map shared memory, return ring pointer references)
3. Implement [`IpiSender`] for your platform (write to IPI register)
4. Implement [`IpiIrqEndpoint`] for your platform (detect interrupt source in IRQ context)
5. Create an `RtShmCore<YourShmAccess, YourIpiSender>` instance
6. In OS Glue, wire up VFS device ops, IRQ registration, and task wake logic

### Example: StarryOS on QEMU virt (CLINT MSIP)

See `os/StarryOS/kernel/src/pseudofs/dev/rt_shm.rs` in the tgoskits
repository for the complete StarryOS OS Glue implementation:

- **`StarryShmAccess`**: wraps `phys_to_virt(SHMBASE)`, returns `&AtomicUsize` at CH1 ring buffer offsets
- **`ClintIpiSender`**: writes `1` to `CLINT_BASE + 4` (MSIP1) with a `Release` fence
- **SSI handler**: registers a percpu IRQ handler on `HwIrq(1)` that wakes the IPC_WAKER
- **VFS**: implements `DeviceOps` + `Pollable`, exposed as `/dev/rt_shm` with NOTIFY/AWAIT ioctls

### Porting to K3 (hardware Mailbox)

To port to the K3 platform, replace the QEMU-specific traits:

```rust
// K3 Mailbox IPI sender — triggers a hardware Mailbox channel
struct K3MailboxIpiSender { mbox: MailboxChannel }
impl IpiSender for K3MailboxIpiSender {
    fn notify_peer(&mut self) {
        core::sync::atomic::fence(Ordering::Release);
        self.mbox.trigger_notification();
    }
}

// K3 Mailbox IRQ endpoint — external interrupt, needs explicit ACK
struct K3MailboxIrq { mbox: MailboxChannel }
impl IpiIrqEndpoint for K3MailboxIrq {
    fn handle_irq(&mut self) -> Event {
        self.mbox.ack_interrupt();
        Event::PeerNotify
    }
}
```

The `RtShmCore` struct and all ring-buffer logic remain **unchanged**.

## License

Apache-2.0 (see the tgoskits root LICENSE).

## See Also

- [cross-kernel-driver skill](https://rcore-os.cn/tgoskits/docs/development/components)
- [ov-channels](https://crates.io/crates/ov-channels) — the ring-buffer library used by rt-async-amp
- StarryOS OS Glue: `os/StarryOS/kernel/src/pseudofs/dev/rt_shm.rs`

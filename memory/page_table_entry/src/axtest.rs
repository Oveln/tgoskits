use ax_memory_addr::PhysAddr;
use axtest::prelude::*;

#[cfg(target_arch = "x86_64")]
use crate::x86_64::{PTF, X64PTE};
#[cfg(all(target_arch = "riscv64", feature = "svpbmt"))]
use crate::riscv::Rv64PTE;
use crate::{GenericPTE, MappingFlags};

#[axtest]
fn page_table_entry_mapping_flags_debug_and_bit_rules_hold() {
    let flags = MappingFlags::READ | MappingFlags::WRITE | MappingFlags::USER;
    ax_assert!(flags.contains(MappingFlags::READ));
    ax_assert!(flags.contains(MappingFlags::WRITE));
    ax_assert!(flags.contains(MappingFlags::USER));
    ax_assert!(!flags.contains(MappingFlags::EXECUTE));
    let debug = alloc::format!("{flags:?}");
    ax_assert!(debug.contains("READ"));
    ax_assert!(debug.contains("WRITE"));
    ax_assert!(debug.contains("USER"));
    ax_assert_eq!(flags.bits(), 0b1011);
}

#[cfg(all(target_arch = "riscv64", feature = "svpbmt"))]
#[axtest]
fn page_table_entry_riscv_svpbmt_uncached_encodes_pbmt_nc() {
    // 回归测试：K3 共享内存 mmap 依赖 Svpbmt PBMT 字段编码 UNCACHED。
    // 未启用 svpbmt feature 时 UNCACHED 不编码（PTE 保持 cacheable），
    // 跨核写会滞留在本核 cache 不达物理内存。
    let pte = Rv64PTE::new_page(
        PhysAddr::from(0x1234_5000),
        MappingFlags::READ | MappingFlags::WRITE | MappingFlags::UNCACHED,
        false,
    );
    // PBMT 字段 = PTE bits 62:61，NC=01。
    let pbmt = (pte.bits() >> 61) & 0b11;
    ax_assert_eq!(pbmt, 0b01, "UNCACHED 应编码为 PBMT=01 (NC)");
    // 读回路径：PBMT=NC → UNCACHED。
    let flags = pte.flags();
    ax_assert!(flags.contains(MappingFlags::UNCACHED));
    ax_assert!(!flags.contains(MappingFlags::DEVICE));
    // 物理地址与基础权限不受影响。
    ax_assert_eq!(pte.paddr(), PhysAddr::from(0x1234_5000));
    ax_assert!(flags.contains(MappingFlags::READ));
    ax_assert!(flags.contains(MappingFlags::WRITE));
}

#[cfg(all(target_arch = "riscv64", feature = "svpbmt"))]
#[axtest]
fn page_table_entry_riscv_svpbmt_device_encodes_pbmt_io() {
    let pte = Rv64PTE::new_page(
        PhysAddr::from(0x1234_6000),
        MappingFlags::READ | MappingFlags::WRITE | MappingFlags::DEVICE,
        false,
    );
    // PBMT 字段 = PTE bits 62:61，IO=10。
    let pbmt = (pte.bits() >> 61) & 0b11;
    ax_assert_eq!(pbmt, 0b10, "DEVICE 应编码为 PBMT=10 (IO)");
    // 读回路径：PBMT=IO → DEVICE。
    let flags = pte.flags();
    ax_assert!(flags.contains(MappingFlags::DEVICE));
    ax_assert!(!flags.contains(MappingFlags::UNCACHED));
    ax_assert_eq!(pte.paddr(), PhysAddr::from(0x1234_6000));
}

#[cfg(all(target_arch = "riscv64", feature = "svpbmt"))]
#[axtest]
fn page_table_entry_riscv_svpbmt_absent_keeps_pbmt_pma() {
    // 无 UNCACHED/DEVICE 时 PBMT 保持 00（PMA 默认，cacheable）。
    let pte = Rv64PTE::new_page(
        PhysAddr::from(0x1234_7000),
        MappingFlags::READ | MappingFlags::WRITE,
        false,
    );
    let pbmt = (pte.bits() >> 61) & 0b11;
    ax_assert_eq!(pbmt, 0b00, "普通映射 PBMT 应为 00 (PMA)");
    let flags = pte.flags();
    ax_assert!(!flags.contains(MappingFlags::UNCACHED));
    ax_assert!(!flags.contains(MappingFlags::DEVICE));
}

#[cfg(target_arch = "x86_64")]
#[axtest]
fn page_table_entry_x64_flags_roundtrip_and_empty_rules_hold() {
    ax_assert_eq!(MappingFlags::from(PTF::empty()), MappingFlags::empty());

    let flags = MappingFlags::READ | MappingFlags::WRITE | MappingFlags::USER;
    let pt_flags = PTF::from(flags);
    ax_assert!(pt_flags.contains(PTF::PRESENT));
    ax_assert!(pt_flags.contains(PTF::WRITABLE));
    ax_assert!(pt_flags.contains(PTF::USER_ACCESSIBLE));
    ax_assert!(pt_flags.contains(PTF::NO_EXECUTE));
    ax_assert_eq!(
        MappingFlags::from(pt_flags),
        MappingFlags::READ | MappingFlags::WRITE | MappingFlags::USER
    );

    let executable = PTF::from(MappingFlags::READ | MappingFlags::EXECUTE);
    ax_assert!(!executable.contains(PTF::NO_EXECUTE));
    let device = PTF::from(MappingFlags::READ | MappingFlags::DEVICE);
    ax_assert!(device.contains(PTF::NO_CACHE));
    ax_assert!(device.contains(PTF::WRITE_THROUGH));
}

#[cfg(target_arch = "x86_64")]
#[axtest]
fn page_table_entry_x64_pte_lifecycle_rules_hold() {
    let mut entry = X64PTE::empty();
    ax_assert!(entry.is_unused());
    ax_assert!(!entry.is_present());
    ax_assert!(!entry.is_huge());

    entry = X64PTE::new_page(
        PhysAddr::from(0x1234_5000),
        MappingFlags::READ | MappingFlags::WRITE,
        true,
    );
    ax_assert!(entry.is_present());
    ax_assert!(entry.is_huge());
    ax_assert_eq!(entry.paddr(), PhysAddr::from(0x1234_5000));
    ax_assert!(entry.flags().contains(MappingFlags::READ));
    ax_assert!(entry.flags().contains(MappingFlags::WRITE));

    entry.set_paddr(PhysAddr::from(0x2000_0000));
    entry.set_flags(
        MappingFlags::READ | MappingFlags::EXECUTE | MappingFlags::USER,
        false,
    );
    ax_assert_eq!(entry.paddr(), PhysAddr::from(0x2000_0000));
    ax_assert!(!entry.is_huge());
    ax_assert!(entry.flags().contains(MappingFlags::EXECUTE));
    ax_assert!(entry.flags().contains(MappingFlags::USER));
    ax_assert!(entry.bits() != 0);
    ax_assert!(alloc::format!("{entry:?}").contains("X64PTE"));

    let table = X64PTE::new_table(PhysAddr::from(0x3000_0000));
    ax_assert!(table.is_present());
    ax_assert!(!table.is_huge());
    ax_assert_eq!(table.paddr(), PhysAddr::from(0x3000_0000));

    entry.clear();
    ax_assert!(entry.is_unused());
}

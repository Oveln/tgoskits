use crate::{
    common::PlatOp,
    irq::{CPU_LOCAL_IRQ_DOMAIN, HwIrq, IrqError, IrqId, IrqSource},
};

mod imsic_aplic;
mod plic;

use crate::irq_routing::{
    RISCV_INTERRUPT_BIT, RISCV_S_SOFT_CAUSE, RiscvTrapIrq, classify_riscv_trap,
    riscv_cpu_local_hwirq_is_runtime_irq, riscv_cpu_local_irq_from_raw, riscv_local_irq_raw,
    riscv_resolve_controller_line,
};

pub struct Plat;

/// 放行 U 态 Zicbom 缓存维护指令（cbo.clean/flush/inval），逐 hart 一次性配置。
///
/// senvcfg（Senvcfg 扩展）按 U/VU 模式门控 CBO 指令：未置位时 U 态执行
/// 即 illegal instruction。位域（RISC-V 特权规范，与 Linux `ENVCFG_*` 同值）：
/// * CBCFE（bit 6）：放行 cbo.clean / cbo.flush；
/// * CBIE（bits[5:4]）= 01：cbo.inval 按 flush 语义执行（写回脏行再作废），
///   杜绝 U 态误用 inval 丢弃脏写——本仓用户态缓存助手（ov-rpc `user-cbo`）
///   依赖该安全性把 refresh 实现为 inval。
///
/// 由 `zicbom` feature 门控：board TOML 只为确有 Zicbom 的板（K3/X100）
/// 开启，QEMU 等平台不编译本代码（无 Senvcfg CSR 的核上 csrs 会陷入）。
/// X100 S 态已实测执行 cbo（rt_shm 缓存同步点），envcfg 实现随 Zicbom 提供。
#[cfg(feature = "zicbom")]
fn enable_user_cbo(cpu_idx: usize) {
    const SENVCFG_CBCFE: usize = 1 << 6;
    const SENVCFG_CBIE_FLUSH_SEMANTICS: usize = 1 << 4;
    unsafe {
        core::arch::asm!(
            "csrs senvcfg, {bits}",
            bits = in(reg) SENVCFG_CBCFE | SENVCFG_CBIE_FLUSH_SEMANTICS,
            options(nostack, preserves_flags)
        );
    }
    log::info!("somehal: user-mode CBO enabled on cpu{cpu_idx} (senvcfg CBCFE|CBIE=flush)");
}

fn plic_irq_id_from_claimed_source(source: usize) -> Result<IrqId, IrqError> {
    let domain = crate::irq::domain_by_kind_fast(crate::irq::IrqDomainKind::RiscvPlic)
        .ok_or(IrqError::Unsupported)?;
    let source = u32::try_from(source).map_err(|_| IrqError::InvalidIrq)?;
    if source == 0 {
        return Err(IrqError::InvalidIrq);
    }
    Ok(IrqId::new(domain, HwIrq(source)))
}

fn checked_cpu_local_irq(hwirq: HwIrq) -> Result<IrqId, IrqError> {
    if riscv_cpu_local_hwirq_is_runtime_irq(hwirq) {
        Ok(IrqId::new(CPU_LOCAL_IRQ_DOMAIN, hwirq))
    } else {
        Err(IrqError::InvalidIrq)
    }
}

impl PlatOp for Plat {
    type ActiveIrq = plic::ActiveIrq;

    fn irq_set_enable(irq: IrqId, enable: bool) -> Result<(), IrqError> {
        if irq.domain == CPU_LOCAL_IRQ_DOMAIN {
            return plic::local_irq_set_enable(riscv_local_irq_raw(irq)?.into(), enable);
        }
        if crate::irq::domain_is_kind(irq.domain, crate::irq::IrqDomainKind::RiscvPlic)
            || crate::irq::domain_is_kind(irq.domain, crate::irq::IrqDomainKind::RiscvAplic)
            || crate::irq::domain_is_kind(irq.domain, crate::irq::IrqDomainKind::RiscvImsic)
        {
            return crate::irq::set_controller_irq_enabled(irq, enable);
        }
        Err(IrqError::InvalidIrq)
    }

    fn irq_set_affinity(irq: IrqId, affinity: crate::irq::IrqAffinity) -> Result<(), IrqError> {
        if irq.domain == CPU_LOCAL_IRQ_DOMAIN {
            return Err(IrqError::Unsupported);
        }
        if crate::irq::domain_is_kind(irq.domain, crate::irq::IrqDomainKind::RiscvPlic) {
            return plic::irq_set_affinity(irq.hwirq, affinity);
        }
        // AIA（APLIC wired / IMSIC MSI）的 affinity 即 MSI target hart。source 在
        // set_enabled 时经 configure_source 已绑定到 current hart；当前单 hart 场景
        // 下 set_affinity 无需操作。多 hart 时需要 reconfigure APLIC source 的
        // hart_index（重新 configure_source 切换 MSI 目标 hart），届时在此实现。
        // TODO: 多 hart APLIC source re-target。
        if crate::irq::domain_is_kind(irq.domain, crate::irq::IrqDomainKind::RiscvAplic)
            || crate::irq::domain_is_kind(irq.domain, crate::irq::IrqDomainKind::RiscvImsic)
        {
            return Ok(());
        }
        Err(IrqError::InvalidIrq)
    }

    fn begin_irq(raw: usize) -> Option<Self::ActiveIrq> {
        match classify_riscv_trap(raw) {
            RiscvTrapIrq::External if imsic_aplic::is_aia_active() => {
                imsic_aplic::begin_external_irq()
            }
            _ => plic::begin_irq(raw),
        }
    }

    fn active_irq_id(active: &Self::ActiveIrq) -> IrqId {
        let raw: usize = active.id().into();
        if raw & RISCV_INTERRUPT_BIT != 0 {
            riscv_cpu_local_irq_from_raw(raw).expect("active RISC-V local IRQ must be validated")
        } else if imsic_aplic::is_aia_active() {
            // stopei returns an EID. Always attribute it to the IMSIC root
            // domain; resolve_irq_route walks parent→leaf to find the correct
            // child-domain handler (APLIC wired or PCI MSI-X).
            let imsic = crate::irq::domain_by_kind_fast(crate::irq::IrqDomainKind::RiscvImsic)
                .expect("AIA active but IMSIC domain missing");
            IrqId::new(imsic, HwIrq(raw as u32))
        } else {
            plic_irq_id_from_claimed_source(raw)
                .expect("active RISC-V PLIC source must come from a validated claim")
        }
    }

    fn systick_irq() -> IrqId {
        riscv_cpu_local_irq_from_raw(plic::systick_irq().into())
            .expect("RISC-V systick IRQ must be a CPU-local timer cause")
    }

    fn resolve_irq_source(source: IrqSource) -> Result<IrqId, IrqError> {
        riscv_resolve_controller_line(source, || {
            matches!(
                source,
                IrqSource::ControllerLine { domain, .. }
                    if crate::irq::domain_is_kind(domain, crate::irq::IrqDomainKind::RiscvPlic)
                        || crate::irq::domain_is_kind(domain, crate::irq::IrqDomainKind::RiscvAplic)
            )
        })?;
        match source {
            IrqSource::ControllerLine { domain, hwirq } if domain == CPU_LOCAL_IRQ_DOMAIN => {
                checked_cpu_local_irq(hwirq)
            }
            IrqSource::ControllerLine { domain, hwirq }
                if crate::irq::domain_is_kind(domain, crate::irq::IrqDomainKind::RiscvAplic) =>
            {
                Ok(IrqId::new(domain, hwirq))
            }
            IrqSource::ControllerLine { domain, hwirq } => {
                plic::source_from_hwirq(hwirq)?;
                Ok(IrqId::new(domain, hwirq))
            }
            IrqSource::AcpiGsi(_) | IrqSource::AcpiGsiRoute(_) => unreachable!(),
        }
    }

    fn secondary_init() {}

    fn init_boot_irq_cpu(cpu_idx: usize, role: crate::irq::CpuBootRole) {
        // 每个 hart 恰好经过本函数一次（主核经 init_boot_irqs、副核经
        // init_secondary_boot_irqs），是逐 hart CSR 配置的单点。
        #[cfg(feature = "zicbom")]
        enable_user_cbo(cpu_idx);
        match role {
            crate::irq::CpuBootRole::Primary => {}
            crate::irq::CpuBootRole::Secondary => {
                if imsic_aplic::is_aia_active() {
                    imsic_aplic::secondary_init_intc();
                } else {
                    plic::secondary_init_intc(cpu_idx);
                }
            }
        }
    }

    fn send_ipi(irq: IrqId, target: crate::irq::IpiTarget) {
        if irq != Self::ipi_irq() {
            warn!("refuse to send non-runtime RISC-V IPI IRQ {irq:?}");
            return;
        }
        match target {
            crate::irq::IpiTarget::Current { cpu_id } | crate::irq::IpiTarget::Other { cpu_id } => {
                plic::send_ipi_to_cpu(cpu_id);
            }
            crate::irq::IpiTarget::AllExceptCurrent { cpu_id, cpu_num } => {
                for target_cpu in 0..cpu_num {
                    if target_cpu != cpu_id {
                        plic::send_ipi_to_cpu(target_cpu);
                    }
                }
            }
        }
    }

    fn ipi_irq() -> IrqId {
        IrqId::new(CPU_LOCAL_IRQ_DOMAIN, HwIrq(RISCV_S_SOFT_CAUSE as u32))
    }

    fn send_ipi_to_cpu(cpu_id: usize) {
        plic::send_ipi_to_cpu(cpu_id);
    }
}

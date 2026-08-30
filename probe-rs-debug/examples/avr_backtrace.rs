//! Scratch AVR backtrace check for hardware bring-up. Not for commit.

use std::time::Duration;

use probe_rs::Permissions;
use probe_rs::probe::list::Lister;
use probe_rs_debug::{DebugInfo, DebugRegisters, StackFrameInfo, exception_handler_for_core};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // In level4, after its buffer is filled.
    const BP: u64 = 0x356;

    let elf = "scratch/avr/da64/target/avr-none/debug/da64.elf";
    let debug_info = DebugInfo::from_file(elf)?;

    let lister = Lister::new();
    let list = lister.list_all();
    let probe = list.first().expect("no probe").open()?;

    let mut session = probe.attach("AVR128DA64", Permissions::default())?;
    let mut core = session.core(0)?;

    core.reset_and_halt(Duration::from_millis(500))?;
    core.set_hw_breakpoint(BP)?;
    core.run()?;
    std::thread::sleep(Duration::from_millis(300));
    println!("status = {:?}", core.status()?);

    let registers = DebugRegisters::from_core(&mut core);
    let handler = exception_handler_for_core(core.core_type());
    let instruction_set = core.instruction_set().ok();

    let mut frames =
        debug_info.unwind(&mut core, registers, handler.as_ref(), instruction_set, 20)?;

    for frame in frames.iter_mut() {
        let info = StackFrameInfo {
            registers: &frame.registers,
            frame_base: frame.frame_base,
            canonical_frame_address: frame.canonical_frame_address,
            scanned: frame.scanned,
        };
        if let Some(cache) = frame.local_variables.as_mut() {
            cache.recurse_deferred_variables(&debug_info, &mut core, 10, info);
        }
    }

    println!("--- backtrace, {} frames ---", frames.len());
    for (i, f) in frames.iter().enumerate() {
        let loc = f
            .source_location
            .as_ref()
            .map(|l| format!("{}:{}", l.path.to_string_lossy(), l.line.unwrap_or(0)))
            .unwrap_or_default();
        println!("#{i}  {}  {}  {loc}", f.pc, f.function_name);
    }

    for f in frames.iter().take(6) {
        println!("--- locals in {} ---", f.function_name);
        let Some(cache) = f.local_variables.as_ref() else {
            println!("  (none)");
            continue;
        };
        let root = cache.root_variable();
        for v in cache.get_children(root.variable_key()) {
            println!(
                "  {:<12} {:<10} = {}",
                v.name,
                v.type_name(),
                v.to_string(cache)
            );
        }
    }

    core.clear_hw_breakpoint(BP).ok();
    Ok(())
}

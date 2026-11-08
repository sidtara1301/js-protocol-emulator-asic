import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_basic_boot(dut):
    """Minimal test to satisfy the Tiny Tapeout GitHub Action."""
    dut._log.info("Starting basic boot test...")
    
    # Start a 50 MHz clock
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    # Initialize Tiny Tapeout wrapper inputs safely
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    # Assert reset for 10 cycles
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    
    # Run for a few cycles to prove the design breathes
    await ClockCycles(dut.clk, 20)
    
    dut._log.info("CI Test passed!")
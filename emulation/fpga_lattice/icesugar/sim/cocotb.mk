# cocotb co-simulation: FPGA build + Pico PSRAM emulator (PIO model).  Linux / WSL:
#   pip install cocotb==2.1.0 numpy scipy ; sudo apt install iverilog gcc
#   make -f cocotb.mk KWS_REPO=/path/to/tiny_kws            (default: ../../../../tapeouts/TTsky26e/tiny_kws)
# Regenerate qspi_psram.pio.h after editing the .pio:  pioasm ../../pico_psram_host/src/qspi_psram.pio qspi_psram.pio.h
SIM ?= icarus
TOPLEVEL_LANG = verilog
RTL = ../rtl
ifeq ($(GATE),)
VERILOG_SOURCES = $(RTL)/kws_sreg.v $(RTL)/kws_spi_slave.v $(RTL)/kws_coef_rom.v $(RTL)/kws_frontend.v \
                  $(RTL)/kws_qspi.v $(RTL)/kws_nn.v $(RTL)/project.v ../top_icesugar.v tb_fpga.v
COMPILE_ARGS += -DSIM
else
# gate level: GATE=<yosys netlist of top_icesugar synthesised with -DSIM>, CELLS=<yosys share/ice40/cells_sim.v>
VERILOG_SOURCES = $(GATE) $(CELLS) tb_fpga.v
COMPILE_ARGS += -DGATE_SIM -DNO_ICE40_DEFAULT_ASSIGNMENTS
endif
TOPLEVEL = tb_fpga
COCOTB_TEST_MODULES = test_fpga_pico
include $(shell cocotb-config --makefiles)/Makefile.sim

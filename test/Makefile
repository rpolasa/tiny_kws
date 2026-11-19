# Makefile - cocotb tests for tt_um_rakesh_tiny_kws
# RTL:  make            GL:  GATES=yes make   (needs gate_level_netlist.v and PDK_ROOT)
SIM ?= icarus
FST ?= -fst
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = project.v kws_spi_slave.v kws_frontend.v kws_coef_rom.v kws_qspi.v kws_nn.v kws_sreg.v

ifneq ($(GATES),yes)
SIM_BUILD        = sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
else
SIM_BUILD        = sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DUSE_POWER_PINS
COMPILE_ARGS    += -DSIM
COMPILE_ARGS    += -DUNIT_DELAY=\#1
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v
endif

ifeq ($(SCANFF),yes)
ifneq ($(GATES),yes)
COMPILE_ARGS    += -DKWS_SKY130_SCANFF
VERILOG_SOURCES += $(PWD)/sky130_scanff_stub.v
SIM_BUILD        = sim_build/rtl_scanff
endif
endif

COMPILE_ARGS    += -I$(SRC_DIR)
VERILOG_SOURCES += $(PWD)/tb.v $(PWD)/psram_model.v
TOPLEVEL = tb
COCOTB_TEST_MODULES ?= test
MODULE ?= $(COCOTB_TEST_MODULES)

include $(shell cocotb-config --makefiles)/Makefile.sim

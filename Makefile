# Git recursive update (excludes ARA toolchain)
.PHONY : git_update
git_update: 
	git -c submodule.toolchain/riscv-gnu-toolchain.update=none \
	-c submodule.toolchain/riscv-isa-sim.update=none \
	-c submodule.toolchain/verilator.update=none \
	-c submodule.toolchain/newlib.update=none \
	-c submodule.toolchain/riscv-llvm.update=none \
	submodule update --init --recursive

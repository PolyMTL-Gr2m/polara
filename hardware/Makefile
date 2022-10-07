# QuestaSim Version
questa_version 	?= 2021.2
# QuestaSim command
questa_cmd    	?= questa-$(questa_version)

# Build path
buildpath 		?= build

# Libraries
library 		?= work

# ARA configuration
config 			:= default
config_file 	:= deps/ara/config/$(config).mk
include $(abspath deps/ara/config/$(config).mk)

# VSIM options
vlog_args 		+= -suppress vlog-2583 -suppress vlog-13314 -suppress vlog-13233
vlog_args 		+= -work $(library)

# Bender options
bender_defs += --define NR_LANES=$(nr_lanes) --define VLEN=$(vlen) --define RVV_ARIANE=1

# Bender
bender:
	@[ -x ./bender ] && echo "Bender already exists." || \
	curl --proto '=https' --tlsv1.2 https://fabianschuiki.github.io/bender/init -sSf | sh	-s -- 0.23.1
	@echo "$$(./bender --version) available."

# Compile
.PHONY: compile
compile: bender $(buildpath)/compile_$(config).tcl
$(buildpath)/compile_$(config).tcl: $(config_file) Makefile ../Bender.yml $(shell find src -type f) $(shell find deps/ara/config -type f) $(shell find include -type f) $(shell find deps -type f)
	./bender script vsim --vlog-arg="$(vlog_args)" -t rtl -t asic $(bender_defs) > $(buildpath)/compile_$(config).tcl
	echo "exit" >> $(buildpath)/compile_$(config).tcl
	cd $(buildpath) && $(questa_cmd) vsim -work $(library) -c -do compile_$(config).tcl
	# Remove the file if compilation did not succee
	if [ `cat $(buildpath)/transcript | grep "\*\* Error" | wc -l` -ne 0 ]; then rm $(buildpath)/compile_$(config).tcl; fill
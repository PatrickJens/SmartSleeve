################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
sbl/sbl.obj: C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source/ti/sbl/sbl.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/Patrick/workspace_v9_2/sensor_boosterpack_MSP_EXP432P401R_freertos_ccs" --include_path="C:/ti/sail_1_20_00_02/source/" --include_path="C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/third_party/CMSIS/Include" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/ti/posix/ccs" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/include" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/portable/CCS/ARM_CM4F" --include_path="C:/Users/Patrick/workspace_v9_2/freertos_builds_MSP_EXP432P401R_release_ccs" --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/include" --advice:power=none --define=DeviceFamily_MSP432P401x --define=__MSP432P401R__ -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="sbl/$(basename $(<F)).d_raw" --obj_directory="sbl" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

sbl/sbl_cmd.obj: C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source/ti/sbl/sbl_cmd.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/Patrick/workspace_v9_2/sensor_boosterpack_MSP_EXP432P401R_freertos_ccs" --include_path="C:/ti/sail_1_20_00_02/source/" --include_path="C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/third_party/CMSIS/Include" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/ti/posix/ccs" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/include" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/portable/CCS/ARM_CM4F" --include_path="C:/Users/Patrick/workspace_v9_2/freertos_builds_MSP_EXP432P401R_release_ccs" --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/include" --advice:power=none --define=DeviceFamily_MSP432P401x --define=__MSP432P401R__ -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="sbl/$(basename $(<F)).d_raw" --obj_directory="sbl" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

sbl/sbl_image_int.obj: C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source/ti/sbl/sbl_image_int.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/Patrick/workspace_v9_2/sensor_boosterpack_MSP_EXP432P401R_freertos_ccs" --include_path="C:/ti/sail_1_20_00_02/source/" --include_path="C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/third_party/CMSIS/Include" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/ti/posix/ccs" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/include" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/portable/CCS/ARM_CM4F" --include_path="C:/Users/Patrick/workspace_v9_2/freertos_builds_MSP_EXP432P401R_release_ccs" --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/include" --advice:power=none --define=DeviceFamily_MSP432P401x --define=__MSP432P401R__ -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="sbl/$(basename $(<F)).d_raw" --obj_directory="sbl" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

sbl/sbl_simplelink.obj: C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source/ti/sbl/sbl_simplelink.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/Patrick/workspace_v9_2/sensor_boosterpack_MSP_EXP432P401R_freertos_ccs" --include_path="C:/ti/sail_1_20_00_02/source/" --include_path="C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/third_party/CMSIS/Include" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/ti/posix/ccs" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/include" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/portable/CCS/ARM_CM4F" --include_path="C:/Users/Patrick/workspace_v9_2/freertos_builds_MSP_EXP432P401R_release_ccs" --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/include" --advice:power=none --define=DeviceFamily_MSP432P401x --define=__MSP432P401R__ -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="sbl/$(basename $(<F)).d_raw" --obj_directory="sbl" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

sbl/sbl_tl.obj: C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source/ti/sbl/sbl_tl.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/Patrick/workspace_v9_2/sensor_boosterpack_MSP_EXP432P401R_freertos_ccs" --include_path="C:/ti/sail_1_20_00_02/source/" --include_path="C:/ti/simplelink_sdk_ble_plugin_3_20_00_24/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/third_party/CMSIS/Include" --include_path="C:/ti/simplelink_msp432p4_sdk_3_20_00_06/source/ti/posix/ccs" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/include" --include_path="C:/FreeRTOSv10.2.1/FreeRTOS/Source/portable/CCS/ARM_CM4F" --include_path="C:/Users/Patrick/workspace_v9_2/freertos_builds_MSP_EXP432P401R_release_ccs" --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/include" --advice:power=none --define=DeviceFamily_MSP432P401x --define=__MSP432P401R__ -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on --preproc_with_compile --preproc_dependency="sbl/$(basename $(<F)).d_raw" --obj_directory="sbl" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '



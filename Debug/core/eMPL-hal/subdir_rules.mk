################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
core/eMPL-hal/eMPL_outputs.obj: ../core/eMPL-hal/eMPL_outputs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-msp430_4.4.5/bin/cl430" -vmspx --abi=coffabi --code_model=large --data_model=restricted --near_data=globals -O0 --opt_for_speed=5 --use_hw_mpy=F5 --include_path="C:/ti/ccsv6/ccs_base/msp430/include" --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-msp430_4.4.5/include" --include_path="C:/Users/harper/workspace_v6_1/boat/core/driver/eMPL" --include_path="C:/Users/harper/workspace_v6_1/boat/core/driver/include" --include_path="C:/Users/harper/workspace_v6_1/boat/core/driver/msp430" --include_path="C:/Users/harper/workspace_v6_1/boat/core/driver/msp430/F5xx_F6xx_Core_Lib" --include_path="C:/Users/harper/workspace_v6_1/boat/core/eMPL-hal" --include_path="C:/Users/harper/workspace_v6_1/boat/core/mllite" --include_path="C:/Users/harper/workspace_v6_1/boat/core/mpl" --include_path="C:/Users/harper/workspace_v6_1/boat/app/MSP430" --include_path="C:/Users/harper/workspace_v6_1/boat/driverlib/MSP430F5xx_6xx" -g --gcc --define=USE_DMP --define=MPU9150 --define=I2C_B0 --define=EMPL --define=EMPL_TARGET_MSP430 --define=__MSP430F5529__ --display_error_number --diag_warning=225 --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU23 --silicon_errata=CPU40 --printf_support=full --preproc_with_compile --preproc_dependency="core/eMPL-hal/eMPL_outputs.pp" --obj_directory="core/eMPL-hal" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '



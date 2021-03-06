#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ds_test_code.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ds_test_code.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS
SUB_IMAGE_ADDRESS_COMMAND=--image-address $(SUB_IMAGE_ADDRESS)
else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=pic/components/oscillator.c pic/components/led.c pic/components/sensor.c pic/components/motor.c pic/components/encoder.c pic/adc.c pic/motor_control.c pic/procedures/straights_procedure.c pic/wait.c pic/procedures/startup_procedure.c pic_main.c pic/procedures/explore_procedure.c pic/procedures/hug_procedure.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/pic/components/oscillator.o ${OBJECTDIR}/pic/components/led.o ${OBJECTDIR}/pic/components/sensor.o ${OBJECTDIR}/pic/components/motor.o ${OBJECTDIR}/pic/components/encoder.o ${OBJECTDIR}/pic/adc.o ${OBJECTDIR}/pic/motor_control.o ${OBJECTDIR}/pic/procedures/straights_procedure.o ${OBJECTDIR}/pic/wait.o ${OBJECTDIR}/pic/procedures/startup_procedure.o ${OBJECTDIR}/pic_main.o ${OBJECTDIR}/pic/procedures/explore_procedure.o ${OBJECTDIR}/pic/procedures/hug_procedure.o
POSSIBLE_DEPFILES=${OBJECTDIR}/pic/components/oscillator.o.d ${OBJECTDIR}/pic/components/led.o.d ${OBJECTDIR}/pic/components/sensor.o.d ${OBJECTDIR}/pic/components/motor.o.d ${OBJECTDIR}/pic/components/encoder.o.d ${OBJECTDIR}/pic/adc.o.d ${OBJECTDIR}/pic/motor_control.o.d ${OBJECTDIR}/pic/procedures/straights_procedure.o.d ${OBJECTDIR}/pic/wait.o.d ${OBJECTDIR}/pic/procedures/startup_procedure.o.d ${OBJECTDIR}/pic_main.o.d ${OBJECTDIR}/pic/procedures/explore_procedure.o.d ${OBJECTDIR}/pic/procedures/hug_procedure.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/pic/components/oscillator.o ${OBJECTDIR}/pic/components/led.o ${OBJECTDIR}/pic/components/sensor.o ${OBJECTDIR}/pic/components/motor.o ${OBJECTDIR}/pic/components/encoder.o ${OBJECTDIR}/pic/adc.o ${OBJECTDIR}/pic/motor_control.o ${OBJECTDIR}/pic/procedures/straights_procedure.o ${OBJECTDIR}/pic/wait.o ${OBJECTDIR}/pic/procedures/startup_procedure.o ${OBJECTDIR}/pic_main.o ${OBJECTDIR}/pic/procedures/explore_procedure.o ${OBJECTDIR}/pic/procedures/hug_procedure.o

# Source Files
SOURCEFILES=pic/components/oscillator.c pic/components/led.c pic/components/sensor.c pic/components/motor.c pic/components/encoder.c pic/adc.c pic/motor_control.c pic/procedures/straights_procedure.c pic/wait.c pic/procedures/startup_procedure.c pic_main.c pic/procedures/explore_procedure.c pic/procedures/hug_procedure.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/ds_test_code.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33FJ128MC804
MP_LINKER_FILE_OPTION=,--script=p33FJ128MC804.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/pic/components/oscillator.o: pic/components/oscillator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/oscillator.o.d 
	@${RM} ${OBJECTDIR}/pic/components/oscillator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/oscillator.c  -o ${OBJECTDIR}/pic/components/oscillator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/oscillator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/oscillator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/components/led.o: pic/components/led.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/led.o.d 
	@${RM} ${OBJECTDIR}/pic/components/led.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/led.c  -o ${OBJECTDIR}/pic/components/led.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/led.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/led.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/components/sensor.o: pic/components/sensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/sensor.o.d 
	@${RM} ${OBJECTDIR}/pic/components/sensor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/sensor.c  -o ${OBJECTDIR}/pic/components/sensor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/sensor.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/sensor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/components/motor.o: pic/components/motor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/motor.o.d 
	@${RM} ${OBJECTDIR}/pic/components/motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/motor.c  -o ${OBJECTDIR}/pic/components/motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/motor.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/motor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/components/encoder.o: pic/components/encoder.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/encoder.o.d 
	@${RM} ${OBJECTDIR}/pic/components/encoder.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/encoder.c  -o ${OBJECTDIR}/pic/components/encoder.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/encoder.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/encoder.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/adc.o: pic/adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic" 
	@${RM} ${OBJECTDIR}/pic/adc.o.d 
	@${RM} ${OBJECTDIR}/pic/adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/adc.c  -o ${OBJECTDIR}/pic/adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/adc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/adc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/motor_control.o: pic/motor_control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic" 
	@${RM} ${OBJECTDIR}/pic/motor_control.o.d 
	@${RM} ${OBJECTDIR}/pic/motor_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/motor_control.c  -o ${OBJECTDIR}/pic/motor_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/motor_control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/motor_control.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/procedures/straights_procedure.o: pic/procedures/straights_procedure.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/procedures" 
	@${RM} ${OBJECTDIR}/pic/procedures/straights_procedure.o.d 
	@${RM} ${OBJECTDIR}/pic/procedures/straights_procedure.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/procedures/straights_procedure.c  -o ${OBJECTDIR}/pic/procedures/straights_procedure.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/procedures/straights_procedure.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/procedures/straights_procedure.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/wait.o: pic/wait.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic" 
	@${RM} ${OBJECTDIR}/pic/wait.o.d 
	@${RM} ${OBJECTDIR}/pic/wait.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/wait.c  -o ${OBJECTDIR}/pic/wait.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/wait.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/wait.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/procedures/startup_procedure.o: pic/procedures/startup_procedure.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/procedures" 
	@${RM} ${OBJECTDIR}/pic/procedures/startup_procedure.o.d 
	@${RM} ${OBJECTDIR}/pic/procedures/startup_procedure.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/procedures/startup_procedure.c  -o ${OBJECTDIR}/pic/procedures/startup_procedure.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/procedures/startup_procedure.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/procedures/startup_procedure.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic_main.o: pic_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/pic_main.o.d 
	@${RM} ${OBJECTDIR}/pic_main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic_main.c  -o ${OBJECTDIR}/pic_main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic_main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic_main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/procedures/explore_procedure.o: pic/procedures/explore_procedure.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/procedures" 
	@${RM} ${OBJECTDIR}/pic/procedures/explore_procedure.o.d 
	@${RM} ${OBJECTDIR}/pic/procedures/explore_procedure.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/procedures/explore_procedure.c  -o ${OBJECTDIR}/pic/procedures/explore_procedure.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/procedures/explore_procedure.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/procedures/explore_procedure.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/procedures/hug_procedure.o: pic/procedures/hug_procedure.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/procedures" 
	@${RM} ${OBJECTDIR}/pic/procedures/hug_procedure.o.d 
	@${RM} ${OBJECTDIR}/pic/procedures/hug_procedure.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/procedures/hug_procedure.c  -o ${OBJECTDIR}/pic/procedures/hug_procedure.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/procedures/hug_procedure.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/procedures/hug_procedure.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/pic/components/oscillator.o: pic/components/oscillator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/oscillator.o.d 
	@${RM} ${OBJECTDIR}/pic/components/oscillator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/oscillator.c  -o ${OBJECTDIR}/pic/components/oscillator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/oscillator.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/oscillator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/components/led.o: pic/components/led.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/led.o.d 
	@${RM} ${OBJECTDIR}/pic/components/led.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/led.c  -o ${OBJECTDIR}/pic/components/led.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/led.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/led.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/components/sensor.o: pic/components/sensor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/sensor.o.d 
	@${RM} ${OBJECTDIR}/pic/components/sensor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/sensor.c  -o ${OBJECTDIR}/pic/components/sensor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/sensor.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/sensor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/components/motor.o: pic/components/motor.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/motor.o.d 
	@${RM} ${OBJECTDIR}/pic/components/motor.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/motor.c  -o ${OBJECTDIR}/pic/components/motor.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/motor.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/motor.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/components/encoder.o: pic/components/encoder.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/components" 
	@${RM} ${OBJECTDIR}/pic/components/encoder.o.d 
	@${RM} ${OBJECTDIR}/pic/components/encoder.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/components/encoder.c  -o ${OBJECTDIR}/pic/components/encoder.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/components/encoder.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/components/encoder.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/adc.o: pic/adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic" 
	@${RM} ${OBJECTDIR}/pic/adc.o.d 
	@${RM} ${OBJECTDIR}/pic/adc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/adc.c  -o ${OBJECTDIR}/pic/adc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/adc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/adc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/motor_control.o: pic/motor_control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic" 
	@${RM} ${OBJECTDIR}/pic/motor_control.o.d 
	@${RM} ${OBJECTDIR}/pic/motor_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/motor_control.c  -o ${OBJECTDIR}/pic/motor_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/motor_control.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/motor_control.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/procedures/straights_procedure.o: pic/procedures/straights_procedure.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/procedures" 
	@${RM} ${OBJECTDIR}/pic/procedures/straights_procedure.o.d 
	@${RM} ${OBJECTDIR}/pic/procedures/straights_procedure.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/procedures/straights_procedure.c  -o ${OBJECTDIR}/pic/procedures/straights_procedure.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/procedures/straights_procedure.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/procedures/straights_procedure.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/wait.o: pic/wait.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic" 
	@${RM} ${OBJECTDIR}/pic/wait.o.d 
	@${RM} ${OBJECTDIR}/pic/wait.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/wait.c  -o ${OBJECTDIR}/pic/wait.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/wait.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/wait.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/procedures/startup_procedure.o: pic/procedures/startup_procedure.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/procedures" 
	@${RM} ${OBJECTDIR}/pic/procedures/startup_procedure.o.d 
	@${RM} ${OBJECTDIR}/pic/procedures/startup_procedure.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/procedures/startup_procedure.c  -o ${OBJECTDIR}/pic/procedures/startup_procedure.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/procedures/startup_procedure.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/procedures/startup_procedure.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic_main.o: pic_main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/pic_main.o.d 
	@${RM} ${OBJECTDIR}/pic_main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic_main.c  -o ${OBJECTDIR}/pic_main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic_main.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic_main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/procedures/explore_procedure.o: pic/procedures/explore_procedure.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/procedures" 
	@${RM} ${OBJECTDIR}/pic/procedures/explore_procedure.o.d 
	@${RM} ${OBJECTDIR}/pic/procedures/explore_procedure.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/procedures/explore_procedure.c  -o ${OBJECTDIR}/pic/procedures/explore_procedure.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/procedures/explore_procedure.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/procedures/explore_procedure.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/pic/procedures/hug_procedure.o: pic/procedures/hug_procedure.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/pic/procedures" 
	@${RM} ${OBJECTDIR}/pic/procedures/hug_procedure.o.d 
	@${RM} ${OBJECTDIR}/pic/procedures/hug_procedure.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  pic/procedures/hug_procedure.c  -o ${OBJECTDIR}/pic/procedures/hug_procedure.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/pic/procedures/hug_procedure.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -O0 -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/pic/procedures/hug_procedure.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/ds_test_code.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/ds_test_code.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/ds_test_code.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/ds_test_code.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}/xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/ds_test_code.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif

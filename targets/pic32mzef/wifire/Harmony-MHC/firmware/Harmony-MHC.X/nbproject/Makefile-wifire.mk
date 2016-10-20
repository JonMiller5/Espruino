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
ifeq "$(wildcard nbproject/Makefile-local-wifire.mk)" "nbproject/Makefile-local-wifire.mk"
include nbproject/Makefile-local-wifire.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=wifire
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Harmony-MHC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Harmony-MHC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/app.c ../src/main.c ../../../../../../../../microchip/harmony/v2_01b/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs_media_manager.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/file_system/ff.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/hardware_access/diskio.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/int/src/sys_int_pic32.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/tmr/src/sys_tmr.c ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host.c ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs_host.c ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_msd.c ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_scsi.c ../src/system_config/wifire/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static.c ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static_read_write.c ../src/system_config/wifire/framework/system/clk/src/sys_clk_static.c ../src/system_config/wifire/framework/system/ports/src/sys_ports_static.c ../src/system_config/wifire/bsp/bsp.c ../src/system_config/wifire/system_init.c ../src/system_config/wifire/system_interrupt.c ../src/system_config/wifire/system_exceptions.c ../src/system_config/wifire/system_tasks.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1223344495/drv_tmr.o ${OBJECTDIR}/_ext/636108392/sys_devcon.o ${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/60175848/sys_fs.o ${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o ${OBJECTDIR}/_ext/1271367993/ff.o ${OBJECTDIR}/_ext/1143829904/diskio.o ${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o ${OBJECTDIR}/_ext/1941867160/sys_tmr.o ${OBJECTDIR}/_ext/1227371524/drv_usbhs.o ${OBJECTDIR}/_ext/1236524817/usb_host.o ${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o ${OBJECTDIR}/_ext/1236524817/usb_host_msd.o ${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o ${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o ${OBJECTDIR}/_ext/1269890196/drv_usart_static.o ${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o ${OBJECTDIR}/_ext/25136712/sys_clk_static.o ${OBJECTDIR}/_ext/120841320/sys_ports_static.o ${OBJECTDIR}/_ext/1882089905/bsp.o ${OBJECTDIR}/_ext/1337531873/system_init.o ${OBJECTDIR}/_ext/1337531873/system_interrupt.o ${OBJECTDIR}/_ext/1337531873/system_exceptions.o ${OBJECTDIR}/_ext/1337531873/system_tasks.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360937237/app.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1223344495/drv_tmr.o.d ${OBJECTDIR}/_ext/636108392/sys_devcon.o.d ${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/60175848/sys_fs.o.d ${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o.d ${OBJECTDIR}/_ext/1271367993/ff.o.d ${OBJECTDIR}/_ext/1143829904/diskio.o.d ${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o.d ${OBJECTDIR}/_ext/1941867160/sys_tmr.o.d ${OBJECTDIR}/_ext/1227371524/drv_usbhs.o.d ${OBJECTDIR}/_ext/1236524817/usb_host.o.d ${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o.d ${OBJECTDIR}/_ext/1236524817/usb_host_msd.o.d ${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o.d ${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o.d ${OBJECTDIR}/_ext/1269890196/drv_usart_static.o.d ${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o.d ${OBJECTDIR}/_ext/25136712/sys_clk_static.o.d ${OBJECTDIR}/_ext/120841320/sys_ports_static.o.d ${OBJECTDIR}/_ext/1882089905/bsp.o.d ${OBJECTDIR}/_ext/1337531873/system_init.o.d ${OBJECTDIR}/_ext/1337531873/system_interrupt.o.d ${OBJECTDIR}/_ext/1337531873/system_exceptions.o.d ${OBJECTDIR}/_ext/1337531873/system_tasks.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1223344495/drv_tmr.o ${OBJECTDIR}/_ext/636108392/sys_devcon.o ${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/60175848/sys_fs.o ${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o ${OBJECTDIR}/_ext/1271367993/ff.o ${OBJECTDIR}/_ext/1143829904/diskio.o ${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o ${OBJECTDIR}/_ext/1941867160/sys_tmr.o ${OBJECTDIR}/_ext/1227371524/drv_usbhs.o ${OBJECTDIR}/_ext/1236524817/usb_host.o ${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o ${OBJECTDIR}/_ext/1236524817/usb_host_msd.o ${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o ${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o ${OBJECTDIR}/_ext/1269890196/drv_usart_static.o ${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o ${OBJECTDIR}/_ext/25136712/sys_clk_static.o ${OBJECTDIR}/_ext/120841320/sys_ports_static.o ${OBJECTDIR}/_ext/1882089905/bsp.o ${OBJECTDIR}/_ext/1337531873/system_init.o ${OBJECTDIR}/_ext/1337531873/system_interrupt.o ${OBJECTDIR}/_ext/1337531873/system_exceptions.o ${OBJECTDIR}/_ext/1337531873/system_tasks.o

# Source Files
SOURCEFILES=../src/app.c ../src/main.c ../../../../../../../../microchip/harmony/v2_01b/framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs_media_manager.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/file_system/ff.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/hardware_access/diskio.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/int/src/sys_int_pic32.c ../../../../../../../../microchip/harmony/v2_01b/framework/system/tmr/src/sys_tmr.c ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host.c ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs_host.c ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_msd.c ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_scsi.c ../src/system_config/wifire/framework/driver/usart/src/drv_usart_mapping.c ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static.c ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static_read_write.c ../src/system_config/wifire/framework/system/clk/src/sys_clk_static.c ../src/system_config/wifire/framework/system/ports/src/sys_ports_static.c ../src/system_config/wifire/bsp/bsp.c ../src/system_config/wifire/system_init.c ../src/system_config/wifire/system_interrupt.c ../src/system_config/wifire/system_exceptions.c ../src/system_config/wifire/system_tasks.c


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
	${MAKE}  -f nbproject/Makefile-wifire.mk dist/${CND_CONF}/${IMAGE_TYPE}/Harmony-MHC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MZ2048EFG100
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/636108392" 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1
	
else
${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/636108392" 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_cache_pic32mz.S  -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/636108392/sys_devcon_cache_pic32mz.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1223344495/drv_tmr.o: ../../../../../../../../microchip/harmony/v2_01b/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1223344495" 
	@${RM} ${OBJECTDIR}/_ext/1223344495/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1223344495/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1223344495/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1223344495/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/1223344495/drv_tmr.o ../../../../../../../../microchip/harmony/v2_01b/framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/636108392/sys_devcon.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/636108392" 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/636108392/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/636108392/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/636108392/sys_devcon.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/636108392" 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/60175848/sys_fs.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/60175848" 
	@${RM} ${OBJECTDIR}/_ext/60175848/sys_fs.o.d 
	@${RM} ${OBJECTDIR}/_ext/60175848/sys_fs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/60175848/sys_fs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/60175848/sys_fs.o.d" -o ${OBJECTDIR}/_ext/60175848/sys_fs.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs_media_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/60175848" 
	@${RM} ${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o.d" -o ${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs_media_manager.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1271367993/ff.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/file_system/ff.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1271367993" 
	@${RM} ${OBJECTDIR}/_ext/1271367993/ff.o.d 
	@${RM} ${OBJECTDIR}/_ext/1271367993/ff.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1271367993/ff.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1271367993/ff.o.d" -o ${OBJECTDIR}/_ext/1271367993/ff.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/file_system/ff.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1143829904/diskio.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/hardware_access/diskio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1143829904" 
	@${RM} ${OBJECTDIR}/_ext/1143829904/diskio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1143829904/diskio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1143829904/diskio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1143829904/diskio.o.d" -o ${OBJECTDIR}/_ext/1143829904/diskio.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/hardware_access/diskio.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1210970430" 
	@${RM} ${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/int/src/sys_int_pic32.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1941867160/sys_tmr.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1941867160" 
	@${RM} ${OBJECTDIR}/_ext/1941867160/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1941867160/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1941867160/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1941867160/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/1941867160/sys_tmr.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/tmr/src/sys_tmr.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1227371524/drv_usbhs.o: ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1227371524" 
	@${RM} ${OBJECTDIR}/_ext/1227371524/drv_usbhs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1227371524/drv_usbhs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1227371524/drv_usbhs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1227371524/drv_usbhs.o.d" -o ${OBJECTDIR}/_ext/1227371524/drv_usbhs.o ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1236524817/usb_host.o: ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1236524817" 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1236524817/usb_host.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1236524817/usb_host.o.d" -o ${OBJECTDIR}/_ext/1236524817/usb_host.o ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o: ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1227371524" 
	@${RM} ${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o.d" -o ${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs_host.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1236524817/usb_host_msd.o: ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_msd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1236524817" 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host_msd.o.d 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host_msd.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1236524817/usb_host_msd.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1236524817/usb_host_msd.o.d" -o ${OBJECTDIR}/_ext/1236524817/usb_host_msd.o ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_msd.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o: ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_scsi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1236524817" 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o.d" -o ${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_scsi.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o: ../src/system_config/wifire/framework/driver/usart/src/drv_usart_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1269890196" 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o ../src/system_config/wifire/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1269890196/drv_usart_static.o: ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1269890196" 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1269890196/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1269890196/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/1269890196/drv_usart_static.o ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o: ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static_read_write.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1269890196" 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o.d 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o.d" -o ${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static_read_write.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/25136712/sys_clk_static.o: ../src/system_config/wifire/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/25136712" 
	@${RM} ${OBJECTDIR}/_ext/25136712/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/25136712/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/25136712/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/25136712/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/25136712/sys_clk_static.o ../src/system_config/wifire/framework/system/clk/src/sys_clk_static.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/120841320/sys_ports_static.o: ../src/system_config/wifire/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/120841320" 
	@${RM} ${OBJECTDIR}/_ext/120841320/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/120841320/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/120841320/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/120841320/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/120841320/sys_ports_static.o ../src/system_config/wifire/framework/system/ports/src/sys_ports_static.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1882089905/bsp.o: ../src/system_config/wifire/bsp/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1882089905" 
	@${RM} ${OBJECTDIR}/_ext/1882089905/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1882089905/bsp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1882089905/bsp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1882089905/bsp.o.d" -o ${OBJECTDIR}/_ext/1882089905/bsp.o ../src/system_config/wifire/bsp/bsp.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1337531873/system_init.o: ../src/system_config/wifire/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1337531873" 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1337531873/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1337531873/system_init.o.d" -o ${OBJECTDIR}/_ext/1337531873/system_init.o ../src/system_config/wifire/system_init.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1337531873/system_interrupt.o: ../src/system_config/wifire/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1337531873" 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1337531873/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1337531873/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1337531873/system_interrupt.o ../src/system_config/wifire/system_interrupt.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1337531873/system_exceptions.o: ../src/system_config/wifire/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1337531873" 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1337531873/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1337531873/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1337531873/system_exceptions.o ../src/system_config/wifire/system_exceptions.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1337531873/system_tasks.o: ../src/system_config/wifire/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1337531873" 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1337531873/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1337531873/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1337531873/system_tasks.o ../src/system_config/wifire/system_tasks.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1223344495/drv_tmr.o: ../../../../../../../../microchip/harmony/v2_01b/framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1223344495" 
	@${RM} ${OBJECTDIR}/_ext/1223344495/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1223344495/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1223344495/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1223344495/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/1223344495/drv_tmr.o ../../../../../../../../microchip/harmony/v2_01b/framework/driver/tmr/src/dynamic/drv_tmr.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/636108392/sys_devcon.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/636108392" 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/636108392/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/636108392/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/636108392/sys_devcon.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/636108392" 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/636108392/sys_devcon_pic32mz.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/devcon/src/sys_devcon_pic32mz.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/60175848/sys_fs.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/60175848" 
	@${RM} ${OBJECTDIR}/_ext/60175848/sys_fs.o.d 
	@${RM} ${OBJECTDIR}/_ext/60175848/sys_fs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/60175848/sys_fs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/60175848/sys_fs.o.d" -o ${OBJECTDIR}/_ext/60175848/sys_fs.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs_media_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/60175848" 
	@${RM} ${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o.d" -o ${OBJECTDIR}/_ext/60175848/sys_fs_media_manager.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/src/dynamic/sys_fs_media_manager.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1271367993/ff.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/file_system/ff.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1271367993" 
	@${RM} ${OBJECTDIR}/_ext/1271367993/ff.o.d 
	@${RM} ${OBJECTDIR}/_ext/1271367993/ff.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1271367993/ff.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1271367993/ff.o.d" -o ${OBJECTDIR}/_ext/1271367993/ff.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/file_system/ff.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1143829904/diskio.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/hardware_access/diskio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1143829904" 
	@${RM} ${OBJECTDIR}/_ext/1143829904/diskio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1143829904/diskio.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1143829904/diskio.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1143829904/diskio.o.d" -o ${OBJECTDIR}/_ext/1143829904/diskio.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/fs/fat_fs/src/hardware_access/diskio.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1210970430" 
	@${RM} ${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/1210970430/sys_int_pic32.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/int/src/sys_int_pic32.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1941867160/sys_tmr.o: ../../../../../../../../microchip/harmony/v2_01b/framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1941867160" 
	@${RM} ${OBJECTDIR}/_ext/1941867160/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1941867160/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1941867160/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1941867160/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/1941867160/sys_tmr.o ../../../../../../../../microchip/harmony/v2_01b/framework/system/tmr/src/sys_tmr.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1227371524/drv_usbhs.o: ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1227371524" 
	@${RM} ${OBJECTDIR}/_ext/1227371524/drv_usbhs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1227371524/drv_usbhs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1227371524/drv_usbhs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1227371524/drv_usbhs.o.d" -o ${OBJECTDIR}/_ext/1227371524/drv_usbhs.o ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1236524817/usb_host.o: ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1236524817" 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1236524817/usb_host.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1236524817/usb_host.o.d" -o ${OBJECTDIR}/_ext/1236524817/usb_host.o ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o: ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs_host.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1227371524" 
	@${RM} ${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o.d 
	@${RM} ${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o.d" -o ${OBJECTDIR}/_ext/1227371524/drv_usbhs_host.o ../../../../../../../../microchip/harmony/v2_01b/framework/driver/usb/usbhs/src/dynamic/drv_usbhs_host.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1236524817/usb_host_msd.o: ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_msd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1236524817" 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host_msd.o.d 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host_msd.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1236524817/usb_host_msd.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1236524817/usb_host_msd.o.d" -o ${OBJECTDIR}/_ext/1236524817/usb_host_msd.o ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_msd.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o: ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_scsi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1236524817" 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o.d" -o ${OBJECTDIR}/_ext/1236524817/usb_host_scsi.o ../../../../../../../../microchip/harmony/v2_01b/framework/usb/src/dynamic/usb_host_scsi.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o: ../src/system_config/wifire/framework/driver/usart/src/drv_usart_mapping.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1269890196" 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o.d 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o.d" -o ${OBJECTDIR}/_ext/1269890196/drv_usart_mapping.o ../src/system_config/wifire/framework/driver/usart/src/drv_usart_mapping.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1269890196/drv_usart_static.o: ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1269890196" 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1269890196/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1269890196/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/1269890196/drv_usart_static.o ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o: ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static_read_write.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1269890196" 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o.d 
	@${RM} ${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o.d" -o ${OBJECTDIR}/_ext/1269890196/drv_usart_static_read_write.o ../src/system_config/wifire/framework/driver/usart/src/drv_usart_static_read_write.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/25136712/sys_clk_static.o: ../src/system_config/wifire/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/25136712" 
	@${RM} ${OBJECTDIR}/_ext/25136712/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/25136712/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/25136712/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/25136712/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/25136712/sys_clk_static.o ../src/system_config/wifire/framework/system/clk/src/sys_clk_static.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/120841320/sys_ports_static.o: ../src/system_config/wifire/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/120841320" 
	@${RM} ${OBJECTDIR}/_ext/120841320/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/120841320/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/120841320/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/120841320/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/120841320/sys_ports_static.o ../src/system_config/wifire/framework/system/ports/src/sys_ports_static.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1882089905/bsp.o: ../src/system_config/wifire/bsp/bsp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1882089905" 
	@${RM} ${OBJECTDIR}/_ext/1882089905/bsp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1882089905/bsp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1882089905/bsp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1882089905/bsp.o.d" -o ${OBJECTDIR}/_ext/1882089905/bsp.o ../src/system_config/wifire/bsp/bsp.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1337531873/system_init.o: ../src/system_config/wifire/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1337531873" 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1337531873/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1337531873/system_init.o.d" -o ${OBJECTDIR}/_ext/1337531873/system_init.o ../src/system_config/wifire/system_init.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1337531873/system_interrupt.o: ../src/system_config/wifire/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1337531873" 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1337531873/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1337531873/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1337531873/system_interrupt.o ../src/system_config/wifire/system_interrupt.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1337531873/system_exceptions.o: ../src/system_config/wifire/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1337531873" 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1337531873/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1337531873/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1337531873/system_exceptions.o ../src/system_config/wifire/system_exceptions.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1337531873/system_tasks.o: ../src/system_config/wifire/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1337531873" 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1337531873/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1337531873/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/wifire" -I"../../../../../../../../microchip/harmony/v2_01b/framework" -I"../src/system_config/wifire/framework" -I"../src/system_config/wifire/bsp" -MMD -MF "${OBJECTDIR}/_ext/1337531873/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1337531873/system_tasks.o ../src/system_config/wifire/system_tasks.c    -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Harmony-MHC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../../../microchip/harmony/v2_01b/bin/framework/peripheral/PIC32MZ2048EFG100_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Harmony-MHC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../../../../microchip/harmony/v2_01b/bin/framework/peripheral/PIC32MZ2048EFG100_peripherals.a      -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x37F   -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,--defsym=_min_heap_size=500,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Harmony-MHC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../../../microchip/harmony/v2_01b/bin/framework/peripheral/PIC32MZ2048EFG100_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Harmony-MHC.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ../../../../../../../../microchip/harmony/v2_01b/bin/framework/peripheral/PIC32MZ2048EFG100_peripherals.a      -DXPRJ_wifire=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=500,--gc-sections,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Harmony-MHC.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/wifire
	${RM} -r dist/wifire

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif

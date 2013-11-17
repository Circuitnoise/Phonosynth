################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../libMozzi \
../libMozzi/config \
../libMozzi/tables


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../libMozzi/FrequencyTimer2.cpp \
../libMozzi/MozziGuts.cpp \
../libMozzi/mozzi_analog.cpp \
../libMozzi/mozzi_fixmath.cpp \
../libMozzi/mozzi_midi.cpp \
../libMozzi/mozzi_rand.cpp \
../libMozzi/mozzi_utils.cpp \
../libMozzi/TimerOne.cpp \
../libMozzi/TimerZero.cpp \
../PhonosynthAtmega88.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
libMozzi/FrequencyTimer2.o \
libMozzi/MozziGuts.o \
libMozzi/mozzi_analog.o \
libMozzi/mozzi_fixmath.o \
libMozzi/mozzi_midi.o \
libMozzi/mozzi_rand.o \
libMozzi/mozzi_utils.o \
libMozzi/TimerOne.o \
libMozzi/TimerZero.o \
PhonosynthAtmega88.o

OBJS_AS_ARGS +=  \
libMozzi/FrequencyTimer2.o \
libMozzi/MozziGuts.o \
libMozzi/mozzi_analog.o \
libMozzi/mozzi_fixmath.o \
libMozzi/mozzi_midi.o \
libMozzi/mozzi_rand.o \
libMozzi/mozzi_utils.o \
libMozzi/TimerOne.o \
libMozzi/TimerZero.o \
PhonosynthAtmega88.o

C_DEPS +=  \
libMozzi/FrequencyTimer2.d \
libMozzi/MozziGuts.d \
libMozzi/mozzi_analog.d \
libMozzi/mozzi_fixmath.d \
libMozzi/mozzi_midi.d \
libMozzi/mozzi_rand.d \
libMozzi/mozzi_utils.d \
libMozzi/TimerOne.d \
libMozzi/TimerZero.d \
PhonosynthAtmega88.d

C_DEPS_AS_ARGS +=  \
libMozzi/FrequencyTimer2.d \
libMozzi/MozziGuts.d \
libMozzi/mozzi_analog.d \
libMozzi/mozzi_fixmath.d \
libMozzi/mozzi_midi.d \
libMozzi/mozzi_rand.d \
libMozzi/mozzi_utils.d \
libMozzi/TimerOne.d \
libMozzi/TimerZero.d \
PhonosynthAtmega88.d

OUTPUT_FILE_PATH +=PhonosynthAtmega88.elf

OUTPUT_FILE_PATH_AS_ARGS +=PhonosynthAtmega88.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

# AVR32/GNU C Compiler





















libMozzi/%.o: ../libMozzi/%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 3.4.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -DNDEBUG -DF_CPU=8000000L -DARDUINO=105  -I"../libMozzi" -I"../libArduino"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g3 -Wall -mmcu=atmega88 -c -v -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<"
	@echo Finished building: $<
	

./%.o: .././%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 3.4.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -DNDEBUG -DF_CPU=8000000L -DARDUINO=105  -I"../libMozzi" -I"../libArduino"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g3 -Wall -mmcu=atmega88 -c -v -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<"
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 3.4.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="PhonosynthAtmega88.map" -Wl,--start-group -Wl,-lm -Wl,-lcore  -Wl,--end-group -Wl,-L"../libArduino" -Wl,-L"../libMozzi"  -Wl,--gc-sections -mmcu=atmega88 
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "PhonosynthAtmega88.elf" "PhonosynthAtmega88.hex"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "PhonosynthAtmega88.elf" "PhonosynthAtmega88.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "PhonosynthAtmega88.elf" > "PhonosynthAtmega88.lss"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "PhonosynthAtmega88.elf" "PhonosynthAtmega88.srec"
	"C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR8 GCC\Native\3.4.2.939\avr8-gnu-toolchain\bin\avr-size.exe" "PhonosynthAtmega88.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "PhonosynthAtmega88.elf" "PhonosynthAtmega88.a" "PhonosynthAtmega88.hex" "PhonosynthAtmega88.lss" "PhonosynthAtmega88.eep" "PhonosynthAtmega88.map" "PhonosynthAtmega88.srec"
	
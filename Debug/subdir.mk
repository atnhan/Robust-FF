################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
O_SRCS += \
../inst_easy.o \
../inst_final.o \
../inst_hard.o \
../inst_pre.o \
../main.o \
../memory.o \
../orderings.o \
../output.o \
../parse.o \
../relax.o \
../scan-fct_pddl.tab.o \
../scan-ops_pddl.tab.o \
../search.o 

C_SRCS += \
../inst_easy.c \
../inst_final.c \
../inst_hard.c \
../inst_pre.c \
../lex.fct_pddl.c \
../lex.ops_pddl.c \
../main.c \
../memory.c \
../orderings.c \
../output.c \
../parse.c \
../relax.c \
../scan-fct_pddl.tab.c \
../scan-ops_pddl.tab.c \
../search.c 

OBJS += \
./inst_easy.o \
./inst_final.o \
./inst_hard.o \
./inst_pre.o \
./lex.fct_pddl.o \
./lex.ops_pddl.o \
./main.o \
./memory.o \
./orderings.o \
./output.o \
./parse.o \
./relax.o \
./scan-fct_pddl.tab.o \
./scan-ops_pddl.tab.o \
./search.o 

C_DEPS += \
./inst_easy.d \
./inst_final.d \
./inst_hard.d \
./inst_pre.d \
./lex.fct_pddl.d \
./lex.ops_pddl.d \
./main.d \
./memory.d \
./orderings.d \
./output.d \
./parse.d \
./relax.d \
./scan-fct_pddl.tab.d \
./scan-ops_pddl.tab.d \
./search.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



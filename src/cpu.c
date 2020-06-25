#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "../inc/cpu.h"
#include "../inc/mem.h"
#include "../inc/int.h"

#define A 0
#define B 1
#define C 2
#define D 3
#define E 4
#define F 5
#define H 6
#define L 7

#define FLAG_ZERO 1<<7
#define FLAG_SUBTRACT 1<<6
#define FLAG_HALF_CARRY 1<<5
#define FLAG_CARRY 1<<4

typedef struct {
    uint8_t regFile[8];
    uint16_t sp;
    uint16_t pc;
    int halted;
} Cpu;

Cpu cpu;

int init_cpu() {
    int i;
    for(i = 0; i < 8; i++) {
        cpu.regFile[i] = 0;
    }
    cpu.pc = 0;
    cpu.halted = 0;
    return 0;
}

/*
*   Helper Functions
*/

void set_flag(uint8_t flag) {
    cpu.regFile[F] |= flag;
    return;
}

void clear_flag(uint8_t flag) {
    cpu.regFile[F] &= ~(flag);
    return;
}

void eval_flag(uint8_t flag, int cond) {
    cpu.regFile[F] = cond ? cpu.regFile[F] | flag : cpu.regFile[F] & ~(flag);
    return;
}

void push8(uint8_t val) {
    write(--cpu.sp, val);
    return;
}

void push16(uint16_t val) {
    write(--cpu.sp, (uint8_t)(val >> 8));
    write(--cpu.sp, (uint8_t)(val & 255));
    return;
}

uint8_t pop8() {
    return read(cpu.sp++);
}

uint16_t pop16() {
    uint8_t low = read(cpu.sp++);
    uint8_t high = read(cpu.sp++);
    return (((uint16_t)(high)) << 8) + low;
}


/*
*   Opcodes
*/

int add_carry_A_r8(int reg) {
    uint8_t carry = (cpu.regFile[F] & FLAG_CARRY) >> 4;
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) + (cpu.regFile[reg] & 0xF) + carry) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] + cpu.regFile[reg] + carry) & 0x100);
    cpu.regFile[A] += cpu.regFile[reg] + carry;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 4;
}

int add_carry_A_HL_indirect() {
    uint8_t mem = read((((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L]);
    uint8_t carry = (cpu.regFile[F] & FLAG_CARRY) >> 4;
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) + (mem & 0xF) + carry) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] + mem + carry) & 0x100);
    cpu.regFile[A] += mem + carry;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int add_carry_A_d8() {
    uint8_t imm = read(cpu.pc++);
    uint8_t carry = (cpu.regFile[F] & FLAG_CARRY) >> 4;
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) + (imm & 0xF) + carry) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] + imm + carry) & 0x100);
    cpu.regFile[A] += imm + carry;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int add_A_r8(int reg) {
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) + (cpu.regFile[reg] & 0xF)) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] + cpu.regFile[reg]) & 0x100);
    cpu.regFile[A] += cpu.regFile[reg];
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 4;
}

int add_A_HL_indirect() {
    uint8_t mem = read((((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L]);
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) + (mem & 0xF)) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] + mem) & 0x100);
    cpu.regFile[A] += mem;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int add_A_d8() {
    uint8_t imm = read(cpu.pc++);
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) + (imm & 0xF)) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] + imm) & 0x100);
    cpu.regFile[A] += imm;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int add_HL_r16(int regH, int regL) {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint16_t reg = (((uint16_t)cpu.regFile[regH]) << 8) + cpu.regFile[regL];
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((hl & 0xFFF) > ((hl + reg) & 0xFFF)));
    eval_flag(FLAG_CARRY, (hl & 0xFFFF) > ((hl + reg) & 0xFFFF));
    cpu.regFile[H] = (uint8_t)((hl+reg) >> 8);
    cpu.regFile[L] = (uint8_t)((hl + reg) & 255);
    return 8;
}

int add_HL_SP() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];   
    uint16_t result = hl + cpu.sp;
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, (hl & 0xFFF) > ((result) & 0xFFF));
    eval_flag(FLAG_CARRY, hl > result);
    cpu.regFile[H] = (uint8_t)((result) >> 8);
    cpu.regFile[L] = (uint8_t)((result) & 255);
    return 8;
}

int add_SP_s8() {
    int8_t imm = (int8_t)read(cpu.pc++);
    clear_flag(FLAG_ZERO | FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, (cpu.sp & 0xF) > ((cpu.sp + imm) & 0xF));
    eval_flag(FLAG_CARRY, (cpu.sp & 0xFF) > ((cpu.sp + imm) & 0xFF));
    cpu.sp += imm;
    return 16;
}

int and_A_r8(int reg) {
    clear_flag(FLAG_SUBTRACT | FLAG_CARRY);
    set_flag(FLAG_HALF_CARRY);
    cpu.regFile[A] &= cpu.regFile[reg];
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 4;
}

int and_A_HL_indirect() {
    uint8_t mem = read((((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L]);
    clear_flag(FLAG_SUBTRACT | FLAG_CARRY);
    set_flag(FLAG_HALF_CARRY);
    cpu.regFile[A] &= mem;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int and_A_d8() {
    uint8_t imm = read(cpu.pc++);
    clear_flag(FLAG_SUBTRACT | FLAG_CARRY);
    set_flag(FLAG_HALF_CARRY);
    cpu.regFile[A] &= imm;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int bit_r8(uint8_t bit, int reg) {
    eval_flag(FLAG_ZERO, (cpu.regFile[reg] & (1<<bit)) == 0);
    clear_flag(FLAG_SUBTRACT);
    set_flag(FLAG_HALF_CARRY);
    return 8;
}

int bit_HL_indirect(uint8_t bit) {
    uint8_t mem = read((((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L]);
    eval_flag(FLAG_ZERO, (mem & (1<<bit)) == 0);
    clear_flag(FLAG_SUBTRACT);
    set_flag(FLAG_HALF_CARRY);
    return 12;
}

int call() {
    uint16_t addr = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    push16(cpu.pc);
    cpu.pc = addr;
    return 24;
}

int call_cond(uint8_t flag) {
    uint16_t addr = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    if((cpu.regFile[F] & flag) != 0) {
        push16(cpu.pc);
        cpu.pc = addr;
        return 24;
    }
    return 12;
}

int call_cond_inv(uint8_t flag) {
    uint16_t addr = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    if((cpu.regFile[F] & flag) == 0) {
        push16(cpu.pc);
        cpu.pc = addr;
        return 24;
    }
    return 12;
}

int complement_carry_flag() {
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    cpu.regFile[F] ^= FLAG_CARRY;
    return 4;
}

int compare_A_r8(int reg) {
    eval_flag(FLAG_ZERO, (cpu.regFile[A] - cpu.regFile[reg]) == 0);
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) - (cpu.regFile[reg] & 0xF)) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] - cpu.regFile[reg]) & 0x100);
    return 4;
}

int compare_A_HL_indirect() {
    uint8_t mem = read((((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L]);
    eval_flag(FLAG_ZERO, (cpu.regFile[A] - mem) == 0);
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) - (mem & 0xF)) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] - mem) & 0x100);
    return 8;
}

int compare_A_d8() {
    uint8_t imm = read(cpu.pc++);
    eval_flag(FLAG_ZERO, (cpu.regFile[A] - imm) == 0);
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) - (imm & 0xF)) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] - imm) & 0x100);
    return 8;
}

int complement_A() {
    set_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    cpu.regFile[A] = ~(cpu.regFile[A]);
    return 4;
}

int decimal_adjust_A() {
    // Adapted from code provided by AWJ on nesdev.com
    if (!(cpu.regFile[F] & FLAG_SUBTRACT)) {  // after an addition, adjust if (half-)carry occurred or if result is out of bounds
        if ((cpu.regFile[F] & FLAG_CARRY) || (cpu.regFile[A] > 0x99)) { 
            cpu.regFile[A] += 0x60; 
            set_flag(FLAG_CARRY);
        }
        if ((cpu.regFile[F] & FLAG_HALF_CARRY) || ((cpu.regFile[A] & 0x0F) > 0x09)) { 
            cpu.regFile[A] += 0x6; 
        }
    } 
    else {  // after a subtraction, only adjust if (half-)carry occurred
        if (cpu.regFile[F] & FLAG_CARRY) { cpu.regFile[A] -= 0x60; }
        if (cpu.regFile[F] & FLAG_HALF_CARRY) { cpu.regFile[A] -= 0x6; }
    }
    // these flags are always updated
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    clear_flag(FLAG_HALF_CARRY);
    return 4;
}

int decrement_r8(int reg) {
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[reg] & 0xF) - 1) & 0x10);
    cpu.regFile[reg] -= 1;
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    return 4;
}

int decrement_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L]; 
    uint8_t mem = read(hl);
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((mem & 0xF) - 1) & 0x10);
    write(hl, mem - 1);
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 12;
}

int decrement_r16(int regH, int regL) {
    uint16_t reg = (((uint16_t)cpu.regFile[regH]) << 8) + cpu.regFile[regL] - 1;
    cpu.regFile[regH] = (uint8_t)(reg >> 8);
    cpu.regFile[regL] = (uint8_t)(reg & 0xFF);
    return 8;
}

int decrement_SP() {
    cpu.sp -= 1;
    return 8;
}

int disable_interrupts() {
    set_ime(0);
    return 4;
}

int enable_interrupts() {
    // IME is shifted at the end of every instruction. Once it equals 1, interrupts are enabled
    set_ime(1 << 2);
    return 4;
}

int halt() {
    cpu.halted = 1;
    return 4;
}

int increment_r8(int reg) {
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[reg] & 0xF) + 1) & 0x10);
    cpu.regFile[reg] += 1;
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    return 4;
}

int increment_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];   
    uint8_t mem = read(hl);
    clear_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((mem & 0xF) + 1) & 0x10);
    write(hl, mem + 1);
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 12;
}

int increment_r16(int regH, int regL) {
    uint16_t reg = (((uint16_t)cpu.regFile[regH]) << 8) + cpu.regFile[regL] + 1;
    cpu.regFile[regH] = (uint8_t)(reg >> 8);
    cpu.regFile[regL] = (uint8_t)(reg & 0xFF);
    return 8;
}

int increment_SP() {
    cpu.sp += 1;
    return 8;
}

int jump_d16() {
    uint16_t addr = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    cpu.pc = addr;
    return 16;
}

int jump_cond_d16(uint8_t flag) {
    uint16_t addr = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    if(cpu.regFile[F] & flag) {
        cpu.pc = addr;
        return 16;
    }
    return 12;
}

int jump_cond_inv_d16(uint8_t flag) {
    uint16_t addr = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    if(!(cpu.regFile[F] & flag)) {
        cpu.pc = addr;
        return 16;
    }
    return 12;
}

int jump_HL() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];  
    cpu.pc = hl;
    return 4;
}

int jump_s8() {
    int8_t offset = (int8_t)read(cpu.pc++);
    cpu.pc += offset;
    return 12;
}

int jump_cond_s8(uint8_t flag) {
    int8_t offset = (int8_t)read(cpu.pc++);
    if(cpu.regFile[F] & flag) {
        cpu.pc += offset;
        return 12;
    }
    return 8;
}

int jump_cond_inv_s8(uint8_t flag) {
    int8_t offset = (int8_t)read(cpu.pc++);

    if((cpu.regFile[F] & flag) == 0) {
        cpu.pc += offset;
        return 12;
    }
    return 8;
}

int load_r8_r8(int reg1, int reg2) {
    cpu.regFile[reg1] = cpu.regFile[reg2];
    return 4;
}

int load_r8_d8(int reg) {
    uint8_t imm = read(cpu.pc++);
    cpu.regFile[reg] = imm;
    return 8;
}

int load_r16_d16(int regH, int regL) {
    uint16_t imm = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    cpu.regFile[regH] = (uint8_t)(imm >> 8);
    cpu.regFile[regL] = (uint8_t)(imm & 0xFF);
    return 12;
}

int load_HL_indirect_r8(int reg) {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    write(hl, cpu.regFile[reg]);
    return 8;
}

int load_HL_indirect_d8() {
    uint8_t imm = read(cpu.pc++);
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    write(hl, imm);
    return 8;
}

int load_r8_HL_indirect(int reg) {
    uint8_t mem = read((((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L]);
    cpu.regFile[reg] = mem;
    return 8;
}

int load_r16_indirect_A(int regH, int regL) {
    uint16_t addr = (((uint16_t)cpu.regFile[regH]) << 8) + cpu.regFile[regL];
    write(addr, cpu.regFile[A]);
    return 8;
}

int load_d16_indirect_A() {
    uint16_t addr = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    write(addr, cpu.regFile[A]);
    return 12;
}

int load_io_d16_indirect_A() {
    uint8_t offset = read(cpu.pc++);
    write(0xFF00 + offset, cpu.regFile[A]);
    return 12;
}

int load_io_C_indirect_A() {
    write(0xFF00 + cpu.regFile[C], cpu.regFile[A]);
    return 8;
}

int load_A_r16_indirect(int regH, int regL) {
    uint16_t addr = (((uint16_t)cpu.regFile[regH]) << 8) + cpu.regFile[regL];
    cpu.regFile[A] = read(addr);
    return 8;
}

int load_A_d16_indirect() {
     uint16_t addr = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
     cpu.regFile[A] = read(addr);
     return 16;
}

int load_io_A_d16_indirect() {
    uint8_t offset = read(cpu.pc++);
    cpu.regFile[A] = read(0xFF00 + offset);
    return 12;
}

int load_io_A_C_indirect() {
    cpu.regFile[A] = read(0xFF00 + cpu.regFile[C]);
    return 8;
}

int load_HL_indirect_increment_A() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    write(hl++, cpu.regFile[A]);
    cpu.regFile[H] = (uint8_t)(hl >> 8);
    cpu.regFile[L] = (uint8_t)(hl & 0xFF);
    return 8;
}

int load_HL_indirect_decrement_A() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    write(hl--, cpu.regFile[A]);
    cpu.regFile[H] = (uint8_t)(hl >> 8);
    cpu.regFile[L] = (uint8_t)(hl & 0xFF);
    return 8;
}

int load_A_HL_indirect_decrement() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    cpu.regFile[A] = read(hl--);
    cpu.regFile[H] = (uint8_t)(hl >> 8);
    cpu.regFile[L] = (uint8_t)(hl & 0xFF);
    return 8;
}

int load_A_HL_indirect_increment() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    cpu.regFile[A] = read(hl++);
    cpu.regFile[H] = (uint8_t)(hl >> 8);
    cpu.regFile[L] = (uint8_t)(hl & 0xFF);
    return 8;
}

int load_SP_d16() {
    uint16_t imm = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    cpu.sp = imm;
    return 12;
}

int load_d16_indirect_SP() {
    uint16_t addr = read(cpu.pc++) + (((uint16_t)read(cpu.pc++))<<8);
    write(addr, (uint8_t)(cpu.sp & 0xFF));
    write(addr + 1, (uint8_t)(cpu.sp >> 8));
    return 20;
}

int load_HL_SP_plus_s8() {
    int8_t imm = (int8_t)read(cpu.pc++);
    uint16_t result = cpu.sp + imm;
    clear_flag(FLAG_ZERO | FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, (cpu.sp & 0xF) > (result & 0xF));
    eval_flag(FLAG_CARRY, (cpu.sp & 0xFF) > (result & 0xFF));
    cpu.regFile[H] = (uint8_t)(result >> 8);
    cpu.regFile[L] = (uint8_t)(result & 0xFF);
    return 12;
}

int load_SP_HL() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    cpu.sp = hl;
    return 8;
}

int no_op() {
    return 4;
}

int or_A_r8(int reg) {
    cpu.regFile[A] |= cpu.regFile[reg];
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY | FLAG_CARRY);
    return 4;
}

int or_A_HL_indirect() {
    uint8_t mem = read((((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L]);
    cpu.regFile[A] |= mem;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY | FLAG_CARRY);
    return 8;
}

int or_A_d8() {
    int8_t imm = (int8_t)read(cpu.pc++);
    cpu.regFile[A] |= imm;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY | FLAG_CARRY);
    return 8;
}

int pop_r16(int regH, int regL) {
    uint16_t reg = pop16();
    cpu.regFile[regH] = (uint8_t)(reg >> 8);
    cpu.regFile[regL] = (uint8_t)(reg & 0xFF);
    if(regL == F) {
        cpu.regFile[F] &= 0xF0;
    }
    return 12;
}

int push_r16(int regH, int regL) {
    uint16_t reg = (((uint16_t)cpu.regFile[regH]) << 8) + cpu.regFile[regL];
    push16(reg);
    return 16;
}

int reset_bit_r8(uint8_t bit, int reg) {
    cpu.regFile[reg] &= ~(1<<bit);
    return 8;
}

int reset_bit_HL_indirect(uint8_t bit) {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    write(hl, mem & ~(1<<bit));
    return 16;
}

int return_from_subroutine() {
    cpu.pc = pop16();
    return 16;
}

int return_cond(uint8_t flag) {
    if(cpu.regFile[F] & flag) {
        cpu.pc = pop16();
        return 20;
    }
    return 8;
}

int return_cond_inv(uint8_t flag) {
    if(!(cpu.regFile[F] & flag)) {
        cpu.pc = pop16();
        return 20;
    }
    return 8;
}

int return_enable_interrupt() {
    set_ime(1 << 1);
    cpu.pc = pop16();
    return 16;
}

int rotate_left_through_carry_r8(int reg) {
    uint8_t carry = (cpu.regFile[F] & FLAG_CARRY) >> 4;
    eval_flag(FLAG_CARRY, cpu.regFile[reg] & (1<<7));
    cpu.regFile[reg] = (cpu.regFile[reg] << 1) | carry;
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    return 8;
}

int rotate_left_through_carry_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    uint8_t carry = (cpu.regFile[F] & FLAG_CARRY) >> 4;
    eval_flag(FLAG_CARRY, mem & (1<<7));
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    write(hl, (mem << 1) | carry);
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 16;
}

int rotate_left_through_carry_A() {
    uint8_t carry = (cpu.regFile[F] & FLAG_CARRY) >> 4;
    eval_flag(FLAG_CARRY, cpu.regFile[A] & (1<<7));
    clear_flag(FLAG_ZERO | FLAG_SUBTRACT | FLAG_HALF_CARRY);
    cpu.regFile[A] = (cpu.regFile[A] << 1) | carry;
    return 4;
}

int rotate_left_r8(int reg) {
    uint8_t msb = (cpu.regFile[reg] & (1<<7)) >> 7;
    eval_flag(FLAG_CARRY, msb);
    cpu.regFile[reg] = (cpu.regFile[reg] << 1) | msb;
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    return 8;
}

int rotate_left_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    uint8_t msb = (mem & (1<<7)) >> 7;
    eval_flag(FLAG_CARRY, msb);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    write(hl, (mem << 1) | msb);
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 16;
}

int rotate_left_A() {
    uint8_t msb = (cpu.regFile[A] & (1<<7)) >> 7;
    eval_flag(FLAG_CARRY, msb);
    clear_flag(FLAG_ZERO | FLAG_SUBTRACT | FLAG_HALF_CARRY);
    cpu.regFile[A] = (cpu.regFile[A] << 1) | msb;
    return 4;
}

int rotate_right_through_carry_r8(int reg) {
    uint8_t carry = ((cpu.regFile[F] & FLAG_CARRY) >> 4) << 7;
    eval_flag(FLAG_CARRY, cpu.regFile[reg] & 1);
    cpu.regFile[reg] = (cpu.regFile[reg] >> 1) | carry;
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    return 8;
}

int rotate_right_through_carry_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    uint8_t carry = ((cpu.regFile[F] & FLAG_CARRY) >> 4) << 7;
    eval_flag(FLAG_CARRY, mem & 1);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    write(hl, (mem >> 1) | carry);
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 16;
}

int rotate_right_through_carry_A() {
    uint8_t carry = ((cpu.regFile[F] & FLAG_CARRY) >> 4) << 7;
    eval_flag(FLAG_CARRY, cpu.regFile[A] & 1);
    cpu.regFile[A] = (cpu.regFile[A] >> 1) | carry;
    clear_flag(FLAG_ZERO | FLAG_SUBTRACT | FLAG_HALF_CARRY);
    return 4;
}

int rotate_right_r8(int reg) {
    uint8_t lsb = (cpu.regFile[reg] & 1) << 7;
    eval_flag(FLAG_CARRY, lsb);
    cpu.regFile[reg] = lsb | (cpu.regFile[reg] >> 1);
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    return 8;
}

int rotate_right_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    uint8_t lsb = (mem & 1) << 7;
    eval_flag(FLAG_CARRY, lsb);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    write(hl, lsb | (mem >> 1));
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 16;
}

int rotate_right_A() {
    uint8_t lsb = (cpu.regFile[A] & 1) << 7;
    eval_flag(FLAG_CARRY, lsb);
    clear_flag(FLAG_ZERO | FLAG_SUBTRACT | FLAG_HALF_CARRY);
    cpu.regFile[A] = lsb | (cpu.regFile[A] >> 1);
    return 4;
}

int reset_vector(uint16_t vec) {
    push16(cpu.pc);
    cpu.pc = vec;
    return 16;
}

int subtract_carry_A_r8(int reg) {
    uint8_t carry = (cpu.regFile[F] & FLAG_CARRY) >> 4;
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) - (cpu.regFile[reg] & 0xF) - carry) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] - cpu.regFile[reg] - carry) & 0x100);
    cpu.regFile[A] -= cpu.regFile[reg] + carry;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 4;
}

int subtract_carry_A_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    uint8_t carry = (cpu.regFile[F] & FLAG_CARRY) >> 4;
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) - (mem & 0xF) - carry) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] - mem - carry) & 0x100);
    cpu.regFile[A] -= mem + carry;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int subtract_carry_A_d8() {
    uint8_t imm = read(cpu.pc++);
    uint8_t carry = (cpu.regFile[F] & FLAG_CARRY) >> 4;
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) - (imm & 0xF) - carry) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] - imm - carry) & 0x100);
    cpu.regFile[A] -= imm + carry;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int set_carry_flag() {
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    set_flag(FLAG_CARRY);
    return 4;
}

int set_bit_r8(uint8_t bit, int reg) {
    cpu.regFile[reg] |= (1 << bit);
    return 8;
}

int set_bit_HL_indirect(uint8_t bit) {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    write(hl, mem | (1 << bit));
    return 16;
}

int shift_left_r8(int reg) {
    eval_flag(FLAG_CARRY, cpu.regFile[reg] & (1<<7));
    cpu.regFile[reg] = cpu.regFile[reg] << 1;
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    return 8;
}

int shift_left_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    eval_flag(FLAG_CARRY, mem & (1 << 7));
    write(hl, mem << 1);
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 16;
}

int shift_right_arithmetic_r8(int reg) {
    uint8_t msb = cpu.regFile[reg] & (1<<7);
    eval_flag(FLAG_CARRY, cpu.regFile[reg] & 1);
    cpu.regFile[reg] = msb | (cpu.regFile[reg] >> 1);
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    return 8;
}

int shift_right_arithmetic_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    uint8_t msb = mem & (1<<7);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    eval_flag(FLAG_CARRY, mem & 1);
    write(hl, msb | (mem >> 1));
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 16;
}

int shift_right_logic_r8(int reg) {
    eval_flag(FLAG_CARRY, cpu.regFile[reg] & 1);
    cpu.regFile[reg] = cpu.regFile[reg] >> 1;
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    return 8;
}

int shift_right_logic_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY);
    eval_flag(FLAG_CARRY, mem & 1);
    write(hl, mem >> 1);
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 16;
}

int stop() {
    // TODO: stop
    return 0;
}

int subtract_A_r8(int reg) {
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) - (cpu.regFile[reg] & 0xF)) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] - cpu.regFile[reg]) & 0x100);
    cpu.regFile[A] -= cpu.regFile[reg];
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 4;
}

int subtract_A_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) - (mem & 0xF)) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] - mem) & 0x100);
    cpu.regFile[A] -= mem;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int subtract_A_d8() {
    uint8_t imm = read(cpu.pc++);
    set_flag(FLAG_SUBTRACT);
    eval_flag(FLAG_HALF_CARRY, ((cpu.regFile[A] & 0xF) - (imm & 0xF)) & 0x10);
    eval_flag(FLAG_CARRY, (cpu.regFile[A] - imm) & 0x100);
    cpu.regFile[A] -= imm;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    return 8;
}

int swap_r8(int reg) {
    cpu.regFile[reg] = (cpu.regFile[reg] >> 4) | (cpu.regFile[reg] << 4);
    eval_flag(FLAG_ZERO, cpu.regFile[reg] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY | FLAG_CARRY);
    return 8;
}

int swap_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY | FLAG_CARRY);
    write(hl, (mem >> 4) | (mem << 4));
    eval_flag(FLAG_ZERO, read(hl) == 0);
    return 16;
}

int xor_A_r8(int reg) {
    cpu.regFile[A] ^= cpu.regFile[reg];
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY | FLAG_CARRY);
    return 4;
}

int xor_A_HL_indirect() {
    uint16_t hl = (((uint16_t)cpu.regFile[H]) << 8) + cpu.regFile[L];
    uint8_t mem = read(hl);
    cpu.regFile[A] ^= mem;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY | FLAG_CARRY);
    return 8;
}

int xor_A_d8() {
    uint8_t imm = read(cpu.pc++);
    cpu.regFile[A] ^= imm;
    eval_flag(FLAG_ZERO, cpu.regFile[A] == 0);
    clear_flag(FLAG_SUBTRACT | FLAG_HALF_CARRY | FLAG_CARRY);
    return 8;
}

int invalid_opcode() {
    return -1;
}

/*
 *  Execute Handlers
*/

int execute_cb() {
    switch(read(cpu.pc++)) {
        case 0x00: return rotate_left_r8(B);
        case 0x01: return rotate_left_r8(C);
        case 0x02: return rotate_left_r8(D);
        case 0x03: return rotate_left_r8(E);
        case 0x04: return rotate_left_r8(H);
        case 0x05: return rotate_left_r8(L);
        case 0x06: return rotate_left_HL_indirect();
        case 0x07: return rotate_left_r8(A);
        case 0x08: return rotate_right_r8(B);
        case 0x09: return rotate_right_r8(C);
        case 0x0A: return rotate_right_r8(D);
        case 0x0B: return rotate_right_r8(E);
        case 0x0C: return rotate_right_r8(H);
        case 0x0D: return rotate_right_r8(L);
        case 0x0E: return rotate_right_HL_indirect();
        case 0x0F: return rotate_right_r8(A);
        case 0x10: return rotate_left_through_carry_r8(B);
        case 0x11: return rotate_left_through_carry_r8(C);
        case 0x12: return rotate_left_through_carry_r8(D);
        case 0x13: return rotate_left_through_carry_r8(E);
        case 0x14: return rotate_left_through_carry_r8(H);
        case 0x15: return rotate_left_through_carry_r8(L);
        case 0x16: return rotate_left_through_carry_HL_indirect();
        case 0x17: return rotate_left_through_carry_r8(A);
        case 0x18: return rotate_right_through_carry_r8(B);
        case 0x19: return rotate_right_through_carry_r8(C);
        case 0x1A: return rotate_right_through_carry_r8(D);
        case 0x1B: return rotate_right_through_carry_r8(E);
        case 0x1C: return rotate_right_through_carry_r8(H);
        case 0x1D: return rotate_right_through_carry_r8(L);
        case 0x1E: return rotate_right_through_carry_HL_indirect();
        case 0x1F: return rotate_right_through_carry_r8(A);
        case 0x20: return shift_left_r8(B);
        case 0x21: return shift_left_r8(C);
        case 0x22: return shift_left_r8(D);
        case 0x23: return shift_left_r8(E);
        case 0x24: return shift_left_r8(H);
        case 0x25: return shift_left_r8(L);
        case 0x26: return shift_left_HL_indirect();
        case 0x27: return shift_left_r8(A);
        case 0x28: return shift_right_arithmetic_r8(B);
        case 0x29: return shift_right_arithmetic_r8(C);
        case 0x2A: return shift_right_arithmetic_r8(D);
        case 0x2B: return shift_right_arithmetic_r8(E);
        case 0x2C: return shift_right_arithmetic_r8(H);
        case 0x2D: return shift_right_arithmetic_r8(L);
        case 0x2E: return shift_right_arithmetic_HL_indirect();
        case 0x2F: return shift_right_arithmetic_r8(A);
        case 0x30: return swap_r8(B);
        case 0x31: return swap_r8(C);
        case 0x32: return swap_r8(D);
        case 0x33: return swap_r8(E);
        case 0x34: return swap_r8(H);
        case 0x35: return swap_r8(L);
        case 0x36: return swap_HL_indirect();
        case 0x37: return swap_r8(A);
        case 0x38: return shift_right_logic_r8(B);
        case 0x39: return shift_right_logic_r8(C);
        case 0x3A: return shift_right_logic_r8(D);
        case 0x3B: return shift_right_logic_r8(E);
        case 0x3C: return shift_right_logic_r8(H);
        case 0x3D: return shift_right_logic_r8(L);
        case 0x3E: return shift_right_logic_HL_indirect();
        case 0x3F: return shift_right_logic_r8(A);
        case 0x40: return bit_r8(0, B);
        case 0x41: return bit_r8(0, C);
        case 0x42: return bit_r8(0, D);
        case 0x43: return bit_r8(0, E);
        case 0x44: return bit_r8(0, H);
        case 0x45: return bit_r8(0, L);
        case 0x46: return bit_HL_indirect(0);
        case 0x47: return bit_r8(0, A);
        case 0x48: return bit_r8(1, B);
        case 0x49: return bit_r8(1, C);
        case 0x4A: return bit_r8(1, D);
        case 0x4B: return bit_r8(1, E);
        case 0x4C: return bit_r8(1, H);
        case 0x4D: return bit_r8(1, L);
        case 0x4E: return bit_HL_indirect(1);
        case 0x4F: return bit_r8(1, A);
        case 0x50: return bit_r8(2, B);
        case 0x51: return bit_r8(2, C);
        case 0x52: return bit_r8(2, D);
        case 0x53: return bit_r8(2, E);
        case 0x54: return bit_r8(2, H);
        case 0x55: return bit_r8(2, L);
        case 0x56: return bit_HL_indirect(2);
        case 0x57: return bit_r8(2, A);
        case 0x58: return bit_r8(3, B);
        case 0x59: return bit_r8(3, C);
        case 0x5A: return bit_r8(3, D);
        case 0x5B: return bit_r8(3, E);
        case 0x5C: return bit_r8(3, H);
        case 0x5D: return bit_r8(3, L);
        case 0x5E: return bit_HL_indirect(3);
        case 0x5F: return bit_r8(3, A);
        case 0x60: return bit_r8(4, B);
        case 0x61: return bit_r8(4, C);
        case 0x62: return bit_r8(4, D);
        case 0x63: return bit_r8(4, E);
        case 0x64: return bit_r8(4, H);
        case 0x65: return bit_r8(4, L);
        case 0x66: return bit_HL_indirect(4);
        case 0x67: return bit_r8(4, A);
        case 0x68: return bit_r8(5, B);
        case 0x69: return bit_r8(5, C);
        case 0x6A: return bit_r8(5, D);
        case 0x6B: return bit_r8(5, E);
        case 0x6C: return bit_r8(5, H);
        case 0x6D: return bit_r8(5, L);
        case 0x6E: return bit_HL_indirect(5);
        case 0x6F: return bit_r8(5, A);
        case 0x70: return bit_r8(6, B);
        case 0x71: return bit_r8(6, C);
        case 0x72: return bit_r8(6, D);
        case 0x73: return bit_r8(6, E);
        case 0x74: return bit_r8(6, H);
        case 0x75: return bit_r8(6, L);
        case 0x76: return bit_HL_indirect(6);
        case 0x77: return bit_r8(6, A);
        case 0x78: return bit_r8(7, B);
        case 0x79: return bit_r8(7, C);
        case 0x7A: return bit_r8(7, D);
        case 0x7B: return bit_r8(7, E);
        case 0x7C: return bit_r8(7, H);
        case 0x7D: return bit_r8(7, L);
        case 0x7E: return bit_HL_indirect(7);
        case 0x7F: return bit_r8(7, A);
        case 0x80: return reset_bit_r8(0, B);
        case 0x81: return reset_bit_r8(0, C);
        case 0x82: return reset_bit_r8(0, D);
        case 0x83: return reset_bit_r8(0, E);
        case 0x84: return reset_bit_r8(0, H);
        case 0x85: return reset_bit_r8(0, L);
        case 0x86: return reset_bit_HL_indirect(0);
        case 0x87: return reset_bit_r8(0, A);
        case 0x88: return reset_bit_r8(1, B);
        case 0x89: return reset_bit_r8(1, C);
        case 0x8A: return reset_bit_r8(1, D);
        case 0x8B: return reset_bit_r8(1, E);
        case 0x8C: return reset_bit_r8(1, H);
        case 0x8D: return reset_bit_r8(1, L);
        case 0x8E: return reset_bit_HL_indirect(1);
        case 0x8F: return reset_bit_r8(1, A);
        case 0x90: return reset_bit_r8(2, B);
        case 0x91: return reset_bit_r8(2, C);
        case 0x92: return reset_bit_r8(2, D);
        case 0x93: return reset_bit_r8(2, E);
        case 0x94: return reset_bit_r8(2, H);
        case 0x95: return reset_bit_r8(2, L);
        case 0x96: return reset_bit_HL_indirect(2);
        case 0x97: return reset_bit_r8(2, A);
        case 0x98: return reset_bit_r8(3, B);
        case 0x99: return reset_bit_r8(3, C);
        case 0x9A: return reset_bit_r8(3, D);
        case 0x9B: return reset_bit_r8(3, E);
        case 0x9C: return reset_bit_r8(3, H);
        case 0x9D: return reset_bit_r8(3, L);
        case 0x9E: return reset_bit_HL_indirect(3);
        case 0x9F: return reset_bit_r8(3, A);
        case 0xA0: return reset_bit_r8(4, B);
        case 0xA1: return reset_bit_r8(4, C);
        case 0xA2: return reset_bit_r8(4, D);
        case 0xA3: return reset_bit_r8(4, E);
        case 0xA4: return reset_bit_r8(4, H);
        case 0xA5: return reset_bit_r8(4, L);
        case 0xA6: return reset_bit_HL_indirect(4);
        case 0xA7: return reset_bit_r8(4, A);
        case 0xA8: return reset_bit_r8(5, B);
        case 0xA9: return reset_bit_r8(5, C);
        case 0xAA: return reset_bit_r8(5, D);
        case 0xAB: return reset_bit_r8(5, E);
        case 0xAC: return reset_bit_r8(5, H);
        case 0xAD: return reset_bit_r8(5, L);
        case 0xAE: return reset_bit_HL_indirect(5);
        case 0xAF: return reset_bit_r8(5, A);
        case 0xB0: return reset_bit_r8(6, B);
        case 0xB1: return reset_bit_r8(6, C);
        case 0xB2: return reset_bit_r8(6, D);
        case 0xB3: return reset_bit_r8(6, E);
        case 0xB4: return reset_bit_r8(6, H);
        case 0xB5: return reset_bit_r8(6, L);
        case 0xB6: return reset_bit_HL_indirect(6);
        case 0xB7: return reset_bit_r8(6, A);
        case 0xB8: return reset_bit_r8(7, B);
        case 0xB9: return reset_bit_r8(7, C);
        case 0xBA: return reset_bit_r8(7, D);
        case 0xBB: return reset_bit_r8(7, E);
        case 0xBC: return reset_bit_r8(7, H);
        case 0xBD: return reset_bit_r8(7, L);
        case 0xBE: return reset_bit_HL_indirect(7);
        case 0xBF: return reset_bit_r8(7, A);
        case 0xC0: return set_bit_r8(0, B);
        case 0xC1: return set_bit_r8(0, C);
        case 0xC2: return set_bit_r8(0, D);
        case 0xC3: return set_bit_r8(0, E);
        case 0xC4: return set_bit_r8(0, H);
        case 0xC5: return set_bit_r8(0, L);
        case 0xC6: return set_bit_HL_indirect(0);
        case 0xC7: return set_bit_r8(0, A);
        case 0xC8: return set_bit_r8(1, B);
        case 0xC9: return set_bit_r8(1, C);
        case 0xCA: return set_bit_r8(1, D);
        case 0xCB: return set_bit_r8(1, E);
        case 0xCC: return set_bit_r8(1, H);
        case 0xCD: return set_bit_r8(1, L);
        case 0xCE: return set_bit_HL_indirect(1);
        case 0xCF: return set_bit_r8(1, A);
        case 0xD0: return set_bit_r8(2, B);
        case 0xD1: return set_bit_r8(2, C);
        case 0xD2: return set_bit_r8(2, D);
        case 0xD3: return set_bit_r8(2, E);
        case 0xD4: return set_bit_r8(2, H);
        case 0xD5: return set_bit_r8(2, L);
        case 0xD6: return set_bit_HL_indirect(2);
        case 0xD7: return set_bit_r8(2, A);
        case 0xD8: return set_bit_r8(3, B);
        case 0xD9: return set_bit_r8(3, C);
        case 0xDA: return set_bit_r8(3, D);
        case 0xDB: return set_bit_r8(3, E);
        case 0xDC: return set_bit_r8(3, H);
        case 0xDD: return set_bit_r8(3, L);
        case 0xDE: return set_bit_HL_indirect(3);
        case 0xDF: return set_bit_r8(3, A);
        case 0xE0: return set_bit_r8(4, B);
        case 0xE1: return set_bit_r8(4, C);
        case 0xE2: return set_bit_r8(4, D);
        case 0xE3: return set_bit_r8(4, E);
        case 0xE4: return set_bit_r8(4, H);
        case 0xE5: return set_bit_r8(4, L);
        case 0xE6: return set_bit_HL_indirect(4);
        case 0xE7: return set_bit_r8(4, A);
        case 0xE8: return set_bit_r8(5, B);
        case 0xE9: return set_bit_r8(5, C);
        case 0xEA: return set_bit_r8(5, D);
        case 0xEB: return set_bit_r8(5, E);
        case 0xEC: return set_bit_r8(5, H);
        case 0xED: return set_bit_r8(5, L);
        case 0xEE: return set_bit_HL_indirect(5);
        case 0xEF: return set_bit_r8(5, A);
        case 0xF0: return set_bit_r8(6, B);
        case 0xF1: return set_bit_r8(6, C);
        case 0xF2: return set_bit_r8(6, D);
        case 0xF3: return set_bit_r8(6, E);
        case 0xF4: return set_bit_r8(6, H);
        case 0xF5: return set_bit_r8(6, L);
        case 0xF6: return set_bit_HL_indirect(6);
        case 0xF7: return set_bit_r8(6, A);
        case 0xF8: return set_bit_r8(7, B);
        case 0xF9: return set_bit_r8(7, C);
        case 0xFA: return set_bit_r8(7, D);
        case 0xFB: return set_bit_r8(7, E);
        case 0xFC: return set_bit_r8(7, H);
        case 0xFD: return set_bit_r8(7, L);
        case 0xFE: return set_bit_HL_indirect(7);
        case 0xFF: return set_bit_r8(7, A);
    }
    return -1;
}

int execute() {
    if(!cpu.halted) { 
        switch(read(cpu.pc++)) {
            case 0x00: return no_op();
            case 0x01: return load_r16_d16(B, C);
            case 0x02: return load_r16_indirect_A(B, C);
            case 0x03: return increment_r16(B, C); 
            case 0x04: return increment_r8(B); 
            case 0x05: return decrement_r8(B); 
            case 0x06: return load_r8_d8(B); 
            case 0x07: return rotate_left_A();
            case 0x08: return load_d16_indirect_SP(); 
            case 0x09: return add_HL_r16(B, C); 
            case 0x0A: return load_A_r16_indirect(B, C); 
            case 0x0B: return decrement_r16(B, C); 
            case 0x0C: return increment_r8(C); 
            case 0x0D: return decrement_r8(C); 
            case 0x0E: return load_r8_d8(C); 
            case 0x0F: return rotate_right_A(); 
            case 0x10: return stop(); 
            case 0x11: return load_r16_d16(D, E); 
            case 0x12: return load_r16_indirect_A(D, E); 
            case 0x13: return increment_r16(D, E); 
            case 0x14: return increment_r8(D); 
            case 0x15: return decrement_r8(D); 
            case 0x16: return load_r8_d8(D); 
            case 0x17: return rotate_left_through_carry_A(); 
            case 0x18: return jump_s8(); 
            case 0x19: return add_HL_r16(D, E); 
            case 0x1A: return load_A_r16_indirect(D, E); 
            case 0x1B: return decrement_r16(D, E); 
            case 0x1C: return increment_r8(E); 
            case 0x1D: return decrement_r8(E); 
            case 0x1E: return load_r8_d8(E); 
            case 0x1F: return rotate_right_through_carry_A(); 
            case 0x20: return jump_cond_inv_s8(FLAG_ZERO);
            case 0x21: return load_r16_d16(H, L); 
            case 0x22: return load_HL_indirect_increment_A(); 
            case 0x23: return increment_r16(H, L); 
            case 0x24: return increment_r8(H); 
            case 0x25: return decrement_r8(H); 
            case 0x26: return load_r8_d8(H); 
            case 0x27: return decimal_adjust_A(); 
            case 0x28: return jump_cond_s8(FLAG_ZERO);
            case 0x29: return add_HL_r16(H, L);
            case 0x2A: return load_A_HL_indirect_increment();
            case 0x2B: return decrement_r16(H, L);
            case 0x2C: return increment_r8(L);
            case 0x2D: return decrement_r8(L);
            case 0x2E: return load_r8_d8(L); 
            case 0x2F: return complement_A();
            case 0x30: return jump_cond_inv_s8(FLAG_CARRY);
            case 0x31: return load_SP_d16();
            case 0x32: return load_HL_indirect_decrement_A();
            case 0x33: return increment_SP();
            case 0x34: return increment_HL_indirect();
            case 0x35: return decrement_HL_indirect();
            case 0x36: return load_HL_indirect_d8();
            case 0x37: return set_carry_flag();
            case 0x38: return jump_cond_s8(FLAG_CARRY);
            case 0x39: return add_HL_SP();
            case 0x3A: return load_A_HL_indirect_decrement();
            case 0x3B: return decrement_SP();
            case 0x3C: return increment_r8(A);
            case 0x3D: return decrement_r8(A);
            case 0x3E: return load_r8_d8(A);
            case 0x3F: return complement_carry_flag();
            case 0x40: return load_r8_r8(B, B);
            case 0x41: return load_r8_r8(B, C);
            case 0x42: return load_r8_r8(B, D);
            case 0x43: return load_r8_r8(B, E);
            case 0x44: return load_r8_r8(B, H);
            case 0x45: return load_r8_r8(B, L);
            case 0x46: return load_r8_HL_indirect(B);
            case 0x47: return load_r8_r8(B, A);
            case 0x48: return load_r8_r8(C, B);
            case 0x49: return load_r8_r8(C, C);
            case 0x4A: return load_r8_r8(C, D);
            case 0x4B: return load_r8_r8(C, E);
            case 0x4C: return load_r8_r8(C, H);
            case 0x4D: return load_r8_r8(C, L);
            case 0x4E: return load_r8_HL_indirect(C);
            case 0x4F: return load_r8_r8(C, A);
            case 0x50: return load_r8_r8(D, B);
            case 0x51: return load_r8_r8(D, C);
            case 0x52: return load_r8_r8(D, D);
            case 0x53: return load_r8_r8(D, E);
            case 0x54: return load_r8_r8(D, H);
            case 0x55: return load_r8_r8(D, L);
            case 0x56: return load_r8_HL_indirect(D);
            case 0x57: return load_r8_r8(D, A);
            case 0x58: return load_r8_r8(E, B);
            case 0x59: return load_r8_r8(E, C);
            case 0x5A: return load_r8_r8(E, D);
            case 0x5B: return load_r8_r8(E, E);
            case 0x5C: return load_r8_r8(E, H);
            case 0x5D: return load_r8_r8(E, L);
            case 0x5E: return load_r8_HL_indirect(E);
            case 0x5F: return load_r8_r8(E, A);
            case 0x60: return load_r8_r8(H, B);
            case 0x61: return load_r8_r8(H, C);
            case 0x62: return load_r8_r8(H, D);
            case 0x63: return load_r8_r8(H, E);
            case 0x64: return load_r8_r8(H, H);
            case 0x65: return load_r8_r8(H, L);
            case 0x66: return load_r8_HL_indirect(H);
            case 0x67: return load_r8_r8(H, A);
            case 0x68: return load_r8_r8(L, B);
            case 0x69: return load_r8_r8(L, C);
            case 0x6A: return load_r8_r8(L, D);
            case 0x6B: return load_r8_r8(L, E);
            case 0x6C: return load_r8_r8(L, H);
            case 0x6D: return load_r8_r8(L, L);
            case 0x6E: return load_r8_HL_indirect(L);
            case 0x6F: return load_r8_r8(L, A);
            case 0x70: return load_HL_indirect_r8(B);
            case 0x71: return load_HL_indirect_r8(C);
            case 0x72: return load_HL_indirect_r8(D);
            case 0x73: return load_HL_indirect_r8(E);
            case 0x74: return load_HL_indirect_r8(H);
            case 0x75: return load_HL_indirect_r8(L);
            case 0x76: return halt();
            case 0x77: return load_HL_indirect_r8(A);
            case 0x78: return load_r8_r8(A, B);
            case 0x79: return load_r8_r8(A, C);
            case 0x7A: return load_r8_r8(A, D);
            case 0x7B: return load_r8_r8(A, E);
            case 0x7C: return load_r8_r8(A, H);
            case 0x7D: return load_r8_r8(A, L);
            case 0x7E: return load_r8_HL_indirect(A);
            case 0x7F: return load_r8_r8(A, A);
            case 0x80: return add_A_r8(B);
            case 0x81: return add_A_r8(C);
            case 0x82: return add_A_r8(D);
            case 0x83: return add_A_r8(E);
            case 0x84: return add_A_r8(H);
            case 0x85: return add_A_r8(L);
            case 0x86: return add_A_HL_indirect();
            case 0x87: return add_A_r8(A);
            case 0x88: return add_carry_A_r8(B);
            case 0x89: return add_carry_A_r8(C);
            case 0x8A: return add_carry_A_r8(D);
            case 0x8B: return add_carry_A_r8(E);
            case 0x8C: return add_carry_A_r8(H);
            case 0x8D: return add_carry_A_r8(L);
            case 0x8E: return add_carry_A_HL_indirect();
            case 0x8F: return add_carry_A_r8(A);
            case 0x90: return subtract_A_r8(B);
            case 0x91: return subtract_A_r8(C);
            case 0x92: return subtract_A_r8(D);
            case 0x93: return subtract_A_r8(E);
            case 0x94: return subtract_A_r8(H);
            case 0x95: return subtract_A_r8(L);
            case 0x96: return subtract_A_HL_indirect();
            case 0x97: return subtract_A_r8(A);
            case 0x98: return subtract_carry_A_r8(B);
            case 0x99: return subtract_carry_A_r8(C);
            case 0x9A: return subtract_carry_A_r8(D);
            case 0x9B: return subtract_carry_A_r8(E);
            case 0x9C: return subtract_carry_A_r8(H);
            case 0x9D: return subtract_carry_A_r8(L);
            case 0x9E: return subtract_carry_A_HL_indirect();
            case 0x9F: return subtract_carry_A_r8(A);
            case 0xA0: return and_A_r8(B);
            case 0xA1: return and_A_r8(C);
            case 0xA2: return and_A_r8(D);
            case 0xA3: return and_A_r8(E);
            case 0xA4: return and_A_r8(H);
            case 0xA5: return and_A_r8(L);
            case 0xA6: return and_A_HL_indirect();
            case 0xA7: return and_A_r8(A);
            case 0xA8: return xor_A_r8(B);
            case 0xA9: return xor_A_r8(C);
            case 0xAA: return xor_A_r8(D);
            case 0xAB: return xor_A_r8(E);
            case 0xAC: return xor_A_r8(H);
            case 0xAD: return xor_A_r8(L);
            case 0xAE: return xor_A_HL_indirect();
            case 0xAF: return xor_A_r8(A);
            case 0xB0: return or_A_r8(B);
            case 0xB1: return or_A_r8(C);
            case 0xB2: return or_A_r8(D);
            case 0xB3: return or_A_r8(E);
            case 0xB4: return or_A_r8(H);
            case 0xB5: return or_A_r8(L);
            case 0xB6: return or_A_HL_indirect();
            case 0xB7: return or_A_r8(A);
            case 0xB8: return compare_A_r8(B);
            case 0xB9: return compare_A_r8(C);
            case 0xBA: return compare_A_r8(D);
            case 0xBB: return compare_A_r8(E);
            case 0xBC: return compare_A_r8(H);
            case 0xBD: return compare_A_r8(L);
            case 0xBE: return compare_A_HL_indirect();
            case 0xBF: return compare_A_r8(A);
            case 0xC0: return return_cond_inv(FLAG_ZERO);
            case 0xC1: return pop_r16(B, C);
            case 0xC2: return jump_cond_inv_d16(FLAG_ZERO);
            case 0xC3: return jump_d16();
            case 0xC4: return call_cond_inv(FLAG_ZERO);
            case 0xC5: return push_r16(B, C);
            case 0xC6: return add_A_d8();
            case 0xC7: return reset_vector(0x00);
            case 0xC8: return return_cond(FLAG_ZERO);
            case 0xC9: return return_from_subroutine();
            case 0xCA: return jump_cond_d16(FLAG_ZERO);
            case 0xCB: return execute_cb();
            case 0xCC: return call_cond(FLAG_ZERO);
            case 0xCD: return call();
            case 0xCE: return add_carry_A_d8();
            case 0xCF: return reset_vector(0x08);
            case 0xD0: return return_cond_inv(FLAG_CARRY);
            case 0xD1: return pop_r16(D, E);
            case 0xD2: return jump_cond_inv_d16(FLAG_CARRY);
            case 0xD3: return invalid_opcode();
            case 0xD4: return call_cond_inv(FLAG_CARRY);
            case 0xD5: return push_r16(D, E);
            case 0xD6: return subtract_A_d8();
            case 0xD7: return reset_vector(0x10);
            case 0xD8: return return_cond(FLAG_CARRY);
            case 0xD9: return return_enable_interrupt();
            case 0xDA: return jump_cond_d16(FLAG_CARRY);
            case 0xDB: return invalid_opcode();
            case 0xDC: return call_cond(FLAG_CARRY);
            case 0xDD: return invalid_opcode();
            case 0xDE: return subtract_carry_A_d8();
            case 0xDF: return reset_vector(0x18);
            case 0xE0: return load_io_d16_indirect_A();
            case 0xE1: return pop_r16(H, L);
            case 0xE2: return load_io_C_indirect_A();
            case 0xE3: return invalid_opcode();
            case 0xE4: return invalid_opcode();
            case 0xE5: return push_r16(H, L);
            case 0xE6: return and_A_d8();
            case 0xE7: return reset_vector(0x20);
            case 0xE8: return add_SP_s8();
            case 0xE9: return jump_HL();
            case 0xEA: return load_d16_indirect_A();
            case 0xEB: return invalid_opcode();
            case 0xEC: return invalid_opcode();
            case 0xED: return invalid_opcode();
            case 0xEE: return xor_A_d8();
            case 0xEF: return reset_vector(0x28);
            case 0xF0: return load_io_A_d16_indirect();
            case 0xF1: return pop_r16(A, F);
            case 0xF2: return load_io_A_C_indirect();
            case 0xF3: return disable_interrupts();
            case 0xF4: return invalid_opcode();
            case 0xF5: return push_r16(A, F);
            case 0xF6: return or_A_d8();
            case 0xF7: return reset_vector(0x30);
            case 0xF8: return load_HL_SP_plus_s8();
            case 0xF9: return load_SP_HL();
            case 0xFA: return load_A_d16_indirect();
            case 0xFB: return enable_interrupts();
            case 0xFC: return invalid_opcode();
            case 0xFD: return invalid_opcode();
            case 0xFE: return compare_A_d8();
            case 0xFF: return reset_vector(0x38);
        }
    }
    else {
        return 4;
    }
    return -1;
}

/*
 *  Public Functions
*/

// Goes through the Gameboy's procedure for setting up an interrupt
// Disables interrupts, pushes the current address to the stack, and
// sets the program counter to the interrupt address
void setup_interrupt(uint16_t int_addr) {
    disable_interrupts();
    push16(cpu.pc);
    cpu.pc = int_addr;
}

void set_halted(int val) {
    cpu.halted = val;
}

uint16_t get_program_counter() {
    return cpu.pc;
}

// Prints the current state of the CPU
void dump_cpu() {
    printf("A: %02X   F: %02X\n", cpu.regFile[A], cpu.regFile[F]);
    printf("B: %02X   C: %02X\n", cpu.regFile[B], cpu.regFile[C]);
    printf("D: %02X   E: %02X\n", cpu.regFile[D], cpu.regFile[E]);
    printf("H: %02X   L: %02X\n", cpu.regFile[H], cpu.regFile[L]);
    printf("PC: %04X\n", cpu.pc);
    printf("SP: %04X\n", cpu.sp);
    printf("\n");
}
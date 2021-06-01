import csv

instruction_list = []
program_instructions = []

#Loads list containing name of instruction and corresponding bitmask in format [["ADD", xxxx0001xx0011..."], ["SUB", "11xx..."], ...] 
with open('RV32I_bitmasks.csv', newline='') as csvfile:
    inread = csv.reader(csvfile, delimiter=',')
    for instruction_code in inread:
        instruction_list.append(instruction_code) # Kinda stupid but coverts types correctly

#Loads all instructions used by the program
with open('bsort_blink.hex', newline='') as programfile:
    inread = programfile.readlines()
    for instruction in inread:   
        if(len(instruction) > 4): #Remove potential empty lines at end of file
            instruction_bits = bin(int(instruction[0:8], 16))[2:].zfill(32)   #Covert from hex to binary 
            program_instructions.append(instruction_bits)

#Finds the name of a given instruction, i.e. takes instruction on form "00010..001", checks against entries in instruction_list and returns the corresponding name e.g. "ADD" if a matching bitmask exists
def findInstruction(instruction, instruction_list):
    if (int(instruction) == 0):
        return "PADDING"

    for bitmask in instruction_list:  #bitmask is confusing name: bitmask[0] contains name of instruction, e.g. "ADD", and bitmask[1] the bitmask "x01x0.."
        mismatch = 0 
        for bitpair in list(zip(bitmask[1],instruction)):
            if( (bitpair[0] != "x") and (bitpair[0] != bitpair[1]) ):
                mismatch = 1
                break
        
        if (mismatch == 0):
            return bitmask[0]
    
    return "NOP"  #Should never happen for a correct program binary

ISA_subset = {}
for instruction in program_instructions:
    instruction_name = findInstruction(instruction, instruction_list)
    if instruction_name in ISA_subset.keys():
        ISA_subset[instruction_name] += 1
    else:
        ISA_subset[instruction_name] = 1

print(ISA_subset.keys())

FEATURES = {
        "ALU_AND": ["AND", "ANDI", "LUI"],             # { feature: [conditions] ... }
        "ALU_OR" : ["OR", "ORI"],
        "ALU_ADD": ["ADD", "AUIPC", "ADDI"],
        "ALU_SUB": ["SUB", "BEQ", "BNE"],               #The branch instruction use result from subtraction
        "ALU_SLT": ["SLT", "SLTU", "SLTI", "SLTIU"],
        "ALU_SRL": ["SRL", "SRLI"],                     #Logical right shift
        "ALU_SRA": ["SRA", "SRAI"],                     #Arithmetic right shift
        "ALU_SLL": ["SLL", "SLLI"],
        "ALU_XOR": ["XOR", "XORI"],
        "ALU_CSRRW": ["CSRRW"],
        "ALU_CSRRS": ["CSRRS"],
        "ALU_CSRRC": ["CSRRC"],
        "ALU_BEQ": ["BEQ"],
        "ALU_BNE": ["BNE"],
        "ALU_BLT": ["BLT"],
        "ALU_BGE": ["BGE"],
        "ALU_BLTU": ["BLTU"],
        "ALU_BGEU": ["BGEU"],
        "CSR_REG": ["CSRRCI", "CSRRSI", "CSRRWI", "CSRRC", "CSRRS", "CSRRW", "CSR"]
}

outfile = open("alu-subset-includes.v", "w")

for feature, conditions in FEATURES.items():
    for condition in conditions:
        if(condition in ISA_subset.keys()):  # If subset contains instruction that requires feature
            outstring = "`define " + str(feature) + " 1\'b1" + "\n"
            outfile.write(outstring)
            break

outfile.close()

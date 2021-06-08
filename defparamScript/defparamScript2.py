import sys

with open(sys.argv[1], 'r') as programfile:
  instructions = programfile.readlines()

instructions = instructions[0:1024]


for i in range(8):
  hexfilename = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/program" + str(8-i) + ".hex"
  hexfile = open(hexfilename, "w")
  
  bitSplit =[]
  for k in range(256):
    bitSplit.append([[ [],[],[],[] ],[ [],[],[],[] ], [ [],[],[],[] ], [ [],[],[],[] ]])

  for idx, instruction in enumerate(instructions):
    place = int(idx/256)
    bitString = bin(int(instruction[i], 16))[2:].zfill(4)
    for j, bit in enumerate(bitString):
        bitSplit[idx % 256][3-j][3-place] = bit
    
  for row in bitSplit:
     outString = ""
     for symbol in row:
        outString =  hex(int("".join(symbol), 2))[2:] + outString
     hexfile.write(outString + "\n")
 

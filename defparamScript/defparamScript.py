import sys

with open(sys.argv[1], 'r') as programfile:
  instructions = programfile.readlines()

instructions = instructions[0:1024]


for i in range(8):
  hexStrings = ["","","","","","","","","","","","","","","",""]
  filename = "/home/students/mec77/team3-f-of-e-tools/verilog/hardware/processor/sail-core/include/mem_cells/mem_cell_" + str(8-i) + "_params.v"
  outfile = open(filename, "w")
  for idx, instruction in enumerate(instructions):
    stringIdx = int(idx/64)
    hexStrings[stringIdx] += instruction[i]
  for idx, hexString in enumerate(hexStrings):
    outfile.write("defparam mem_cell_" + str(8-i) + ".INIT_" + "%X" % idx + " = 256'h" + hexString[::-1] + ";\n")
 

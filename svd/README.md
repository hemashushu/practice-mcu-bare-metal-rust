# About SVD

ARM defines an SVD (System View Description) file format in its CMSIS standard as a means for Cortex-M-based chip manufacturers to provide a common description of peripherals, registers, and register fields.

You can easily view the values of the peripheral registers in GDB using PyCortexMDebug (although you can also view the registers directly by viewing the memory).

## Install

1. install PyCortexMDebug
   download source from [https://github.com/bnahill/PyCortexMDebug], and change into the source directory and run `$ python setup.py install --user`.

2. get the SVD file for your MCU
   download from [https://github.com/posborne/cmsis-svd].

## Load

1. launch GDB (client).
   `$ arm-none-eabi-gdb firmware.elf`
   or
   `$ arm-none-eabi-gdb firmware.elf -ex "your_gdb_init_command" -ex "another_init_command"`
   or
   `$ arm-none-eabi-gdb firmware.elf -x your_gdb_init_commands_file.gdb`

2. load SVD file.
   type commands in GDB:
   `(gdb) source path/to/gdb.py`
   `(gdb) svd_load path/to/*.svd`

## SVD command usage

1. list available peripherals with descriptions
   `(gdb) svd`

2. see all of the registers (with their values) for a given peripheral
   `(gdb) svd peripheral_name`

3. see all of the field values with descriptions
   `(gdb) svd peripheral_name register_name`

format modifiers:

- `svd/x` will display values in hex
- `svd/o` will display values in octal
- `svd/t` or `svd/b` will display values in binary
- `svd/a` will display values in hex and try to resolve symbols from the values

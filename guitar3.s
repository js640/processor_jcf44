DEPTH = 4096;
WIDTH = 32;

ADDRESS_RADIX = DEC;
DATA_RADIX = BIN;

CONTENT
BEGIN
    -- main: nop
0000 : 00000000000000000000000000000000;
    -- noop # 
0001 : 00000000000000000000000000000000;
    -- lw $1, 0($0) # target y coordinate
0002 : 01000000010000000000000000000000;
    -- moveBlock1: nop
0003 : 00000000000000000000000000000000;
    -- addi $1, $1, 1 #
0004 : 00101000010000100000000000000001;
    -- noop # 
0005 : 00000000000000000000000000000000;
    -- noop # 
0006 : 00000000000000000000000000000000;
    -- noop # 
0007 : 00000000000000000000000000000000;
    -- noop #
0008 : 00000000000000000000000000000000;
    -- jal moveBlock1
0009 : 00011moveBlock1;
[0010 .. 4095] : 00000000000000000000000000000000;
END;

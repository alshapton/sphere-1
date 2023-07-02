
    .CR     MINIMAL
    .TF     test.min,min,16

VALUE = 0x1234
DEST = 0xABCD
BYTE = 0x56

    NOP
    BNK
    OUT
    CLC
    SEC
    LSL
    ROL
    LSR
    ROR
    ASR
    INP
    NEG
    INC
    DEC

    PHS
    PLS
    RTS

    LDI     VALUE
    ADI     #VALUE
    SBI     /VALUE
    CPI     <VALUE
    ACI     >VALUE
    SCI     VALUE

    JPA     DEST
    LDA     DEST
    STA     DEST
    ADA     DEST
    SBA     DEST
    CPA     DEST
    ACA     DEST
    SCA     DEST

    CLB     DEST
    NEB     DEST
    INB     DEST
    DEB     DEST
    ADB     DEST
    SBB     DEST
    ACB     DEST
    SCB     DEST
    CLW     DEST
    NEW     DEST
    INW     DEST
    DEW     DEST
    ADW     DEST
    SBW     DEST
    ACW     DEST
    SCW     DEST

    JPS     DEST
    BNE     DEST
    BEQ     DEST
    BCC     DEST
    BCS     DEST
    BPL     DEST
    BMI     DEST

    JPR     DEST
    LDR     DEST
    STR     DEST
    ADR     DEST
    SBR     DEST
    CPR     DEST
    ACR     DEST
    SCR     DEST

    LDS     BYTE
    STS     BYTE        nop nop nop

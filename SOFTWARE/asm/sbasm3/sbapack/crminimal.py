# ------------------------------------------------------------------------------
#
#   crminimal.py
#
#   Package module file for the SB-Assembler sbasm
#   See www.sbprojects.net for details
#
#   Author: San Bergmans
#   Date  : 2022-04-17
#
#   8085 Cross Overlay
#
# ------------------------------------------------------------------------------

import sys
import os

import assem
import dec
import errors
import target

crossversion = '3.00.00'
minversion = '3.03.06'


# ------------------------------------------------------------------------------

def Help():

    print('Tell a little story about this cross overlay')


# ------------------------------------------------------------------------------

def CrossInit():

    global Asm, Flags

    assem.CheckVersions(crossversion, minversion)

    # The instructions directory contains a tuple with:
    #   function which handles this opcode,
    #   integer value of opcode
    #   string with cycle time(s) of this instruction on an 8085
    #   string with cycle time(s) of this instruction on an 8080

    dec.Asm.Instructions = {

        'ASR' : (Inherent, 0x09, '15'),
        'BNK' : (Inherent, 0x01, '4'),
        'CLC' : (Inherent, 0x03, '5'),
        'DEC' : (Inherent, 0x0D, '5'),
        'INC' : (Inherent, 0x0C, '5'),
        'INP' : (Inherent, 0x0A, '6'),
        'LSL' : (Inherent, 0x05, '5'),
        'LSR' : (Inherent, 0x07, '13'),
        'NEG' : (Inherent, 0x0B, '6'),
        'NOP' : (Inherent, 0x00, '16'),
        'OUT' : (Inherent, 0x02, '4'),
        'PHS' : (Inherent, 0x36, '12'),
        'PLS' : (Inherent, 0x37, '10'),
        'ROL' : (Inherent, 0x06, '5'),
        'ROR' : (Inherent, 0x08, '12'),
        'RTS' : (Inherent, 0x39, '14'),
        'SEC' : (Inherent, 0x04, '5'),

        'ACI' : (Immediate, 0x12, '5'),
        'ADI' : (Immediate, 0x0F, '5'),
        'CPI' : (Immediate, 0x11, '5'),
        'LDI' : (Immediate, 0x0E, '4'),
        'SBI' : (Immediate, 0x10, '5'),
        'SCI' : (Immediate, 0x13, '5'),

        'ACA' : (Absolute, 0x1A, '8'),
        'ADA' : (Absolute, 0x17, '8'),
        'CPA' : (Absolute, 0x19, '8'),
        'JPA' : (Absolute, 0x14, '6'),
        'LDA' : (Absolute, 0x15, '7'),
        'SBA' : (Absolute, 0x18, '8'),
        'SCA' : (Absolute, 0x1B, '8'),
        'STA' : (Absolute, 0x16, '8'),

        'ACB' : (Absolute, 0x2A, '11'),
        'ACW' : (Absolute, 0x32, '12'),
        'ADB' : (Absolute, 0x28, '9'),
        'ADW' : (Absolute, 0x30, '11'),
        'CLB' : (Absolute, 0x24, '8'),
        'CLW' : (Absolute, 0x2C, '10'),
        'DEB' : (Absolute, 0x27, '10'),
        'DEW' : (Absolute, 0x2F, '12'),
        'INB' : (Absolute, 0x26, '10'),
        'INW' : (Absolute, 0x2E, '12'),
        'NEB' : (Absolute, 0x25, '10'),
        'NEW' : (Absolute, 0x2D, '12'),
        'SBB' : (Absolute, 0x29, '10'),
        'SBW' : (Absolute, 0x31, '12'),
        'SCB' : (Absolute, 0x2B, '11'),
        'SCW' : (Absolute, 0x33, '13'),

        'BCC' : (Absolute, 0x3C, '5/6'),
        'BCS' : (Absolute, 0x3D, '5/6'),
        'BEQ' : (Absolute, 0x3B, '5/6'),
        'BNE' : (Absolute, 0x3A, '5/6'),
        'BMI' : (Absolute, 0x3F, '5/6'),
        'BPL' : (Absolute, 0x3E, '5/6'),
        'JPS' : (Absolute, 0x38, '16'),

#       Relative addresses can alsoe be interpreted as Absolute addresses
#       The effective address is stored at the Absolute address

        'JPR' : (Absolute, 0x1C, '9'),
        'LDR' : (Absolute, 0x1D, '10'),
        'STR' : (Absolute, 0x1E, '10'),
        'ADR' : (Absolute, 0x1F, '11'),
        'SBR' : (Absolute, 0x20, '11'),
        'CPR' : (Absolute, 0x21, '11'),
        'ACR' : (Absolute, 0x22, '11'),
        'SCR' : (Absolute, 0x23, '11'),

#       Stack pointer + index can be handled like Immediate addressing mode

        'LDS' : (Immediate, 0x34, '9'),
        'STS' : (Immediate, 0x35, '16')
        }

    dec.Asm.Timing_Length = 3

    dec.Asm.Memory = 0
    if dec.Asm.Pass == 1:
        sys.stdout.write('Loaded ' + dec.Cross.Name[2:] + ' overlay version ' +
                         crossversion + dec.EOL)

    dec.Asm.Max_Address = dec.MAX16
    dec.Asm.PP_TA_Factor = 1
    dec.Flags.BigEndian = False

    return


# ------------------------------------------------------------------------------

def CrossDirective():

    # This cross overlay has no extra/changed directives

    return False    # We didn't handle any directives


# ------------------------------------------------------------------------------

def CrossCleanUp():

    # This cross overlay does not need any clean up

    return


# ------------------------------------------------------------------------------

def CrossMnemonic():

    """
    Entry point for parsing the line
    The minimal CPU assembler allows multiple instructions on one line.
    The first instruction is mandatory, the rest are optional.
    If the next word is a legal mnemonic the assembly continues.
    If the next word is not a legal mnemonic it is treated as a comment.
    If a previous instruction on the line contains an error the rest of the
     line is no longer assembled.
    """

    global Asm

    if dec.Asm.Mnemonic in dec.Asm.Instructions:
        func = dec.Asm.Instructions[dec.Asm.Mnemonic][0]
        func()
    else:
        errors.DoError('badopco', False)

    while dec.Flags.ErrorInLine == 0:
        # keep looking for more mnemonics unless an error is found previously

        if assem.FindNextNonSpace() > 0:
            # We're not at the end of the line. Next word can be an instruction
            dec.Asm.Parse_Pointer -= 1
            dec.Asm.Mnemonic = assem.GetMnemonic()

            if dec.Asm.Mnemonic in dec.Asm.Instructions:
                # An other mnemonic is found

                # Print all generated bytes and start a new line
                addrlen = len(hex(dec.Asm.BOL_Address))
                if addrlen < 7:
                    # Minimum address length = 4 digits (0x is also counted!)
                    addrlen = 4
                elif addrlen == 7 or addrlen == 8:
                    # Now the address lenght is 6 digits
                    addrlen = 6
                else:
                    # No, the address lenght is 8 digits
                    addrlen = 8
                dec.Asm.List_Line = dec.Asm.List_Line + dec.EOL + " "\
                    * addrlen + " "
                dec.Asm.List_Byte_Cnt = 0

                func = dec.Asm.Instructions[dec.Asm.Mnemonic][0]
                func()
            else:
                # It was not a mnemonic, treat it as a comment now
                break
        else:
            # We're at the end of the program line
            break


# -----------------------------------------------------------------------------

def MissingOperand():

    if dec.Asm.Parse_Pointer == 0:
        errors.DoError('missoper', False)
        return True
    else:
        return False


# -----------------------------------------------------------------------------


def NoMore():

    """
    No more parameters are expected.
    """

    if assem.MoreParameters():
        errors.DoWarning('extrign', False)


# -----------------------------------------------------------------------------

def Inherent():

    """
    No operands required
    """

    global Asm

    dec.Asm.Timing = dec.Asm.Instructions[dec.Asm.Mnemonic][2]
    target.CodeByte(dec.Asm.Instructions[dec.Asm.Mnemonic][1])


# -----------------------------------------------------------------------------

def Immediate():

    """
    Immediate addressing mode. Mnemonic is followed by one byte of data
    """

    global Asm

    if MissingOperand():
        return

    prefix = assem.NowChar()

    if prefix in '#/=\\<>':
        prefix = assem.NowChar(True)
    else:
        prefix = '#'

    value = assem.EvalExpr()

    if prefix == '#' or prefix == '<':
        byte = value[0]
    elif prefix == '/' or prefix == '>':
        byte = value[0] >> 8
    elif prefix == '=':
        byte = value[0] >> 16
    else:
        byte = value[0] >> 24

    dec.Asm.Timing = dec.Asm.Instructions[dec.Asm.Mnemonic][2]
    target.CodeByte(dec.Asm.Instructions[dec.Asm.Mnemonic][1])
    target.CodeByte(byte)

    NoMore()


# -----------------------------------------------------------------------------

def Absolute():

    global Asm

    if MissingOperand():
        return

    value = assem.EvalExpr()

    if dec.Asm.Pass == 2 or (not value[1]):
        # Test range only if in pass 2, or pass 1 if not forward referenced
        if value[0] > dec.Asm.Max_Address or value[0] < 0:
            # It's a range error, simply ignore everything which doesn't fit
            errors.DoError('range', False)

    dec.Asm.Timing = dec.Asm.Instructions[dec.Asm.Mnemonic][2]
    target.CodeByte(dec.Asm.Instructions[dec.Asm.Mnemonic][1])
    target.CodeWord(value[0])

    NoMore()


# -----------------------------------------------------------------------------

if __name__ == '__main__':
    print("")
    print("This is a python module, it's not a program.")
    print("This module is part of the sbasm package.")
    print("Run sbasm instead.")
    print("")

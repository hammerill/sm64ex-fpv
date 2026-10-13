#!/usr/bin/env python3
# Minimal stand-in for `objcopy -j SECTION IN -O binary OUT`, for hosts without
# GNU binutils (macOS). Handles 32/64-bit ELF of either endianness.
import struct
import sys


def usage():
    sys.exit("usage: elf_objcopy.py -j SECTION IN -O binary OUT")


def main(argv):
    section = None
    files = []
    i = 0
    while i < len(argv):
        if argv[i] == "-j" and i + 1 < len(argv):
            section = argv[i + 1]
            i += 2
        elif argv[i] == "-O" and i + 1 < len(argv):
            if argv[i + 1] != "binary":
                usage()
            i += 2
        else:
            files.append(argv[i])
            i += 1
    if section is None or len(files) != 2:
        usage()

    with open(files[0], "rb") as f:
        elf = f.read()
    if elf[:4] != b"\x7fELF":
        sys.exit(f"{files[0]}: not an ELF file")
    is64 = elf[4] == 2
    end = "<" if elf[5] == 1 else ">"

    if is64:
        shoff, = struct.unpack_from(end + "Q", elf, 0x28)
        shentsize, shnum, shstrndx = struct.unpack_from(end + "HHH", elf, 0x3A)
        sh_fmt = end + "IIQQQQ"
    else:
        shoff, = struct.unpack_from(end + "I", elf, 0x20)
        shentsize, shnum, shstrndx = struct.unpack_from(end + "HHH", elf, 0x2E)
        sh_fmt = end + "IIIIII"

    # name, type, flags, addr, offset, size
    headers = [struct.unpack_from(sh_fmt, elf, shoff + n * shentsize) for n in range(shnum)]
    strtab_off = headers[shstrndx][4]

    for name_off, sh_type, _, _, offset, size in headers:
        name = elf[strtab_off + name_off:elf.index(b"\0", strtab_off + name_off)].decode()
        if name == section:
            # SHT_NOBITS (.bss) has no file contents
            data = bytes(size) if sh_type == 8 else elf[offset:offset + size]
            with open(files[1], "wb") as f:
                f.write(data)
            return
    sys.exit(f"{files[0]}: no section {section}")


if __name__ == "__main__":
    main(sys.argv[1:])


def braille_char(dots):
    code = 0x2800
    for d in dots:
        code += 1 << (d - 1)
    return chr(code)
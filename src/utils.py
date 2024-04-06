def parse_int(data):
    """Parses the first contiguous integer from data"""
    
    result_chars = []
    for c in data:
        if c.isdigit() or c == "-":
            result_chars.append(c)
        else:
            if result_chars:
                break
    if result_chars:
        return int("".join(result_chars))
    else:
        return None

from scipy import stats
import unicodedata

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

def estimate_linear_regression_coefficients(points):
    """Return the coefficients (a and b) for a y=a+bx simple linear regression, given a list points (x,y tuples)"""
    x_data = [p[0] for p in points]
    y_data = [p[1] for p in points]
    slope, intercept, r, p, std_err = stats.linregress(x_data, y_data)
    
    return (intercept, slope)
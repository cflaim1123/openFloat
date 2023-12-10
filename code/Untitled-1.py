import numpy as np 
import sigfig as sf

vol = {0.003: 2}
mass = '15' # kg

print(list(vol.keys())[0])
print(sf.round(list(vol.keys())[0], sigfigs=vol[list(vol.keys())[0]]))


def count_sig_figs(digits):
    '''Return the number of significant figures of the input digit string'''

    integral, _, fractional = digits.partition(".")
    print("integral: ", integral)
    print("fractional: ", fractional)

    if fractional:
        if not integral:
            return len((integral + fractional).lstrip('0'))
        else:
            return len((integral + fractional).lstrip('0'))
    else:
        return len(integral.strip('0'))
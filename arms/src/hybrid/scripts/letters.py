# origin: power button/touch id
ORIGIN_X = -243.60 + 204.18592  # mm
ORIGIN_Y = 295.76 + 37.08883  # mm
# ORIGIN_Z = 288.27  # mm
ORIGIN_Z = 263.27  # mm

# coords measured from center of power button, pos x is left, pos y is down
ROW1_Y = -18.74441  # mm
ROW2_Y = -37.08883  # mm
ROW3_Y = -55.83324  # mm
ROW4_Y = -74.67766  # mm
ROW5_Y = -93.72207  # mm

LETTERS_ROW1_RELATIVE: dict[str, float] = {
    "1": -237.55146,
    "2": -218.54692,
    "3": -199.54238,
    "4": -180.53784,  #
    "5": -161.53330,
    "6": -142.52876,
    "7": -123.52422,
    "8": -104.51968,
    "9": -85.51514,
    "0": -66.51060,
    "-": -47.50606,
    "=": -28.50151,
    "`": -256.55600,
}

LETTERS_ROW2_RELATIVE: dict[str, float] = {
    "e": -190.12031,
    "i": -95.06016,  #
    "o": -76.04813,
    "p": -57.03609,
    "q": -228.14438,
    "r": -171.10828,
    "t": -152.09625,
    "u": -114.07219,
    "w": -209.13235,
    "y": -133.08422,
    "[": -38.02406,
    "]": -19.01203,
    "\\": -0.0,
    # "tab": -249.37434,
}

LETTERS_ROW3_RELATIVE: dict[str, float] = {
    "a": -223.19795,
    "d": -185.17389,
    "f": -166.16186,
    "g": -147.14983,
    "h": -128.13779,  #
    "j": -109.12576,
    "k": -90.11373,
    "l": -71.10170,
    "s": -204.18592,  #
    ";": -52.08967,
    "'": -33.07764,
    "caps": -249.37434,
}

LETTERS_ROW4_RELATIVE: dict[str, float] = {
    "b": -137.78355,
    "c": -175.80762,
    "m": -99.75949,
    "n": -118.77152,  #
    "v": -156.79558,
    "x": -194.81965,
    "z": -213.83168,
    ",": -80.74746,
    ".": -61.73543,
    "/": -42.72339,
}

LETTERS_ROW5_RELATIVE: dict[str, float] = {
    " ": -137.67488,  #
}

LETTER_POSITIONS: dict[str, tuple[float, float, float]] = (
    {
        letter: (ORIGIN_X + x_rel, ORIGIN_Y + ROW1_Y, ORIGIN_Z)
        for letter, x_rel in LETTERS_ROW1_RELATIVE.items()
    }
    | {
        letter: (ORIGIN_X + x_rel, ORIGIN_Y + ROW2_Y, ORIGIN_Z)
        for letter, x_rel in LETTERS_ROW2_RELATIVE.items()
    }
    | {
        letter: (ORIGIN_X + x_rel, ORIGIN_Y + ROW3_Y, ORIGIN_Z)
        for letter, x_rel in LETTERS_ROW3_RELATIVE.items()
    }
    | {
        letter: (ORIGIN_X + x_rel, ORIGIN_Y + ROW4_Y, ORIGIN_Z)
        for letter, x_rel in LETTERS_ROW4_RELATIVE.items()
    }
    | {
        letter: (ORIGIN_X + x_rel, ORIGIN_Y + ROW5_Y, ORIGIN_Z)
        for letter, x_rel in LETTERS_ROW5_RELATIVE.items()
    }
)

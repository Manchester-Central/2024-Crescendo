from itertools import combinations
from pathplannerutil import createAutoPath, getAllPoints

all_pairs = list(combinations(getAllPoints(), 2))
print(all_pairs)

for [a, b] in all_pairs:
    createAutoPath("Template", "Templates", a, b)
    createAutoPath("Template", "Templates", b, a)
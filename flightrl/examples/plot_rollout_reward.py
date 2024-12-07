# Example format
'''
-2.50034
0.000966976
0.000962631
0.000949998
0.000921434
0.000865973
0.000768403
0.00060791
0.00150582
0.00132122
-8.56747e-05
0.000547133
0.00126729
-0.00026484
0.000327139
-0.000125809
-0.000610848
-0.0011216
-0.000396317
-0.00191314
-0.00115419
-0.00136706
-0.00154263
-0.00167645
-0.00176861
-0.000492131
-0.00147925
-9.17892e-05
0.000339994
0.000811638
-2.27338e-05
0.00148465
0.00197513
0.00378265
0.00316493
0.00343838
0.00357299
0.00491688
0.00373239
0.00336935
0.00420619
0.00255628
0.00180861
0.00237249
0.00061005
-8.60766e-05
0.000689293
-0.000720149
0.00039845
-0.000663411
0.000772615
0.00127884
0.00176921
0.00214701
0.00374984
0.00270029
0.00383844
0.00236898
0.00321904
0.00163455
EOR
-2.50041
0.000966571
0.00095991
0.00094007
0.000896427
0.00081619
0.000685604
0.000491688
0.00133838
1.88184e-05
0.000735833
-0.000720281
-0.000118077
-0.000571263
-0.0010707
-0.00160883
-0.00218067
-0.00278474
-0.00222202
-0.0026786
-0.00314139
-0.00360782
-0.00409008
-0.00336064
-0.00491035
-0.00425596
-0.00338454
-0.00487771
-0.00418656
-0.00592191
-0.00547297
-0.00615361
-0.00560137
-0.00747579
-0.00707144
-0.00902673
-0.00860028
-0.00907858
-0.0107546
-0.00985316
-0.00969751
-0.00925911
-0.0085317
-0.0075347
-0.00631407
-0.00494281
-0.00351975
-0.00214947
-0.000899842
0.000239778
0.00133888
0.00243573
0.00346487
0.00425874
0.00327645
0.00421374
0.00371234
0.00135359
0.000707055
-0.00318302
-0.00556182
EOR
'''

# Plot the reward of each rollout, to do this sum the rewards between EOR tokens

import seaborn as sns
import pandas as pd
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from collections import defaultdict

sns.set_style("whitegrid")

if __name__ == "__main__":

    data = defaultdict(int)

    with open("rewards.csv", "r") as f:
        current_rollout = 0
        for i, line in enumerate(f):
            if "EOR" in line:
                current_rollout += 1
                continue
            data[current_rollout] += float(line)
        
    # Print the rewards nicely
    for key, value in data.items():
        print(f"Rollout {key}: {value}")


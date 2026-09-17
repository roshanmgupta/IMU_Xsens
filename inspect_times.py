from compare_imus import read_time_and_col
from pathlib import Path
import numpy as np

for fn in ('0kgtrial4.csv','0kg_movella_test1.csv'):
    t,v = read_time_and_col(Path(fn))
    print('File:',fn)
    print('len t:',len(t))
    print('first 10 t:', np.round(t[:10],6))
    print('first 10 vals:', np.round(v[:10],6))
    print('min t, max t:', np.min(t), np.max(t))
    print('---')

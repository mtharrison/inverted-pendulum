import numpy as np
import json

with open('./metrics.json', 'r') as f:
    data = json.load(f)
    
    keys = list(data.keys())
    
    for key in keys:
        values = np.array(data[key])
        print(f'{key}: Mean: {values.mean() * 1000:.2f}ms, Std: {values.std() * 1000:.2f}ms len: {len(values)}')

# df = pd.read_json('./metrics.json')

# print(df)
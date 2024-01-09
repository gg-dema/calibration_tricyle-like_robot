"""
to use only 1 times: the original dataset use different space between different lines
(example: model_pose: x  y z ) ----> this create bug in my code, so i just re-create the dataset

"""

source_file = open('data/non_formatted_dataset.txt')
destination_file = open('data/dataset.txt', 'w')

lines = source_file.read().splitlines()
for line in lines: 
    
    if line.startswith("#"): 
        destination_file.write(line+'\n')
        continue
    tokens = line.split(" ")
    tokens = [token for token in tokens if token!=""]
    destination_file.write(' '.join(tokens)+'\n')
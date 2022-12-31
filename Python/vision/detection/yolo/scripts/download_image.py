import os
import argparse
import shutil
from datetime import timezone,datetime

from bing_image_downloader import downloader

############################################ Arguments ###########################################

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--images", help="Path to folder that save the images.", type=str, nargs="?")
parser.add_argument("-n", "--num", help="Number of images to capture.", type=int, nargs="?")
parser.add_argument("-q", "--query", help="Path to the text file containing the query", type=str, nargs="?")
args = parser.parse_args()

folder = args.images or ''
num = args.num or 0
query_path = args.query or ''

############################################ Check Path Existance ###########################################

flag = True
if not os.path.exists(folder) :
    print('Invalide images path')
    flag = False

if not num : 
    print('Precise number of images to save.')
    flag = False

if not os.path.exists(query_path) :
    print('Invalide queries path')
    flag = False
else :
    with open(query_path,'r') as f:
        search_queries = f.read()
        search_queries = search_queries.split('\n')
        if not search_queries[-1]:
            search_queries = search_queries[:-1]

############################################ Download ###########################################

if flag: 
    
    # query
    for query in search_queries:
        downloader.download(query, limit=num,  output_dir=folder, 
        adult_filter_off=True, force_replace=False, timeout=60)

        for image in os.listdir(os.path.join(folder,query)):
            (dt, micro) = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S.%f').split('.')
            src_path = os.path.join(folder,query,image)
            dst_path = os.path.join(folder,f'{dt}_{micro}.jpg')
            shutil.move(src_path,dst_path)
        shutil.rmtree(os.path.join(folder,query))


import os
import shutil
import glob
import xml.etree.ElementTree as et
from random import shuffle
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('xml_dir', help='directory containing the VOC XML annotations (labels)')

args = parser.parse_args()



INVALID_LABEL = "tl_none"

def valid_xml_files():
	"""
	Generator filtering out the files with invalid label 'tl_none', 
	which denotes transitory traffic light states. 
	"""
	for xml_file in glob.glob(os.path.join(args.xml_dir, '*.xml')):
		print(xml_file)
		if et.parse(xml_file).getroot().find('object/name').text != INVALID_LABEL:
			yield xml_file

def class_stats(xml_list):
	label_count = {'tl_red': 0, 'tl_yellow': 0, 'tl_green': 0}
	for xml in xml_list:
		label = et.parse(xml).getroot().find('object/name').text
		label_count[label] += 1
	return label_count


# plot class statistics

label_count = class_stats(valid_xml_files())
xt = [0, 1, 2]
plt.bar(xt, list(label_count.values()), color=['red', 'yellow', 'green'])
plt.xticks(xt, list(label_count.keys()))
plt.show()


# valid_xml_list = shuffle([valid_xml_files])
# # create directories for train/test sets
# os.mkdir('./train')
# os.mkdir('./test')

# for xml_file in valid_xml_files:
# 	shutils.copy('./', '')
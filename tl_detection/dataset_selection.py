import os
import shutil
import glob
import xml.etree.ElementTree as et
from random import shuffle
import matplotlib.pyplot as plt
import argparse
from tqdm import tqdm

parser = argparse.ArgumentParser()
parser.add_argument('source_dir', help='directory containing image files and corresponding '
									   'annotations (labels) in VOC XML format')
parser.add_argument('target_dir', help='directory where the train and test subdirectories '
	                                   'will be placed')
parser.add_argument('--balance-classes', help='creates data set with balanced classes',
										 action='store_true')

args = parser.parse_args()



INVALID_LABEL = "tl_none"
RED = "tl_red"
YELLOW = "tl_yellow"
GREEN = "tl_green"
TEST_PERCENT = 10

def valid_xml_files():
	"""
	List of valid files, not containing `tl_none` label.
	"""
	valid = []
	for xml_file in glob.glob(os.path.join(args.source_dir, '*.xml')):
		if et.parse(xml_file).getroot().find('object/name').text != INVALID_LABEL:
			valid.append(os.path.basename(xml_file))
	return valid

def ryg_split(xml_list):
	red_list, yellow_list, green_list = [], [], []
	for xml in xml_list:
		src_path = os.path.join(args.source_dir, xml)
		label = et.parse(src_path).getroot().find('object/name').text
		if label == RED:
			red_list.append(xml)
		if label == YELLOW:
			yellow_list.append(xml)
		if label == GREEN:
			green_list.append(xml)
	return red_list, yellow_list, green_list

def class_stats():
	label_count = {'tl_red': 0, 'tl_yellow': 0, 'tl_green': 0}
	for xml in xml_list:
		label = et.parse(xml).getroot().find('object/name').text
		label_count[label] += 1
	return label_count

def create_train_test_dirs(train_list, test_list):
	# create directories for train/test sets
	train_path = os.path.join(args.target_dir, 'train')
	test_path = os.path.join(args.target_dir, 'test')

	print()
	print("Creating train set in: {}".format(train_path))
	os.mkdir(train_path)
	for filename in tqdm(train_list):
		src_path = os.path.join(args.source_dir, filename)
		tgt_path = os.path.join(train_path, filename)
		shutil.copy(src_path + '.jpg', tgt_path + '.jpg')
		shutil.copy(src_path + '.xml', tgt_path + '.xml')
	
	print()
	print("Creating test set in: {}".format(test_path))
	os.mkdir(test_path)
	for filename in tqdm(test_list):
		src_path = os.path.join(args.source_dir, filename)
		tgt_path = os.path.join(test_path, filename)
		shutil.copy(src_path + '.jpg', tgt_path + '.jpg')
		shutil.copy(src_path + '.xml', tgt_path + '.xml')


# # plot class statistics
# label_count = class_stats(valid_xml_files())
# xt = [0, 1, 2]
# plt.bar(xt, list(label_count.values()), color=['red', 'yellow', 'green'])
# plt.xticks(xt, list(label_count.keys()))
# plt.show()


if args.balance_classes:
	xml_files = valid_xml_files()
	shuffle(xml_files)
	red_files, yellow_files, green_files = ryg_split(xml_files)
	last_ind = min([len(red_files), len(yellow_files), len(green_files)]) - 1
	
	data_list = red_files[:last_ind] + yellow_files[:last_ind] + green_files[:last_ind]
	shuffle(data_list)
	test_ind = len(data_list) // TEST_PERCENT
	test_list = [fn.split('.')[0] for fn in data_list[:test_ind]]
	train_list = [fn.split('.')[0] for fn in data_list[test_ind:]]
else:
	shuffled_xml_list = valid_xml_files()
	shuffle(shuffled_xml_list)
	test_ind = len(shuffled_xml_list) // TEST_PERCENT
	test_list = [fn.split('.')[0] for fn in shuffled_xml_list[:test_ind]]
	train_list = [fn.split('.')[0] for fn in shuffled_xml_list[test_ind:]]

create_train_test_dirs(train_list, test_list)
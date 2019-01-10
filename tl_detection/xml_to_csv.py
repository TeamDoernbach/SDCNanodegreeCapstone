import os
import glob
import pandas as pd
import xml.etree.ElementTree as ET
import argparse

# modified: https://github.com/datitran/raccoon_dataset/blob/master/xml_to_csv.py


parser = argparse.ArgumentParser()
parser.add_argument('source_dir', help='directory containing image files and corresponding '
                                       'annotations (labels) in VOC XML format')
parser.add_argument('csv_filename', help='name of the CSV file to be created')
args = parser.parse_args()

def xml_to_csv(path):
    xml_list = []
    for xml_file in glob.glob(os.path.join(path, '*.xml')):
        root = ET.parse(xml_file).getroot()
        for member in root.findall('object'):  # for all bounding boxes (objects)
            value = (
                root.find('filename').text,
                int(root.find('size')[0].text),  # image width and height
                int(root.find('size')[1].text),
                member[0].text,  # image label
                int(member[4][0].text),  # bbox coordinates (xmin, ymin, xmax, ymax)
                int(member[4][1].text),
                int(member[4][2].text),
                int(member[4][3].text)
            )
            xml_list.append(value)
    column_name = ['filename', 'width', 'height', 'class', 'xmin', 'ymin', 'xmax', 'ymax']
    xml_df = pd.DataFrame(xml_list, columns=column_name)
    return xml_df


if __name__ == "__main__":
    xml_df = xml_to_csv(args.source_dir)
    xml_df.to_csv(args.csv_filename, index=None)
    print('Successfully converted xml to csv.')
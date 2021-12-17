#! /usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import shutil

import argparse
from collections import namedtuple

import pandas as pd


Config = namedtuple('Config', ['input'], verbose=False)


def get_arguments():
    """ 
    Get command-line arguments
    """
    # init parser:
    parser = argparse.ArgumentParser("Generate 100Hz OXTS measurements for LIO/VIO development.")

    # add required and optional groups:
    required = parser.add_argument_group('Required')
    optional = parser.add_argument_group('Optional')

    # add required:
    required.add_argument(
        "-i", dest="input", help="Input directory of KITTI test drive data.",
        required=True, type=str
    )

    # parse arguments:
    return parser.parse_args()


def main(config):
    """
    Generate 100Hz OXTS using both extract and sync measurements
    """
    # generate input paths:
    filename_oxts_extract_timestamps = os.path.join(
        config.input, 'oxts_extract', 'timestamps.txt'
    )
    filename_oxts_sync_timestamps = os.path.join(
        config.input, 'oxts_sync', 'timestamps.txt'
    )

    # load:
    df_timestamps_extract = pd.read_csv(filename_oxts_extract_timestamps, header=None)
    df_timestamps_sync = pd.read_csv(filename_oxts_sync_timestamps, header=None)

    # format:
    df_timestamps_sync.columns = df_timestamps_extract.columns = ['timestamp']

    # generate corresponding data filename:
    df_timestamps_extract['input_data'] = df_timestamps_extract.index.to_series().apply(
        lambda x: os.path.join(
            config.input, 'oxts_extract', 'data' , '{:010d}.txt'.format(x)
        )
    )
    df_timestamps_extract = df_timestamps_extract.drop_duplicates(subset=['timestamp'], keep='first')
    df_timestamps_sync['input_data'] = df_timestamps_sync.index.to_series().apply(
        lambda x: os.path.join(
            config.input, 'oxts_sync', 'data' , '{:010d}.txt'.format(x)
        )
    )
    df_timestamps_extract = df_timestamps_extract.drop_duplicates(subset=['timestamp'], keep='first')

    # concate extract & sync then sort by timestamp:
    df_timestamps = pd.concat(
        [df_timestamps_extract, df_timestamps_sync]
    ).drop_duplicates(subset=['timestamp'], keep='first').sort_values('timestamp').reset_index(
        drop=True
    )

    # create output data filenames:
    df_timestamps['output_data'] = df_timestamps.index.to_series().apply(
        lambda x: os.path.join(
            config.input, 'oxts', 'data' , '{:010d}.txt'.format(x)
        )
    )

    # generate output:
    if os.path.exists(
        os.path.join(config.input, 'oxts')
    ):
        shutil.rmtree(
            os.path.join(config.input, 'oxts')
        )
    
    # init output dir structure:
    os.makedirs(
        os.path.join(config.input, 'oxts', 'data')
    )
    # write timestamps:
    df_timestamps[['timestamp']].to_csv(
        os.path.join(config.input, 'oxts', 'timestamps.txt'),
        header=None,
        index=False
    )

    # write data:
    for i, oxts in df_timestamps.iterrows():
        shutil.copyfile(oxts['input_data'], oxts['output_data'])

    print df_timestamps['timestamp'].apply(
        lambda x: pd.to_datetime(x)
    ).diff().fillna(
        pd.Timedelta(0)
    ).describe()

    sys.exit(os.EX_OK) 

if __name__ == '__main__':
    # parse arguments:
    arguments = get_arguments()

    config = Config(
        input = arguments.input
    )

    main(config)
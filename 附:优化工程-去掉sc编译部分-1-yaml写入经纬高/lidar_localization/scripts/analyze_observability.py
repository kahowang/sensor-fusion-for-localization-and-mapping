#! /usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import shutil

import argparse
from collections import namedtuple

import numpy as np
import pandas as pd


Config = namedtuple(
    'Config', 
    ['motion_file', 'SOM', 'output_dir', 'sv_thresh'], 
    verbose=False
)


def get_arguments():
    """ 
    Get command-line arguments
    """
    # init parser:
    parser = argparse.ArgumentParser("Analyze observability for each motion def stage.")

    # add required and optional groups:
    required = parser.add_argument_group('Required')
    optional = parser.add_argument_group('Optional')

    # add required:
    required.add_argument(
        "-m", dest="motion_file", help="Input filepath of GNSS-INS-Sim motion def file.",
        required=True, type=str
    )
    required.add_argument(
        "-s", dest="SOM", help="Input filepath of SOM data file.",
        required=True, type=str
    )
    required.add_argument(
        "-o", dest="output_dir", help="Output directory of SOM analysis result.",
        required=True, type=str
    )

    # add optional
    optional.add_argument(
        "-t", dest="sv_thresh", help="Singular value threshold. Defaults to 1.0e-4",
        required=False, default=5.0e-5, type=float, 
    )

    # parse arguments:
    return parser.parse_args()


def main(config):
    """
    Get system observability & state variable's degree of observability for each motion stage
    """
    # load motion def:
    df_motion = pd.read_csv(config.motion_file, header=None, skiprows=[0, 1, 2])
    df_motion.columns = ['command_type', 'yaw', 'pitch', 'roll', 'x', 'y', 'z', 'duration', 'gps_visibility']
    # add timestamp:
    df_motion['timestamp'] = df_motion.duration.cumsum()
    # add motion stage id:
    df_motion['motion_stage_id'] = df_motion.index.values

    # load SOM data:
    df_Q = pd.read_csv(config.SOM + ".csv")
    df_SOM = pd.read_csv(config.SOM + "_som.csv")
    # convert to relative time
    df_Q['T'] = df_Q['T'] - df_Q.iloc[0, 0] + 1
    df_SOM['T'] = df_SOM['T'] - df_SOM.iloc[0, 0] + 1

    # get motion stage id of each SOM record:
    motion_stage_bins = df_motion.timestamp.values

    df_Q['motion_stage_id'] = df_Q['T'].apply(lambda x: np.digitize(x, motion_stage_bins))

    df_SOM['motion_stage_id'] = df_SOM['T'].apply(lambda x: np.digitize(x, motion_stage_bins))
    # get rank of each SOM record:
    degree_of_observability_columns = [ col for col in df_Q.columns if col.startswith('doo') ]
    df_Q['Q_rank'] = np.sum(
        df_Q.loc[:, degree_of_observability_columns].values > config.sv_thresh, axis = 1
    )
    df_Q['singular_value_max'] = 100.0 / df_Q['sv1'].max() * df_Q['sv1']

    df_SOM['SOM_rank'] = np.sum(
        df_SOM.loc[:, degree_of_observability_columns].values > config.sv_thresh, axis = 1
    )
    df_SOM['singular_value_max'] = 100.0 / df_SOM['sv1'].max() * df_SOM['sv1']

    #
    # get 
    #     1. min. Q rank
    #     2. min. degree of observability for each state variable
    # for each motion stage
    #
    df_Q['SOM_rank'] = df_SOM['SOM_rank']
    df_Q =  df_Q[
        ['motion_stage_id', 'SOM_rank', 'Q_rank', 'singular_value_max'] + degree_of_observability_columns
    ].groupby(
        ['motion_stage_id']
    ).min()

    df_SOM =  df_SOM[
        ['motion_stage_id', 'SOM_rank', 'singular_value_max'] + degree_of_observability_columns
    ].groupby(
        ['motion_stage_id']
    ).min()
    
    # set state variable names for SOM table
    state_variable_columns = []
    if len(degree_of_observability_columns) == 16:
        state_variable_columns = [
            'pE', 'pN', 'pU', 
            'vE', 'vN', 'vU', 
            'qw', 'qx', 'qy', 'qz',
            'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z',
            'accel_bias_x', 'accel_bias_y', 'accel_bias_z',
        ]
    else:
        state_variable_columns = [
            'pE', 'pN', 'pU', 
            'vE', 'vN', 'vU', 
            'thetaE', 'thetaN', 'thetaU',
            'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z',
            'accel_bias_x', 'accel_bias_y', 'accel_bias_z',
        ]
    df_Q.columns = ['SOM_rank', 'Q_rank', 'singular_value_max'] + state_variable_columns
    df_Q.reset_index(inplace=True, drop=True)

    df_SOM.columns = ['SOM_rank', 'singular_value_max'] + state_variable_columns
    df_SOM.reset_index(inplace=True, drop=True)

    # finally:
    df_output_Q = df_motion.join(df_Q, how='left')
    df_output_SOM = df_motion.join(df_SOM, how='left')

    # write to output:
    df_output_Q.to_csv(
        os.path.join(config.output_dir, os.path.basename(config.SOM) + ".csv")
    )
    df_output_SOM.to_csv(
        os.path.join(config.output_dir, os.path.basename(config.SOM) + "_som.csv")
    )

    sys.exit(os.EX_OK) 


if __name__ == '__main__':
    # parse arguments:
    arguments = get_arguments()

    config = Config(
        motion_file = arguments.motion_file,
        SOM = arguments.SOM,
        sv_thresh = arguments.sv_thresh,
        output_dir = arguments.output_dir
    )

    main(config)
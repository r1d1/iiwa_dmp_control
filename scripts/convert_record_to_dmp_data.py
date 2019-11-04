#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import argparse



if __name__ == "__main__":
    # Argument handling
    parser = argparse.ArgumentParser(description="Display skill list file")
    parser.add_argument('--file', type=str, help="CSV filename")
    args = parser.parse_args()

    if args.file == None:
        print("Provide filename")
        exit()

    data=pd.read_csv(args.file) #, index_col=0)

    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    cols = ['timestamp', 'arm_cart_pose_wf_0','arm_cart_pose_wf_1','arm_cart_pose_wf_2','arm_cart_pose_wf_3','arm_cart_pose_wf_4','arm_cart_pose_wf_5','arm_cart_pose_wf_6','suction_strenght_0', 'eef_gripper_0']
    cols_ = ['scaled_time', 'arm_cart_pose_wf_0','arm_cart_pose_wf_1','arm_cart_pose_wf_2','arm_cart_pose_wf_3','arm_cart_pose_wf_4','arm_cart_pose_wf_5','arm_cart_pose_wf_6','suction_strenght_0', 'eef_gripper_0']
    #ax.plot(data['timestamp'], data[cols])

    #print(cols)
    converted_data = pd.DataFrame(data[cols])
    converted_data['scaled_time'] = data['timestamp']
    new_cols = converted_data.columns.to_list()
    new_cols = new_cols[-1:] + new_cols[1:-1]
    print(converted_data.columns)
    converted_data=converted_data[new_cols]
    #converted_data.rename(columns={'timestamp':'scaled_time'}, inplace=True)
    #converted_data['scaled_time'] = pd.Series([0]*len(data['timestamp']))
    #converted_data = pd.concat([converted_data.to_frame().T, data[cols]])
    #cols_ = ['scaled_time'] + cols
    #converted_data=converted_data[cols_]
    print(converted_data)
    #ax.plot(data['timestamp'], data[~data.index.isin(['timestamp'])])
    #plt.show()

    with open(args.file.split('.')[0]+'_cleaned.csv', 'w') as f:
        converted_data.to_csv(f,sep=',')

import os
import open3d as o3d
import sys, getopt

'''
@AUTHOR: 
@MODIFIER: iLori <haiyang.jiang@dorabot.com>
@BRIEF: This script transform .ply cloud file from Lucid camera to .pcd cloud file
@USAGE: Run this script with "-h" flag
'''

if __name__=="__main__":

    fileList = []
    save_path = ""

    try:
        opts, args = getopt.getopt(sys.argv[1:],"hf:d:s:",["input_files=","input_directory=","save_path="])
    except getopt.GetoptError:
        print('ply_to_pcd.py -f <input_file_1> <input_file_2> ... -d <input_directory_1> <input_directory_2> ... -s <save_path>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('ply_to_pcd.py -f <input_file_1> <input_file_2> ... -d <input_directory_1> <input_directory_2> ... -s <save_path>')
            sys.exit()
        elif opt in ("-f", "--input_files"):
            fileList.append(arg)
        elif opt in ("-d", "--input_directories"):
            if str(arg)[-1] != "/":
                raise Exception("Input directory should end with '/'")
            path = str(arg)
            fileList_temp = os.listdir(path)
            fileList_temp.sort()
            for i in range(len(fileList_temp)):
                fileList_temp[i] = path + fileList_temp[i]
            fileList += fileList_temp
        elif opt in ("-s", "--save_path"):
            if str(arg)[-1] != "/":
                raise Exception("Input save path should end with '/'")
            save_path = str(arg)

    for file in fileList:
        file = str(file)
        if '.ply' in file: 
            num = file[-5:-4]
            num = "cloud_" + num
            pcd = o3d.io.read_point_cloud(file, remove_nan_points=False)

            new_name = num + '.pcd'
            new_file = save_path + new_name
            
            print(file,'->',new_name)
            
            o3d.io.write_point_cloud(new_file, pcd, write_ascii=True)

    print("Transformation done!")
    print("Files saved to directory:", save_path)

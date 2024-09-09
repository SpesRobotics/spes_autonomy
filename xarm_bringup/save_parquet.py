import os
import pandas as pd
import numpy as np
from datetime import datetime
import datasets
from PIL import Image
import io
import re
import argparse

REWARD_STEP = 0.0065

def load_images(image_folder):
    images = []
    filtered_index = []
    for filename in os.listdir(image_folder):
        if filename.endswith(".jpg"):
            image_path = os.path.join(image_folder, filename)
            
            filtered_name = re.findall("^[0-9]+", image_path.split('/')[-1])[0]
            filtered_index.append(int(filtered_name))
    
    filtered_index.sort()
   
    for index in filtered_index:
        new_image_path = image_folder + '/' + str(index)+'.jpg' 

        image = Image.open(new_image_path)
        imgByteArr = io.BytesIO()
        image.save(imgByteArr, format=image.format)
        imgByteArr = imgByteArr.getvalue()
        images.append(imgByteArr)
    return images

def load_episodes(path):
    episode_name = []
    for filename in os.listdir(path):
        episode_name.append(filename[:-4])
        print(filename[:-4])
    
    return episode_name


def delete_rotation(episode_content):
    num_frames = len(episode_content)
    actions = []

    for line in range(0, num_frames):
        current_pose = episode_content[line].replace("\n", "")[1:-1]
        current_pose = current_pose.split(',')
        
        current_pose_np = []
        for i in current_pose:
            current_pose_np.append(float(i))

        action= current_pose_np[0:3]

        action_str = '[{}]'.format(', '.join(map(str, action))) + '\n'
        actions.append(action_str)
    return actions

def check_actions(episode_content, filename):
    num_frames = len(episode_content)
    actions = []

    for line in range(0, num_frames):
        current_pose = episode_content[line].replace("\n", "")[1:-1]
        current_pose = current_pose.split(',')
        
        current_pose_np = []
        for i in current_pose:
            current_pose_np.append(float(i))

        action= current_pose_np[3:]
        for a in action:
            if abs(a) > 0.002:
                print(f'[ERROR] file {filename} conatains invalid value')
                return

def save_data_frame(path):
    output_dir = path + '/parquest_output'
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)


    date = datetime.now()
    output_path = output_dir + "/" + \
        date.strftime("%Y_%m_%d_%H_%M_%S") + ".parquet"
    print("File is saved on path: ", output_path)
    
    index = 0 
    data = {
            'observation.image': [],
            'observation.state': [],
            'action': [],
            'episode_index': [],
            'frame_index': [],
            'timestamp': [],
            'next.reward': [],
            'next.done': [],
            'next.success': [],
            'index': []
        }
    
    action_data = path + '/actions'
    episode_name = load_episodes(action_data)
    i = 0
    for name in episode_name:
        
        print(f'[INFO] Start {i}.  {name} episode saving...')

        
        image_folder = path +  '/' + name
        file_path = path +  '/' + 'actions/' + name + '.txt'
        observation_file_path = path +  '/' + 'observations/' + name + '.txt'
        reward_file_path = path +  '/' + 'rewards/' + name + '.txt'

        images = load_images(image_folder)

        file_content = open(file_path, 'r').readlines()
        check_actions(file_content, image_folder)
        
        observation_file_content = open(observation_file_path, 'r').readlines()
        file_length = len(file_content)

        frame_index = 0
        timestamp = 0.0
        next_done = False
        next_success = False
        image_index = 0

        print(len(images), file_length)
        for line in range(1, file_length):
            next_line = file_content[line].replace("\n", "")
            observation_line = observation_file_content[line].replace("\n", "")
            reward_line = 1000

            current_image = images[image_index]
        
            data['observation.image'].append({'bytes': current_image, 'path': None})
            data['observation.state'].append(observation_line)
            
            data['action'].append(next_line)
            data['episode_index'].append(i)
            data['frame_index'].append(frame_index)
            data['timestamp'].append(timestamp)
            data['next.reward'].append(reward_line)
            data['next.done'].append(next_done)
            data['next.success'].append(next_success)
            data['index'].append(index)

            index += 1
            frame_index += 1
            timestamp += 0.1
            
            image_index +=1
            if line >= (len(file_content) - 10):
                next_done = True
                next_success = True

        i+=1
    df = pd.DataFrame(data)
    df.to_parquet(output_path, engine='pyarrow')
    print("File is saved on path: ", output_path)


def load_hf_dataset_from_parquet(file_path):
    hf_dataset = datasets.load_dataset("parquet", data_files=str(file_path))
    return hf_dataset



def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--real', action='store_true', help='Enable saving data from real environment.')
    parser.add_argument('--filename', type=str, help='Name of data directory.')

    args = parser.parse_args()

    DATA_FILE = 'DATA'
    if args.real:
        DATA_FILE = 'DATA_REAL'

    if args.filename:
        DATA_FILE = args.filename
    file_path = "/home/marija/spes_autonomy/xarm_bringup/" + DATA_FILE
    image_path = "/home/marija/Desktop/imitation-learning/isaac_sim/colect_training_data/data/2024_06_21_16_01_13"

    save_data_frame(file_path)

def test():
    test_str = ['[10, 12, 11, 0, 0, 0]\n', '[25, 24, 21, 0, 0, 0]\n', '[2, 14, 12, 0, 0, 0]\n', '[10, 11, 30, 0, 0, 0]\n', '[10, 10, 30, 0, 0, 0]\n']
    print(delete_rotation(test_str))
    
if __name__ == '__main__':
    # test()
    main()
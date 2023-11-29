from roboflow import Roboflow
from ultralytics import YOLO
from glob import glob
import yaml
import os
import argparse


DATA_YAML_FILENAME = 'dataset/data.yaml'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--key", help="Roboflow API key")
    parser.add_argument("--project", help="Roboflow project name")
    parser.add_argument("--workspace", help="Roboflow workspace name")
    parser.add_argument("--destination", help="Remote destination")
    parser.add_argument("--steps", default='download,train,upload', help="Steps to run")
    parser.add_argument("--epochs", default=50, help="Number of epochs")
    parser.add_argument("--imgsz", default=32*20, type=int, help="Image size")
    args = parser.parse_args()

    steps = args.steps.split(',')

    if 'download' in steps:
        rf = Roboflow(api_key=args.key)
        project = rf.workspace(args.workspace).project(args.project)

        # Generate and download the latest version of the dataset
        project.generate_version(
            {
                "augmentation": {},
                "preprocessing": {},
            }
        )
        latest_version = project.versions()[0].version
        latest_version_number = int(latest_version.split("/")[-1])
        project.version(latest_version_number).download("yolov8", location="dataset")

        # Fix the paths in data.yaml file
        data_dir = os.path.abspath(os.path.dirname(DATA_YAML_FILENAME))
        data_yaml = yaml.load(open(DATA_YAML_FILENAME), Loader=yaml.FullLoader)
        data_yaml["train"] = os.path.join(data_dir, "train", "images")
        data_yaml["val"] = os.path.join(data_dir, "valid", "images")
        data_yaml["test"] = os.path.join(data_dir, "test", "images")
        yaml.dump(data_yaml, open(DATA_YAML_FILENAME, "w"))

    if 'train' in steps:
        model = YOLO("yolov8n.pt")
        model.train(data=DATA_YAML_FILENAME, epochs=int(args.epochs), imgsz=int(args.imgsz))

    if 'upload' in steps:
        model_path = glob("**/best.pt", recursive=True)[-1]
        os.system(f"scp {model_path} {args.destination}")


if __name__ == "__main__":
    main()

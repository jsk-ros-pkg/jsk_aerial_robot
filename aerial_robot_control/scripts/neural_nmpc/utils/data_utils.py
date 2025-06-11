import os
import json
import errno
import shutil
from pathlib import Path
import joblib
import numpy as np
import pandas as pd
import torch
import ml_casadi.torch as mc
from config.configurations import DirectoryConfig


def make_blank_dict(target_dim, state_dim, input_dim):
    blank_recording_dict = {
        "timestamp": np.zeros((0, 1)),
        "comp_time": np.zeros((0, 1)),
        "target": np.zeros((0, target_dim)),
        "state_in": np.zeros((0, state_dim)),
        "state_out": np.zeros((0, state_dim)),
        "state_pred": np.zeros((0, state_dim)),
        "error": np.zeros((0, state_dim)),
        "control": np.zeros((0, input_dim)),
    }
    return blank_recording_dict

def safe_mkdir_recursive(directory, overwrite=False):
    """
    Recursively creates a path to the desired directory.
    :param directory: Directory to be created
    :param overwrite: If True, the directory will first be removed if it already exists
    :return: None
    """
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(directory):
                # Directory already exists
                pass
            else:
                raise
    else:
        if overwrite:
            try:
                shutil.rmtree(directory)
            except Exception as e:
                print(f'Error while removing directory: {directory} | Error: {e}')
            # Create the directory again
            os.makedirs(directory)

def safe_mkfile_recursive(destiny_dir, file_name, overwrite):
    """
    Recursively creates the directory path and file name. If the file already exists, it will be removed if
    overwrite is set to True. Otherwise, it will not be removed and the function will return True.

    :param destiny_dir: Directory where the file will be created
    :param file_name: Name of the file to be created
    :param overwrite: If True, the file will be removed if it already exists
    :return: False if the file already exists and was not overwritten, True if the file was created or overwritten
    """
    # Recursively create the storing directory path
    safe_mkdir_recursive(destiny_dir)   # Don't use overwrite as the storing file might exist
    # Check if file already exists and remove it
    if overwrite and os.path.exists(os.path.join(destiny_dir, file_name)):
        os.remove(os.path.join(destiny_dir, file_name))
    if not os.path.exists(os.path.join(destiny_dir, file_name)):
        # Create file
        Path(os.path.join(destiny_dir, file_name)).touch()
        return True  # File was newly created or overwritten
    return False     # File already exists and was not overwritten

# def store_recording_data(rec_dict, state_curr, x_pred):
#     """
#     Store the data in the recording dictionary in place.
#     :param rec_dict: Dictionary to store the data
#     :param state_curr: Current state of the system in NMPC
#     :param x_pred: Predicted state of the system in NMPC
#     :param u_cmd: Last commanded control input
#     """
#     rec_dict["state_out"] = np.append(rec_dict["state_out"], state_curr[np.newaxis, :], axis=0)

#     if x_pred is not None:
#         error = state_curr - x_pred
#         rec_dict["error"] = np.append(rec_dict["error"], error[np.newaxis, :], axis=0)
#         rec_dict["state_pred"] = np.append(rec_dict["state_pred"], x_pred[np.newaxis, :], axis=0)

def write_recording_data(rec_dict, rec_file):
    # # Current target was reached - remove incomplete recordings
    # if len(rec_dict["state_in"]) > len(rec_dict["state_out"]):
    #     rec_dict["timestamp"] = rec_dict["timestamp"][:-1]
    #     rec_dict["comp_time"] = rec_dict["comp_time"][:-1]
    #     rec_dict["target"] = rec_dict["target"][:-1]
    #     rec_dict["state_in"] = rec_dict["state_in"][:-1]
    #     rec_dict["control"] = rec_dict["control"][:-1]

    for key in rec_dict.keys():
        rec_dict[key] = jsonify(rec_dict[key])

    df = pd.DataFrame(rec_dict)
    df.to_csv(rec_file, index=True, mode='a', header=False) # Append to CSV file

def get_recording_dict_and_file(recording_options, target_dim, state_dim, input_dim, sim_options, overwrite=True):
    dataset_name = recording_options["dataset_name"]

    # Directory and file name for data recording
    rec_file_dir, rec_file_name = get_data_dir_and_file(dataset_name, recording_options["split"], sim_options)
    rec_file = os.path.join(rec_file_dir, rec_file_name)

    # Recursively create the storing directory path
    is_blank = safe_mkfile_recursive(rec_file_dir, rec_file_name, overwrite=overwrite)

    # Create empty recording dictionary
    # TODO shouldn't we load the existing data if file exists?
    rec_dict = make_blank_dict(target_dim, state_dim, input_dim)
    # Generate new CSV to store data in
    rec_json = dict()
    if is_blank:
        for key in rec_dict.keys():
            rec_json[key] = jsonify(rec_dict[key])

        df = pd.DataFrame(rec_json)
        df.to_csv(rec_file, index=False, header=True)

    return rec_dict, rec_file

def get_data_dir_and_file(ds_name, split, params, read_only=False):
    """
    Returns the directory and file name where to store the next simulation-based dataset.
    :param ds_name: Name of the dataset
    :param split: Either "train" or "test" depending on the dataset usecase.
    :param params: Dictionary with the parameters of the dataset
    :param read_only: If True, the function will not create any directories or files. It will only return the
    directory and file name.
    :return: Tuple with the directory and file name
    """

    # Data folder directory
    data_dir = DirectoryConfig.DATA_DIR

    # Dataset split sanity check
    if not (split == "train" or split == "test"):
        raise ValueError("Split must be either 'train' or 'test'.")

    # Check if current dataset folder already exists and store any recorded datasets. Create it otherwise
    dataset_dir = os.path.join(data_dir, ds_name, split)
    if os.path.exists(dataset_dir):
        dataset_instances = []
        for (_, _, file_names) in os.walk(dataset_dir):
            dataset_instances.extend([os.path.splitext(file)[0] for file in file_names if not file.startswith('.')])
            break
    else:
        if read_only:
            return None
        safe_mkdir_recursive(dataset_dir)
        dataset_instances = []

    # Check if metadata file exists
    json_file_name = os.path.join(data_dir, "metadata.json")
    try:
        with open(json_file_name, "r") as json_file:
            metadata = json.load(json_file)
        
        # Check if current dataset name with data split exists
        if ds_name in metadata.keys() and split in metadata[ds_name].keys():
            # Check if configuration parameters already exists in other metadata 
            existing_instance_idx = -1
            for i, instance in enumerate(dataset_instances):
                if metadata[ds_name][split][instance] == params:
                    existing_instance_idx = i
                    if not read_only:
                        print("This configuration already exists in the dataset with the same name.")

            if existing_instance_idx == -1:
                if read_only:
                    return None

                if dataset_instances:
                    # Dataset exists but currently a new configuration is used
                    existing_instances = [int(instance.split("_")[1]) for instance in dataset_instances]
                    max_instance_number = max(existing_instances)
                    ds_instance_name = "dataset_" + str(max_instance_number + 1).zfill(3)   # Add counter in the filename

                    # Add the new parameter configuration to metadata dictionary
                    metadata[ds_name][split][ds_instance_name] = params

                else:
                    # Edge case where, for some error, there was something added to the metadata file but no actual
                    # datasets were recorded. Remove entries from metadata and add them again.
                    ds_instance_name = "dataset_001"
                    metadata[ds_name][split] = {}
                    metadata[ds_name][split][ds_instance_name] = params

            else:
                # Dataset exists and there is an instance with the same configuration
                ds_instance_name = dataset_instances[existing_instance_idx]

        # Dataset does not exist yet in metadata
        else:
            if read_only:
                return None

            # Add the new dataset to metadata dictionary
            ds_instance_name = "dataset_001"
            if ds_name in metadata.keys():
                metadata[ds_name][split] = {ds_instance_name: params}
            else:
                metadata[ds_name] = {split: {ds_instance_name: params}}

        if not read_only:
            with open(json_file_name, 'w') as json_file:
                json.dump(metadata, json_file, indent=4)
        

    except FileNotFoundError:
        if read_only:
            return None
        
        print("Metadata file not found, creating a new one.")
        with open(json_file_name, "w") as json_file:
            ds_instance_name = "dataset_001"
            json.dump({ds_name: {split: {ds_instance_name: params}}}, json_file, indent=4)

    return dataset_dir, ds_instance_name + '.csv'


def get_model_dir_and_file(model_options):
    """
    Returns the directory and file name of the fitted model.
    :param model_options: Parameter dictionary with the keys:
                          "git", "model_name", "model_type", "params".
    :return: Tuple with the directory and file name
    """
    # Model folder directory
    directory = os.path.join(DirectoryConfig.SAVE_DIR, str(model_options["git"]), str(model_options["model_name"]))

    # Store parameters in file name
    model_params = model_options["disturbances"]
    file_name = ''
    model_vars = list(model_params.keys())
    model_vars.sort()
    for i, param in enumerate(model_vars):
        if i > 0:
            file_name += '__'
        file_name += 'NO_' if not model_params[param] else ''
        file_name += param

    return directory, file_name


def load_pickled_models(directory='', file_name='', model_options=None):
    """
    Loads a pre-trained model from the specified directory, contained in a given pickle filename. Otherwise, if
    the model_options dictionary is given, use its contents to reconstruct the directory location of the pre-trained
    model fitting the requirements.

    :param directory: directory where the model file is located
    :param file_name: file name of the pre-trained model
    :param model_options: dictionary with the keys: "noisy" (bool), "drag" (bool), "git" (string), "training_samples"
    (int), "payload" (bool).

    :return: a dictionary with the recovered models from the pickle files.
    """

    if model_options is not None:
        directory, file_name = get_model_dir_and_file(model_options)

    try:
        pickled_files = os.listdir(directory)
    except FileNotFoundError:
        return None

    loaded_models = []

    try:
        for file in pickled_files:
            if not file.startswith(file_name) and file != 'feats.csv':
                continue
            if '.pt' in file:
                directory, file_name = get_model_dir_and_file(load_ops)
                saved_dict = torch.load(os.path.join(directory, f"{file_name}.pt"))
                mlp_model = mc.nn.MultiLayerPerceptron(saved_dict['input_size'], saved_dict['hidden_size'],
                                               saved_dict['output_size'], saved_dict['hidden_layers'], 'Tanh')
                model = NormalizedMLP(mlp_model, torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
                                      torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
                                      torch.tensor(np.zeros((saved_dict['output_size'],))).float(),
                                      torch.tensor(np.zeros((saved_dict['output_size'],))).float())
                model.load_state_dict(saved_dict['state_dict'])
                model.eval()
                pre_trained_models = model
                loaded_models.append(joblib.load(os.path.join(directory, file)))
                continue
            if '.pkl' not in file and '.csv' not in file:
                continue
            if '.pkl' in file:
                loaded_models.append(joblib.load(os.path.join(directory, file)))

    except IsADirectoryError:
        raise FileNotFoundError("Tried to load file from directory %s, but it was not found." % directory)

    if loaded_models is not None:
        if loaded_models:
            pre_trained_models = {"models": loaded_models}
        else:
            pre_trained_models = None
    else:
        pre_trained_models = None

    return pre_trained_models

def jsonify(array):
    if isinstance(array, np.ndarray):
        return array.tolist()
    if isinstance(array, list):
        return array
    return array

def undo_jsonify(array):
    x = []
    for elem in array:
        a = elem.split('[')[1].split(']')[0].split(',')
        a = [float(num) for num in a]
        x = x + [a]
    return np.array(x)
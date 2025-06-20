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


def safe_mkdir_recursive(directory, overwrite: bool = False):
    if overwrite:
        shutil.rmtree(directory)
        os.makedirs(directory)
    else:
        os.makedirs(directory, exist_ok=True)

def safe_mkfile_recursive(destiny_dir, file_name, overwrite: bool = False):
    safe_mkdir_recursive(destiny_dir)

    if overwrite and os.path.exists(os.path.join(destiny_dir, file_name)):
        os.remove(os.path.join(destiny_dir, file_name))
    if not os.path.exists(os.path.join(destiny_dir, file_name)):
        Path(os.path.join(destiny_dir, file_name)).touch()
        return True  # File was newly created or overwritten
    return False     # File already exists and was not overwritten

def get_recording_dict_and_file(recording_options, target_dim, state_dim, input_dim, sim_options, overwrite: bool = True):
    """
    Returns a dictionary to store the recording data and the file where to store it.
    :param recording_options: Dictionary with the options for the recording.
    :param target_dim: Dimension of the target vector.
    :param state_dim: Dimension of the state vector.
    :param input_dim: Dimension of the input vector.
    :param sim_options: Dictionary with the options for disturbances in simulation.
    :param overwrite: If True, the existing file will be overwritten. Otherwise, it will be appended to.
    :return: Tuple with the recording dictionary and the file name where to store the data.
    """
    rec_file_dir, rec_file_name = get_data_dir_and_file(recording_options["dataset_name"],
                                                        recording_options["split"],
                                                        state_dim, input_dim, sim_options)
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

def get_data_dir_and_file(ds_name, split, state_dim, input_dim, sim_options, read_only=False):
    """
    Returns the directory and file name where to store the next simulation-based dataset.
    :param ds_name: Name of the dataset
    :param split: Either "train" or "val" or "test" depending on the dataset usecase.
    :param state_dim: Dimension of the state space.
    :param input_dim: Dimension of the input space.
    :param sim_options: Dictionary with the options for disturbances in simulation.
    :param read_only: If True, the function will not create any directories or files. It will only return the
    directory and file name.
    :return: Tuple with the directory and file name
    """
    dataset_dir = os.path.join(DirectoryConfig.DATA_DIR, ds_name, split)

    sim_setup = {"state_dim": state_dim, "input_dim": input_dim, **sim_options}

    # Dataset split sanity check
    if not (split == "train" or split == "test" or split == "val"):
        raise ValueError("Split must be either 'train' or 'test' or 'val'.")

    if os.path.exists(dataset_dir):
        dataset_instances = []
        for (_, _, file_names) in os.walk(dataset_dir):
            dataset_instances.extend([os.path.splitext(file)[0] for file in file_names if not file.startswith('.')])
    else:
        if read_only:
            return None
        safe_mkdir_recursive(dataset_dir)
        dataset_instances = []

    # Check if metadata file exists
    json_file_name = os.path.join(DirectoryConfig.DATA_DIR, "metadata.json")
    if os.path.exists(json_file_name):
        with open(json_file_name, "r") as json_file:
            metadata = json.load(json_file)
        
        # Check if current dataset name with data split exists
        if ds_name in metadata.keys() and split in metadata[ds_name].keys():
            # Check if simulation options, i.e., disturbance parameters already exists in metadata 
            existing_instance_idx = -1
            for i, instance in enumerate(dataset_instances):
                if metadata[ds_name][split][instance] == sim_setup:
                    existing_instance_idx = i
                    if not read_only:
                        print("Current configuration already exists. Not adding new entry to metadata file.")

            if existing_instance_idx == -1:
                if read_only:
                    return None

                if dataset_instances:
                    # Dataset name exists but current configuration is new
                    existing_instances = [int(instance.split("_")[1]) for instance in dataset_instances]
                    max_instance_number = max(existing_instances)
                    ds_instance_name = "dataset_" + str(max_instance_number + 1).zfill(3)   # Add counter in the filename

                    # Add the new simulation configuration to metadata
                    metadata[ds_name][split][ds_instance_name] = sim_setup

                else:
                    # Edge case where, for some error, there was something added to the metadata file but no actual
                    # datasets were recorded. Remove entries from metadata and add them again.
                    ds_instance_name = "dataset_001"
                    metadata[ds_name][split] = {}
                    metadata[ds_name][split][ds_instance_name] = sim_setup

            else:
                # Dataset exists and there is an instance with the same configuration
                ds_instance_name = dataset_instances[existing_instance_idx]

        else:
            # Dataset does not exist yet in metadata
            if read_only:
                return None

            # Add the new dataset to metadata dictionary
            ds_instance_name = "dataset_001"
            if ds_name in metadata.keys():
                metadata[ds_name][split] = {ds_instance_name: sim_setup}
            else:
                metadata[ds_name] = {split: {ds_instance_name: sim_setup}}

        if not read_only:
            with open(json_file_name, 'w') as json_file:
                json.dump(metadata, json_file, indent=4)
        

    else:
        # Metadata file does not exist, create it
        if read_only:
            return None
        
        with open(json_file_name, "w") as json_file:
            ds_instance_name = "dataset_001"
            metadata = {ds_name: {split: {ds_instance_name: sim_setup}}}
            json.dump(metadata, json_file, indent=4)

    return dataset_dir, ds_instance_name + '.csv'

def get_model_dir_and_file(git_version, model_name, state_dim, input_dim, sim_options):
    """
    Returns the directory and file name of the fitted model.
    :param git_version: Git commit hash of the model.
    :param model_name: Name of the model.
    :param state_dim: Dimension of the state space.
    :param input_dim: Dimension of the input space.
    :param sim_options: Dictionary with the options for disturbances in simulation.
    :return: Tuple with the directory and file name
    """
    directory = os.path.join(DirectoryConfig.SAVE_DIR, git_version, model_name)

    # Store disturbances in file name
    file_name = 'nx_' + str(state_dim) + '_nu_' + str(input_dim)
    model_vars = list(sim_options.keys())
    model_vars.sort()
    for i, dist in enumerate(model_vars):
        file_name += '__'
        file_name += 'NO_' if not sim_options[dist] else ''
        file_name += dist

    return directory, file_name

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

def write_recording_data(rec_file, rec_dict):
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

def read_dataset(ds_name, split, state_dim, input_dim, sim_options):
    """
    Attempts to read a dataset given its name and its metadata.
    :param ds_name: Name of the dataset.
    :param split: String indicating to load a training, validation or test split.
    :param state_dim: Dimension of the state space.
    :param input_dim: Dimension of the input space.
    :param sim_options: Dictionary with the options for disturbances in simulation.
    :return: Pandas DataFrame of the csv dataset.
    """
    response = get_data_dir_and_file(ds_name, split, state_dim, input_dim, sim_options, read_only=True)
    if response is None:
        raise FileNotFoundError
    rec_file_dir, rec_file_name = response

    rec_file = os.path.join(rec_file_dir, rec_file_name)
    return pd.read_csv(rec_file)

def load_pickled_models(directory='', file_name='', model_options=None):
    raise NotImplementedError()
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
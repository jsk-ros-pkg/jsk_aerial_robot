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

def get_recording_dict_and_file(ds_name, model_options, sim_options, solver_options, target_dim, overwrite: bool = True):
    """
    Returns a dictionary to store the recording data and the file where to store it.
    :param overwrite: If True, the existing file will be overwritten. Otherwise, it will be appended to.
    """
    rec_file_dir, rec_file_name = get_data_dir_and_file(ds_name, model_options,
                                                        sim_options, solver_options)
    rec_file = os.path.join(rec_file_dir, rec_file_name)

    # Recursively create the storing directory path
    is_blank = safe_mkfile_recursive(rec_file_dir, rec_file_name, overwrite=overwrite)

    # Create empty recording dictionary
    # TODO shouldn't we load the existing data if file exists?
    rec_dict = make_blank_dict(target_dim, model_options["state_dim"], model_options["control_dim"])
    # Generate new CSV to store data in
    rec_json = dict()
    if is_blank:
        for key in rec_dict.keys():
            rec_json[key] = jsonify(rec_dict[key])

        df = pd.DataFrame(rec_json)
        df.to_csv(rec_file, index=False, header=True)

    return rec_dict, rec_file

def get_data_dir_and_file(ds_name, model_options, sim_options, solver_options):
    """
    Returns the directory and file name where to store the next simulation-based dataset.
    """
    dataset_dir = os.path.join(DirectoryConfig.DATA_DIR, ds_name)

    outer_fields = {**model_options, "solver_options": solver_options}
    inner_fields = sim_options

    # Parse recorded datasets
    if os.path.exists(dataset_dir):
        dataset_instances = []
        for (_, _, file_names) in os.walk(dataset_dir):
            dataset_instances.extend([os.path.splitext(file)[0] for file in file_names if not file.startswith('.')])
    else:
        safe_mkdir_recursive(dataset_dir)
        dataset_instances = []

    # Check if metadata file exists
    json_file_name = os.path.join(DirectoryConfig.DATA_DIR, "metadata.json")
    if os.path.exists(json_file_name):
        with open(json_file_name, "r") as json_file:
            metadata = json.load(json_file)
        
        # Check if current dataset name exists
        if ds_name in metadata.keys():
            # Check if current controller configuration exists
            existing_controller = 1
            for field in metadata[ds_name].keys():
                if field.startswith("dataset_"):
                    continue
                if metadata[ds_name][field] == outer_fields[field]:
                    existing_controller *= 1
                else:
                    existing_controller *= 0

            if existing_controller:
                # Check if simulation options, i.e., disturbance parameters already exists in metadata 
                existing_instance_idx = -1
                for i, instance in enumerate(dataset_instances):
                    if metadata[ds_name][instance] == inner_fields:
                        existing_instance_idx = i
                        print("Current configuration already exists. Skipping new entry to metadata file.")

                if existing_instance_idx == -1:
                    if dataset_instances:
                        # Dataset name and controller exists but current simulation options are new
                        existing_instances = [int(instance.split("_")[1]) for instance in dataset_instances]
                        max_instance_number = max(existing_instances)
                        ds_instance_name = "dataset_" + str(max_instance_number + 1).zfill(3)   # Add counter in the filename

                        # Add the new simulation configuration to metadata
                        metadata[ds_name][ds_instance_name] = inner_fields

                    else:
                        # Edge case where, for some error, there was something added to the metadata file but no actual
                        # datasets were recorded. Remove entries from metadata and add them again.
                        # TODO think this through, possibly not working as expected
                        ds_instance_name = "dataset_001"
                        metadata[ds_name][ds_instance_name] = inner_fields

                else:
                    # Dataset exists and there is an instance with the same configuration
                    # Don't update metadata, just return the existing instance for loading/overwriting
                    print(f"Dataset \"{ds_name}\" with instance \"{dataset_instances[existing_instance_idx]}\" already exists.")
                    print("[!] Warning: When generating new data, the existing dataset will be overwritten.")
                    ds_instance_name = dataset_instances[existing_instance_idx]

            else:
                raise ValueError(f"Existing configuration {outer_fields} for dataset {ds_name} found. \
                                   Set unique dataset name for new configuration.")
            #     # Dataset name exists but current controller configuration is new
            #     metadata[ds_name] = outer_fields
            #     ds_instance_name = "dataset_001"
            #     metadata[ds_name][ds_instance_name] = inner_fields

        else:
            # Dataset does not exist yet in metadata
            # Add the new dataset to metadata dictionary
            ds_instance_name = "dataset_001"
            metadata[ds_name] = outer_fields
            metadata[ds_name][ds_instance_name] = inner_fields

        # Write updated metadata to file
        with open(json_file_name, 'w') as json_file:
            json.dump(metadata, json_file, indent=4)

    else:
        # Metadata file does not exist yet
        with open(json_file_name, "w") as json_file:
            ds_instance_name = "dataset_001"
            metadata = {ds_name:
                        {**outer_fields,
                        ds_instance_name: inner_fields}}
            json.dump(metadata, json_file, indent=4)

    return dataset_dir, ds_instance_name + '.csv'

def get_model_dir_and_file(ds_name, ds_instance, model_name):
    """
    Returns the directory and file name of the fitted neural network model.
    """
    model_dir = os.path.join(DirectoryConfig.SAVE_DIR, model_name)

    # Check for existing models
    if os.path.exists(model_dir):
        model_instances = []
        for (_, _, file_names) in os.walk(model_dir):
            model_instances.extend([os.path.splitext(file)[0] for file in file_names if not file.startswith('.')])
    else:
        safe_mkdir_recursive(model_dir)
        model_instances = []

    # Increment counter for model file name
    if model_instances:
        existing_instances = [int(instance.split("_")[1]) for instance in model_instances]
        max_instance_number = max(existing_instances)
        model_file = "neuralmodel_" + str(max_instance_number + 1).zfill(3)   # Add counter in the filename
    else:
        model_file = "neuralmodel_001"

    # Get metadata information for configuration of controller and simulation setup
    json_file_name = os.path.join(DirectoryConfig.DATA_DIR, "metadata.json")
    with open(json_file_name, "r") as json_file:
        metadata = json.load(json_file)
    model_options = metadata[ds_name].copy()

    # Add model information to metadata
    with open(json_file_name, "w") as json_file:
        metadata[ds_name][ds_instance].update({"trained_model": model_file})
        json.dump(metadata, json_file, indent=4)

    return model_dir, model_file, model_options

def make_blank_dict(target_dim, state_dim, control_dim):
    blank_recording_dict = {
        "timestamp": np.zeros((0, 1)),
        "dt": np.zeros((0, 1)),
        "comp_time": np.zeros((0, 1)),
        "target": np.zeros((0, target_dim)),
        "state_in": np.zeros((0, state_dim)),
        "state_out": np.zeros((0, state_dim)),
        "state_pred": np.zeros((0, state_dim)),
        "error": np.zeros((0, state_dim)),
        "control": np.zeros((0, control_dim)),
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
    if len(rec_dict["state_in"]) > len(rec_dict["state_out"]):
        raise ValueError("Recording dictionary is not consistent.")
    #     rec_dict["timestamp"] = rec_dict["timestamp"][:-1]
    #     rec_dict["comp_time"] = rec_dict["comp_time"][:-1]
    #     rec_dict["target"] = rec_dict["target"][:-1]
    #     rec_dict["state_in"] = rec_dict["state_in"][:-1]
    #     rec_dict["control"] = rec_dict["control"][:-1]

    for key in rec_dict.keys():
        rec_dict[key] = jsonify(rec_dict[key])

    df = pd.DataFrame(rec_dict)
    df.to_csv(rec_file, index=True, mode='a', header=False) # Append to CSV file

def sanity_check_dataset(ds_name, ds_instance):
    # Check actual files
    if not os.path.exists(os.path.join(DirectoryConfig.DATA_DIR, ds_name, ds_instance + ".csv")):
        raise FileNotFoundError(f"Dataset directory for dataset {ds_name} and instance {ds_instance} does not exist.\
                                  Record dataset or check naming.")

    # Check metadata
    json_file_name = os.path.join(DirectoryConfig.DATA_DIR, "metadata.json")
    with open(json_file_name, "r") as json_file:
        metadata = json.load(json_file)
    if ds_name not in metadata.keys():
        raise ValueError(f"Dataset \"{ds_name}\" not found in metadata.")
    if ds_instance not in metadata[ds_name].keys():
        raise ValueError(f"Dataset instance \"{ds_instance}\" not found in metadata for dataset \"{ds_name}\".")

def read_dataset(ds_name, ds_instance):
    """
    Attempts to read a dataset given its name and returns a Pandas DataFrame of the csv dataset.
    """
    ds_file_name = os.path.join(DirectoryConfig.DATA_DIR, ds_name, f"{ds_instance}.csv")
    if not os.path.exists(ds_file_name):
        raise FileNotFoundError(f"Dataset {ds_name} with instance {ds_instance} not found.")

    return pd.read_csv(ds_file_name)

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
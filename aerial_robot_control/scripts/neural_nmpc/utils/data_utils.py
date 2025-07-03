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
from config.configurations import DirectoryConfig, MLPConfig


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

def get_model_dir_and_file(ds_name, ds_instance, model_name, x_feats, u_feats, y_reg_dims):
    """
    Reads the metadata for datasets and appends the model information to it.
    Creates/appends the current model configuration to the model's metadata file.
    Returns the directory and file name of the new neural network model.
    """
    model_dir = os.path.join(DirectoryConfig.SAVE_DIR, model_name)

    # Check if metadata file for models exists
    json_file_name = os.path.join(DirectoryConfig.SAVE_DIR, "metadata.json")
    if os.path.exists(json_file_name):
        with open(json_file_name, "r") as json_file:
            metadata = json.load(json_file)
    else:
        metadata = {}

    # Check if current model name exists
    if model_name not in metadata.keys():
        metadata[model_name] = {}

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
        model_instance = "neuralmodel_" + str(max_instance_number + 1).zfill(3)   # Add counter in the filename
    else:
        model_instance = "neuralmodel_001"

    # Add model information to dataset metadata and get dataset config
    json_file_name_dataset = os.path.join(DirectoryConfig.DATA_DIR, "metadata.json")
    with open(json_file_name_dataset, "r") as json_file:
        metadata_dataset = json.load(json_file)
    with open(json_file_name_dataset, "w") as json_file:
        if "trained_models" not in metadata_dataset[ds_name][ds_instance]:
            metadata_dataset[ds_name][ds_instance]["trained_models"] = [model_instance]
        else:
            metadata_dataset[ds_name][ds_instance]["trained_models"].append(model_instance)
        json.dump(metadata_dataset, json_file, indent=4)

    # Store new model configuration in metadata
    metadata[model_name][model_instance] = {
        "ds_name": ds_name,
        "ds_instance": ds_instance,
        "ds_nmpc_type": metadata_dataset[ds_name]["nmpc_type"],
        "ds_disturbances": metadata_dataset[ds_name][ds_instance]["disturbances"],
        "x_feats": str(x_feats),
        "u_feats": str(u_feats),
        "y_reg_dims": str(y_reg_dims),
        "MLPConfig": {key: value for (key, value) in vars(MLPConfig).items() if not key.startswith("__")},
    }

    # Write updated metadata to file
    with open(json_file_name, 'w') as json_file:
        json.dump(metadata, json_file, indent=4)

    return model_dir, model_instance

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

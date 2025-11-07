import os
import json
import time
import shutil
from pathlib import Path
import numpy as np
import pandas as pd
import rospkg
from config.configurations import DirectoryConfig, MLPConfig, ModelFitConfig


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
    return False  # File already exists and was not overwritten


def get_recording_dict_and_file(
    ds_name, model_options, sim_options, solver_options, target_dim, overwrite: bool = True
):
    """
    Returns a dictionary to store the recording data and the file where to store it.
    :param overwrite: If True, the existing file will be overwritten. Otherwise, it will be appended to.
    """
    rec_file_dir, rec_file_name = get_data_dir_and_file(ds_name, model_options, sim_options, solver_options)
    rec_file = os.path.join(rec_file_dir, rec_file_name)

    # Recursively create the storing directory path
    # TODO rethink the overwrite/extend logic
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

    outer_fields = {
        "date": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
        **model_options,
        "solver_options": solver_options,
    }
    inner_fields = sim_options

    # Parse recorded datasets
    if os.path.exists(dataset_dir):
        dataset_instances = []
        for _, _, file_names in os.walk(dataset_dir):
            dataset_instances.extend([os.path.splitext(file)[0] for file in file_names if not file.startswith(".")])
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
                if field.startswith("dataset_"):  # or field.startswith("date"):
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
                        ds_instance_name = "dataset_" + str(max_instance_number + 1).zfill(
                            3
                        )  # Add counter in the filename

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
                    print(
                        f'Dataset "{ds_name}" with instance "{dataset_instances[existing_instance_idx]}" already exists.'
                    )
                    print("[!] Warning: When generating new data, the existing dataset will be overwritten.")
                    ds_instance_name = dataset_instances[existing_instance_idx]

            else:
                # Dataset name exists but current controller configuration is new
                if ds_name.split("_")[-1].isdigit():
                    base_name = ds_name[:-3]  # strip "_XX"
                    max_counter = int(ds_name.split("_")[-1])
                    ds_name = ds_name[:-2]
                else:
                    base_name = ds_name
                    max_counter = 1
                    ds_name += "_"  # prepare for counter
                for _, dirs, _ in os.walk(DirectoryConfig.DATA_DIR):
                    for dir_name in dirs:
                        if dir_name.startswith(base_name):
                            if dir_name.split("_")[-1].isdigit():
                                # Similar dataset name exists
                                counter = int(dir_name.split("_")[-1])
                                if counter >= max_counter:
                                    max_counter = counter
                            else:
                                # Similar dataset name exists without counter
                                if max_counter == 1:
                                    max_counter = 2
                    break  # only need to check the top-level directory
                # Increment counter for dataset name
                ds_name += str(max_counter + 1).zfill(2)
                metadata[ds_name] = outer_fields
                ds_instance_name = "dataset_001"
                metadata[ds_name][ds_instance_name] = inner_fields
                # Update directory path
                dataset_dir = os.path.join(DirectoryConfig.DATA_DIR, ds_name)
        else:
            # Dataset does not exist yet in metadata
            # Add the new dataset to metadata dictionary
            ds_instance_name = "dataset_001"
            metadata[ds_name] = outer_fields
            metadata[ds_name][ds_instance_name] = inner_fields

        # Write updated metadata to file
        with open(json_file_name, "w") as json_file:
            json.dump(metadata, json_file, indent=4)

    else:
        # Metadata file does not exist yet
        with open(json_file_name, "w") as json_file:
            ds_instance_name = "dataset_001"
            metadata = {ds_name: {**outer_fields, ds_instance_name: inner_fields}}
            json.dump(metadata, json_file, indent=4)

    return dataset_dir, ds_instance_name + ".csv"


def get_model_dir_and_file(ds_name, ds_instance, model_name):
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
        for _, _, file_names in os.walk(model_dir):
            model_instances.extend([os.path.splitext(file)[0] for file in file_names if not file.startswith(".")])
    else:
        safe_mkdir_recursive(model_dir)
        model_instances = []

    # Increment counter for model file name
    if model_instances:
        existing_instances = [int(instance.split("_")[1]) for instance in model_instances]
        max_instance_number = max(existing_instances)
        model_instance = "neuralmodel_" + str(max_instance_number + 1).zfill(3)  # Add counter in the filename
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

    # fmt: off
    # Store new model configuration in metadata
    metadata[model_name][model_instance] = {
        "ModelFitConfig": {key: value for (key, value) in vars(ModelFitConfig).items() if not key.startswith("__")
                                                            and not key in ["state_feats", "u_feats", "y_reg_dims"]},
        "date": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
        "ds_nmpc_type": metadata_dataset[ds_name]["nmpc_type"],
        "ds_nmpc_params": metadata_dataset[ds_name]["nmpc_params"],
        "ds_disturbances": metadata_dataset[ds_name][ds_instance]["disturbances"],
        "MLPConfig": {key: value for (key, value) in vars(MLPConfig).items() if not key.startswith("__")},
    }
    metadata[model_name][model_instance]["ModelFitConfig"].update(
        {
            "state_feats": str(ModelFitConfig.state_feats),
            "u_feats": str(ModelFitConfig.u_feats),
            "y_reg_dims": str(ModelFitConfig.y_reg_dims),
        }
    )
    # fmt: on

    # Write updated metadata to file
    with open(json_file_name, "w") as json_file:
        json.dump(metadata, json_file, indent=4)

    return model_dir, model_instance


def delete_previous_solver_files(model_name):
    rospack = rospkg.RosPack()
    folder_path = os.path.join(
        rospack.get_path("aerial_robot_control"), "include", "aerial_robot_control", "neural_nmpc", model_name
    )

    if not os.path.exists(folder_path):
        return

    c_generated_dir = os.path.join(folder_path, "c_generated_code")

    if os.path.exists(c_generated_dir):
        shutil.rmtree(c_generated_dir)
        print("Deleted previous C generated files for solver.")

    if os.path.isfile(os.path.join(folder_path, f"{model_name}_acados_ocp.json")):
        os.remove(os.path.join(folder_path, f"{model_name}_acados_ocp.json"))

    if os.path.isfile(os.path.join(folder_path, f"{model_name}_acados_sim.json")):
        os.remove(os.path.join(folder_path, f"{model_name}_acados_sim.json"))


def make_blank_dict(target_dim, state_dim, control_dim):
    blank_recording_dict = {
        "timestamp": np.zeros((0, 1)),
        "dt": np.zeros((0, 1)),
        "comp_time": np.zeros((0, 1)),
        "target": np.zeros((0, target_dim)),
        "state_ref": np.zeros((0, state_dim)),
        "state_in": np.zeros((0, state_dim)),
        "state_out": np.zeros((0, state_dim)),
        "state_prop": np.zeros((0, state_dim)),
        "control": np.zeros((0, control_dim)),
    }
    return blank_recording_dict


def write_recording_data(rec_dict, rec_file):
    # # Current target was reached - remove incomplete recordings
    if len(rec_dict["state_in"]) > len(rec_dict["state_out"]):
        raise ValueError("Recording dictionary is not consistent.")
    #     rec_dict["timestamp"] = rec_dict["timestamp"][:-1]
    #     rec_dict["comp_time"] = rec_dict["comp_time"][:-1]
    #     rec_dict["target"] = rec_dict["target"][:-1]
    #     rec_dict["state_in"] = rec_dict["state_in"][:-1]
    #     rec_dict["control"] = rec_dict["control"][:-1]

    rec_dict_json = dict()
    for key in rec_dict.keys():
        rec_dict_json[key] = jsonify(rec_dict[key])

    df = pd.DataFrame(rec_dict_json)
    df.to_csv(rec_file, index=True, mode="a", header=False)  # Append to CSV file


def sanity_check_dataset(ds_name, ds_instance):
    # Check if actual file exists
    if not os.path.exists(os.path.join(DirectoryConfig.DATA_DIR, ds_name, ds_instance + ".csv")):
        raise FileNotFoundError(
            f"Dataset directory for dataset {ds_name} and instance {ds_instance} does not exist.\
              Record dataset or check naming."
        )

    # Check metadata
    json_file_name = os.path.join(DirectoryConfig.DATA_DIR, "metadata.json")
    with open(json_file_name, "r") as json_file:
        metadata = json.load(json_file)
    if ds_name not in metadata.keys():
        raise ValueError(f'Dataset "{ds_name}" not found in metadata.')
    if ds_instance not in metadata[ds_name].keys():
        raise ValueError(f'Dataset instance "{ds_instance}" not found in metadata for dataset "{ds_name}".')


def read_dataset(ds_name, ds_instance):
    """
    Attempts to read a dataset given its name and returns a Pandas DataFrame of the csv dataset.
    """
    ds_file_name = os.path.join(DirectoryConfig.DATA_DIR, ds_name, f"{ds_instance}.csv")
    if not os.path.exists(ds_file_name):
        raise FileNotFoundError(f"Dataset {ds_name} with instance {ds_instance} not found.")

    return pd.read_csv(ds_file_name)


def log_metrics(total_losses, inference_times, learning_rates, save_file_path, save_file_name):
    metrics = {
        "total_losses": total_losses,
        "inference_times": inference_times,
        "learning_rates": learning_rates,
        "model_config": {
            "MLPConfig": {key: value for (key, value) in vars(MLPConfig).items() if not key.startswith("__")},
            "ModelFitConfig": {key: value for (key, value) in vars(ModelFitConfig).items() if not key.startswith("__")},
        },
    }
    metrics_file_path = os.path.join(save_file_path + "/log", f"{save_file_name}_metrics.json")
    safe_mkfile_recursive(save_file_path + "/log", metrics_file_path)
    with open(metrics_file_path, "w") as f:
        json.dump(metrics, f, indent=4)


def jsonify(array):
    if isinstance(array, np.ndarray):
        return array.tolist()
    if isinstance(array, list):
        return array
    return array


def undo_jsonify(array):
    x = []
    for elem in array:
        a = elem.split("[")[1].split("]")[0].split(",")
        a = [float(num) for num in a]
        x = x + [a]
    return np.array(x)

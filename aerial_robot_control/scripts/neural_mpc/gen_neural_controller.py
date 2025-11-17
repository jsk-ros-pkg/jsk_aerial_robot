from config.configurations import EnvConfig
from neural_controller import NeuralMPC

model_options = EnvConfig.model_options

# controller_list = ["nominal"]
controller_list = ["neural_plus"]
# controller_list = ["nominal", "neural_plus"]
# controller_list.append("neural_minus")

if ["nominal"] in controller_list:
    print("Generating nominal controller...")
    model_options["only_use_nominal"] = True
    rtnmpc = NeuralMPC(
        model_options=model_options,
        solver_options=EnvConfig.solver_options,
        sim_options=EnvConfig.sim_options,
        run_options=EnvConfig.run_options,
    )
    print("Successfully generated nominal controller!")
    print("========================================")

if ["neural_plus"] in controller_list:
    print("Generating neural plus controller...")
    model_options["only_use_nominal"] = False
    model_options["plus_neural"] = True
    model_options["minus_neural"] = False
    rtnmpc = NeuralMPC(
        model_options=model_options,
        solver_options=EnvConfig.solver_options,
        sim_options=EnvConfig.sim_options,
        run_options=EnvConfig.run_options,
    )
    print("Successfully generated neural plus controller!")
    print("========================================")

if ["neural_minus"] in controller_list:
    print("Generating neural minus controller...")
    model_options["only_use_nominal"] = False
    model_options["plus_neural"] = False
    model_options["minus_neural"] = True
    rtnmpc = NeuralMPC(
        model_options=model_options,
        solver_options=EnvConfig.solver_options,
        sim_options=EnvConfig.sim_options,
        run_options=EnvConfig.run_options,
    )
    print("Successfully generated neural minus controller!")

# TODO display log
# TODO Add this script into CMakeLists.txt to run it automatically when building the package

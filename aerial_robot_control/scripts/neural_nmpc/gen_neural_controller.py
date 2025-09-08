from config.configurations import EnvConfig
from neural_controller import NeuralNMPC

rtnmpc = NeuralNMPC(
    model_options=EnvConfig.model_options,
    solver_options=EnvConfig.solver_options,
    sim_options=EnvConfig.sim_options,
    run_options=EnvConfig.run_options,
)

# TODO generate nominal and neural controllers by setting model options here
# TODO display log
# TODO Add this script into CMakeLists.txt to run it automatically when building the package

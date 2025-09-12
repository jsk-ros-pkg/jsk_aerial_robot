from config.configurations import EnvConfig
from neural_controller import NeuralNMPC

model_options = EnvConfig.model_options

# print("Generating nominal controller...")
# model_options["only_use_nominal"] = True
# rtnmpc = NeuralNMPC(
#     model_options=model_options,
#     solver_options=EnvConfig.solver_options,
#     sim_options=EnvConfig.sim_options,
#     run_options=EnvConfig.run_options,
# )
# print("Successfully generated nominal controller!")
# print("========================================")


print("Generating neural plus controller...")
model_options["only_use_nominal"] = False
model_options["plus_neural"] = True
model_options["minus_neural"] = False
rtnmpc = NeuralNMPC(
    model_options=model_options,
    solver_options=EnvConfig.solver_options,
    sim_options=EnvConfig.sim_options,
    run_options=EnvConfig.run_options,
)
print("Successfully generated neural plus controller!")
print("========================================")

print("Generating neural minus controller...")
model_options["only_use_nominal"] = False
model_options["plus_neural"] = False
model_options["minus_neural"] = True
rtnmpc = NeuralNMPC(
    model_options=model_options,
    solver_options=EnvConfig.solver_options,
    sim_options=EnvConfig.sim_options,
    run_options=EnvConfig.run_options,
)
print("Successfully generated neural minus controller!")

# TODO display log
# TODO Add this script into CMakeLists.txt to run it automatically when building the package

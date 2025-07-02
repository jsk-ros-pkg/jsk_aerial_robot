def load_model(model_options, sim_options):
    """
    Load a pre-trained neural network for the NMPC controller.
    """
    # # === Load the model ===
    # model = SimpleMLP().to(device)
    # model.load_state_dict(torch.load("model.pth", weights_only=True))

    # TODO get config from metadata
    mlp_config = {'approximated': False, 'v_inp': True, 'u_inp': False, 'T_out': False, 'ground_map_input': False,
                    'torque_output': False, 'two_step_rti': False}
    
    # Load trained MLP model
    directory, file_name = get_model_dir_and_file(model_params)
    saved_network = torch.load(os.path.join(directory, f"{file_name}.pt"))
    # saved_network = {"input_size": 12, "hidden_size": 64, "output_size": 12, "hidden_layers": 3, "state_dict": {}}
    base_mlp = mc.nn.MultiLayerPerceptron(saved_network['input_size'], saved_network['hidden_size'],
                                            saved_network['output_size'], saved_network['hidden_layers'],
                                            'Tanh')
    neural_model = NormalizedMLP(
                        base_mlp,
                        torch.tensor(np.zeros((saved_network['input_size'],))).float(),
                        torch.tensor(np.zeros((saved_network['input_size'],))).float(),
                        torch.tensor(np.zeros((saved_network['output_size'],))).float(),
                        torch.tensor(np.zeros((saved_network['output_size'],))).float())
    # Load weights and biases from saved model
    neural_model.load_state_dict(saved_network['state_dict'])
    neural_model.eval()

    return neural_model, mlp_config

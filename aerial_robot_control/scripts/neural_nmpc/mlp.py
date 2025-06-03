
class MLP():
    def __init__(self):
        mlp_conf = {'approximated': False, 'v_inp': True, 'u_inp': False, 'T_out': False, 'ground_map_input': False,
                    'torque_output': False, 'two_step_rti': False}
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
        rdrv_d = None

        if reg_type.endswith('approx2'):
            mlp_conf['approximated'] = True
            mlp_conf['approx_order'] = 2
        elif reg_type.endswith('approx') or reg_type.endswith('approx_1'):
            mlp_conf['approximated'] = True
            mlp_conf['approx_order'] = 1
        if '_u' in reg_type:
            mlp_conf['u_inp'] = True
        if '_T' in reg_type:
            mlp_conf['T_out'] = True
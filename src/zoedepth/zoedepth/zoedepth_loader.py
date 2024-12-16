import torch

def zoedepth_loader(model_repo, model_type, device):
    # Model configuration
    model_configs = {
        "N": {
            "name": "ZoeD_N",
            "weights": "https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_N.pt",
        },
        "K": {
            "name": "ZoeD_K",
            "weights": "https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_K.pt",
        },
        "NK": {
            "name": "ZoeD_NK",
            "weights": "https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_NK.pt",
        },
    }

    # Validate model type
    if model_type not in model_configs:
        raise ValueError(f"model_type must be one of: {list(model_configs.keys())}")

    config = model_configs[model_type]

    # Load model
    model = torch.hub.load(model_repo, config["name"], pretrained=False)
    pretrained_dict = torch.hub.load_state_dict_from_url(
        config["weights"],
        map_location=device,
    )
    model.load_state_dict(pretrained_dict["model"], strict=False)

    # Apply Identity to drop_path
    for b in model.core.core.pretrained.model.blocks:
        b.drop_path = torch.nn.Identity()

    return model

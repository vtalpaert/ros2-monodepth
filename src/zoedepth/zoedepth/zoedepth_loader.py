import torch

def zoedepth_loader(model_repo, model_name, device):
    # Model weights mapping
    weights_urls = {
        "ZoeD_N": "https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_N.pt",
        "ZoeD_K": "https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_K.pt",
        "ZoeD_NK": "https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_NK.pt",
    }

    # Validate model name
    if model_name not in weights_urls:
        raise ValueError(f"model_name must be one of: {list(weights_urls.keys())}")

    # Load model
    model = torch.hub.load(model_repo, model_name, pretrained=False)
    pretrained_dict = torch.hub.load_state_dict_from_url(
        weights_urls[model_name],
        map_location=device,
    )
    model.load_state_dict(pretrained_dict["model"], strict=False)

    # Apply Identity to drop_path
    for b in model.core.core.pretrained.model.blocks:
        b.drop_path = torch.nn.Identity()

    return model

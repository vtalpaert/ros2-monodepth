import onnxruntime as rt


def create_session(model_path):
    sess_options = rt.SessionOptions()

    # Set graph optimization level
    sess_options.graph_optimization_level = (
        rt.GraphOptimizationLevel.ORT_ENABLE_EXTENDED
    )
    # ORT_ENABLE_ALL

    # To enable model serialization after graph optimization set this
    # sess_options.optimized_model_filepath = "<model_output_path\optimized_model.onnx>"

    providers = [
        (
            "TensorrtExecutionProvider",
            {
                # "device_id": 0,  # Select GPU to execute
                # "trt_max_workspace_size": 2147483648,  # Set GPU memory usage limit
                "trt_fp16_enable": True,  # Enable FP16 precision for faster inference
                "trt_dla_enable": True,  # Note: Not all Nvidia GPUs support Deep Learning Accelerator
            },
        ),
        (
            "CUDAExecutionProvider",
            {
                # "device_id": 0,
                "arena_extend_strategy": "kNextPowerOfTwo",
                # "gpu_mem_limit": 2 * 1024 * 1024 * 1024,
                # "cudnn_conv_algo_search": "EXHAUSTIVE",
                # "do_copy_in_default_stream": True,
            },
        ),
        "CPUExecutionProvider",
    ]

    session = rt.InferenceSession(
        model_path, sess_options=sess_options, providers=providers
    )

    return session


def predict(session, input_tensor):
    pred_onx = session.run(None, {"input": input_tensor})[0]
    return pred_onx


def main(args=None):
    import torch
    from zoedepth.zoedepth_loader import zoedepth_loader

    # Get parameters
    model_repo = "isl-org/ZoeDepth"
    model_name = "ZoeD_N"
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model_path = f"{model_name}.onnx"

    model = zoedepth_loader(model_repo, model_name, device)

    print(f"Replacing Resizer {model.core.prep.resizer} with nn.Identity")
    model.core.prep.resizer = torch.nn.Identity()

    model = model.to(device)
    print("Model loaded successfully!")

    torch.onnx.export(
        model,
        (
            # torch.rand((1, 3, 384, 512), dtype=torch.float32),
            torch.randn((1, 3, 480, 640), dtype=torch.float32).to(device),
            # np.random.rand(1, 3, 480, 640).astype(np.float32),
        ),
        model_path,
        input_names=["input"],
        dynamo=False,
        fallback=False,
        # export_options=export_options,
        verbose=True,
    )
    print(f"Model exported to {model_path}")


if __name__ == "__main__":
    main()

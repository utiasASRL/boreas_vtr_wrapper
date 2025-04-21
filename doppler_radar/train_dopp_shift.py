import os
import neptune
from neptune_pytorch import NeptuneLogger
from neptune.utils import stringify_unsupported
import torch
from dataset_dopp_shift import DoppShiftDataset



def main():
    # neptune_mode = "debug"  # async, debug
    # run = neptune.init_run(
    #     project="temp",     # Your neptune project here
    #     api_token="temp",   # Your neptune api here
    #     mode=neptune_mode
    # )

    params = {
        "device": torch.device("cuda" if torch.cuda.is_available() else "cpu"),

        # Dataset params
        "data_dir": "/home/dl/Documents/phd/dev/doppler_radar/data/vtr_data",
        "folder_name": "paper1_radar",
        "num_train": -1,
        "num_val": -1,
        "float_type": torch.float32,

        # Iterator params
        "batch_size_train": 16,
        "batch_size_test": 32,
        "shuffle": True,
    
        # Training params
        "num_epochs": 30,
        "learning_rate": 1e-4,
    }

    print("Using device: ", params['device'])

    train_sequences = [
        "boreas-2024-01-23-11-45",  # Glen
        "boreas-2024-02-13-15-50",  # Hwy 7
        "boreas-2024-02-29-14-47",  # Tunnel
    ]

    validate_sequences = [
        "boreas-2024-03-08-11-43",  # Glen
        "boreas-2024-03-08-12-13",  # Hwy 7
        "boreas-2024-02-29-14-58",  # Tunnel
    ]

    # Load the dataset
    dataset_train = DoppShiftDataset(train_sequences, params)



if __name__ == "__main__":
    main()
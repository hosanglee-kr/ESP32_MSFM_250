# read preprocessed data
# structure data so that it can used for modeling
#  - merge test results
# select columns for ml model

import os
import argparse
import pickle
import pandas as pd
from pkgs.pipelines import _read_json
from azureml.train.automl import AutoMLConfig
from azureml.core import Workspace, Dataset
from azureml.core.model import Model
from azureml.core.experiment import Experiment
import time


def main(
    input_file_dir="../data/processed/",
    input_file_name=None,
    input_file_format="csv",
    model_register=False,
    model_name="04.00.00",
    model_file_dir="../models/",
    model_file_name="model.pkl",
    feature_names=None,
    label="SR_RELABELED",
    azureml_config_dir="../.azureml/config.json",
    experiment_name="challenger",
    desc="",
    use_azureml=False,
    compute_target=None,
    automl_settings=None,
):
    # Set workspace
    ws = Workspace.from_config(azureml_config_dir)

    # Read data
    if use_azureml is False:
        file_path = os.path.join(input_file_dir, f"{input_file_name}.{input_file_name}")
        training_data = pd.read_csv(file_path)
    else:
        datastore = ws.get_default_datastore()
        training_data = Dataset.Tabular.from_delimited_files(
            path=(
                datastore,
                os.path.join(
                    f"{input_file_dir}", f"{input_file_name}.{input_file_format}"
                ),
            )
        )

    # Train
    automl_config = AutoMLConfig(
        task="classification",
        debug_log="automl_errors.log",
        path="../",
        compute_target=compute_target,
        training_data=training_data,
        label_column_name=label,
        **automl_settings,
    )
    experiment = Experiment(workspace=ws, name=experiment_name)
    run = experiment.submit(automl_config, show_output=True)

    # Save the best model
    best_run, fitted_model = run.get_output()
    model_path = os.path.join(model_file_dir, model_file_name)
    with open(model_path, "wb") as f:
        pickle.dump(fitted_model, f)

    # Register the best model
    # library_version = "DL"+sklearn.__version__.replace(".","x")
    if model_register is True:
        Model.register(
            model_path=model_path,
            model_name=model_name,
            tags={"area": "IoT Edge", "type": "automl"},
            description=f"InputData:{input_file_name}, Comment:{desc}",
            workspace=ws,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_file_dir", default="../data/processed/")
    parser.add_argument("--input_file_name", "-if")
    parser.add_argument("--input_file_format", default="csv")
    parser.add_argument("--model_register", dest="model_register", action="store_true")
    parser.add_argument(
        "--not_model_register", dest="model_register", action="store_false"
    )
    parser.set_defaults(model_register=False)
    parser.add_argument("--model_name", default="04.00.00")
    parser.add_argument("--model_file_dir", default="../models")
    parser.add_argument("--model_file_name", default="model_v2.pkl")
    parser.add_argument("--file_surfix", default="_pred")
    parser.add_argument("--label", default="SR_RELABELED")
    parser.add_argument("--azureml_config_dir", default="../.azureml/config.json")
    parser.add_argument("--experiment_name", default="challenger")
    parser.add_argument("--compute_target", default="mc-cmpp2")
    parser.add_argument("--use_azureml", dest="use_azureml", action="store_true")
    parser.add_argument("--not_use_azureml", dest="use_azureml", action="store_false")
    parser.set_defaults(use_azureml=False)
    parser.add_argument(
        "--cfg_features_name",
        dest="cfg_features_name_json",
        default="config/build_features_train_V2.json",
    )
    args = parser.parse_args()

    if args.use_azureml is True:
        args.model_file_dir = "outputs/"
        if args.compute_target is None:
            raise ("compute_target을 입력하세요.")
    if args.cfg_features_name_json is not None:
        cfgs = _read_json(args.cfg_features_name_json)
        args.feature_names = cfgs["feature_names"]

    # Set Train Configuration
    automl_settings = {
        "name": f"{args.experiment_name}_{time.time()}",
        "experiment_timeout_minutes": 20,
        "enable_early_stopping": True,
        "iteration_timeout_minutes": 10,
        "n_cross_validations": 5,
        "primary_metric": "AUC_weighted",
        "max_concurrent_iterations": 10,
    }

    main(
        input_file_dir=args.input_file_dir,
        input_file_name=args.input_file_name,
        input_file_format=args.input_file_format,
        model_register=args.model_register,
        model_name=args.model_name,
        model_file_dir=args.model_file_dir,
        model_file_name=args.model_file_name,
        feature_names=args.feature_names,
        label=args.label,
        azureml_config_dir=args.azureml_config_dir,
        experiment_name=args.experiment_name,
        use_azureml=args.use_azureml,
        compute_target=args.compute_target,
        automl_settings=automl_settings,
    )

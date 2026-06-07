# read preprocessed data
# structure data so that it can used for modeling
#  - merge test results
# select columns for ml model

import os
import shutil
import argparse
import pathlib
import pickle
import glob
from azureml.core.model import Model
from azureml.train.automl.run import AutoMLRun
from azureml.core import Run


def main(
    training_data,
    input_model,
    input_model_name=None,
    metrics=None,
    model_register=True,
    desc=None,
    model_upload_dir="<NO_UPLOAD>",
):
    run = Run.get_context()
    if type(run) == _OfflineRun:
        ws = Workspace.from_config()
    else:
        ws = run.experiment.workspace
        # get model
        run_id = os.path.basename(os.path.dirname(input_model))
        _run = AutoMLRun(run.experiment, run_id=run_id)
        best_run, _ = _run.get_output()
        print(best_run)

        # save model
        if input_model_name is None:
            input_model_name = os.path.basename(input_model)
        input_model_dir = os.path.join("outputs/", input_model_name)
        os.makedirs(input_model_dir, exist_ok=True)
        best_run.download_files(output_directory=input_model_dir)

        shutil.copyfile(os.path.join(input_model_dir, "outputs/model.pkl"), input_model)

    # upload model
    if model_upload_dir != "<NO_UPLOAD>":
        datastore = ws.get_default_datastore()
        datastore.upload(
            src_dir="outputs/", target_path=model_upload_dir, overwrite=True
        )

    # register model
    if model_register is True:
        training_data_name = os.path.basename(training_data)
        model = Model.register(
            model_path=input_model,
            model_name=input_model_name,
            # datasets = list(list("training", train_ds), list("inferencing", infer_ds))
            tags={"area": "IoT Edge", "type": "automl"},
            description=f"InputData:{training_data_name}, Comment:{desc}",
            workspace=ws,
        )
        print("Registered version {0} of model {1}".format(model.version, model.name))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--training_data")
    parser.add_argument("--input_model")
    parser.add_argument("--input_model_name", default=None)
    parser.add_argument("--metrics", default=None)
    parser.add_argument("--model_register", dest="model_register", action="store_true")
    parser.add_argument(
        "--not_model_register", dest="model_register", action="store_false"
    )
    parser.set_defaults(model_register=False)
    parser.add_argument("--desc", default="")
    parser.add_argument("--model_upload_dir", default="<NO_UPLOAD>")
    args = parser.parse_args()

    main(
        training_data=args.training_data,
        input_model=args.input_model,
        input_model_name=args.input_model_name,
        metrics=args.metrics,
        model_register=args.model_register,
        desc=args.desc,
        model_upload_dir=args.model_upload_dir,
    )

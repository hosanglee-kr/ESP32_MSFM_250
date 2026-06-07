# read preprocessed data
# structure data so that it can used for modeling
#  - merge test results
# select columns for ml model

import os
import argparse
import pathlib
from azureml.core import Run
from azureml.core import Datastore, Dataset


def upload_data(output_data, output_upload_dir):
    # upload data
    # get file name and dir absolute
    file_path = str(pathlib.Path(output_data).absolute())
    rel_root = os.path.dirname(file_path)
    # upload
    run = Run.get_context()
    ws = run.experiment.workspace
    datastore = ws.get_default_datastore()
    datastore.upload_files(
        files=[file_path],
        relative_root=rel_root,
        target_path=output_upload_dir,
        overwrite=True,
    )
    print(f"{output_data} uploaded to {datastore.name}/{output_upload_dir}")


def main(
    input_data,
    input_data_name=None,
    datastore_name="workspaceblobstore",
    desc=None,
    model_upload_dir="<NO_UPLOAD>",
):
    if input_data_name is None:
        input_data_name = os.path.basename(input_data)
    # upload dataset
    if model_upload_dir != "<NO_UPLOAD>":
        upload_data(input_data, model_upload_dir)

    # register dataset
    run = Run.get_context()
    ws = run.experiment.workspace
    # retrieve an existing datastore in the workspace by name
    datastore = Datastore.get(ws, datastore_name)
    datastore_paths = [(datastore, os.path.join(model_upload_dir, input_data_name))]
    # data_train = data_build.parse_delimited_files(
    #     file_extension=None,
    #     set_column_types=set_feature_datatype(
    #         os.path.join(SOURCE_DIR, FEATURE_NAMES_CFG03)
    #     ),

    ds = Dataset.Tabular.from_delimited_files(path=datastore_paths)

    ds = ds.register(workspace=ws, name=input_data_name, description=desc)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_data")
    parser.add_argument("--input_data_name", default=None)
    parser.add_argument("--desc", default="")
    parser.add_argument("--model_upload_dir", default="<NO_UPLOAD>")
    args = parser.parse_args()

    main(
        input_data=args.input_data,
        input_data_name=args.input_data_name,
        desc=args.desc,
        model_upload_dir=args.model_upload_dir,
    )

# read preprocessed data
# structure data so that it can used for modeling
#  - merge test results
# select columns for ml model
import os
import joblib
import argparse
import pathlib
import pandas as pd
from pkgs.utils import read_json
from azureml.core import Run


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
    output_data,
    model_path,
    label="SR_RELABELED",
    feature_names=None,
    output_upload_dir="<NO_UPLOAD>",
):
    # Read data
    df = pd.read_csv(input_data)
    print("Original Shape: ", end="")
    print(df.shape)

    # Preprocess data
    df = df.drop_duplicates()

    # Load model
    model = joblib.load(model_path)

    # Run model
    try:
        if feature_names is not None:
            df_features = df[feature_names]
            df_features = df_features.dropna()
        else:
            df_features = df.drop(label, axis=1)
            df_features = df_features.dropna()
        print(model.predict_proba(df_features))
        df_features["PROB"] = model.predict_proba(df_features)[:, 1]
        # Save data
    except Exception as e:
        raise (e)
    print("Predicted Shape: ", end="")
    print(df_features.shape)

    # Merge Results
    if "PROB" in df.columns:
        df["PROB_OLD"] = df["PROB"]
        df = df.drop("PROB", axis=1)
    if "R" in df.columns:
        df["R_OLD"] = df["R"]
        df = df.drop("R", axis=1)
    df = df.join(df_features["PROB"])

    # Save data
    os.makedirs(os.path.dirname(output_data), exist_ok=True)
    df.to_csv(output_data, index=False)

    # upload data
    if output_upload_dir != "<NO_UPLOAD>":
        upload_data(output_data, output_upload_dir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_data")
    parser.add_argument("--output_data")
    parser.add_argument("--model_path")
    parser.add_argument("--label", default="SR_RELABELED")
    parser.add_argument("--output_upload_dir", default="<NO_UPLOAD>")
    parser.add_argument(
        "--cfg_features_name",
        dest="cfg_features_name_json",
        default="",
    )
    args = parser.parse_args()
    if args.cfg_features_name_json == "":
        args.feature_names = None
    else:
        cfgs = read_json(args.cfg_features_name_json)
        args.feature_names = cfgs["feature_names"]

    main(
        input_data=args.input_data,
        output_data=args.output_data,
        model_path=args.model_path,
        feature_names=args.feature_names,
        label=args.label,
        output_upload_dir=args.output_upload_dir,
    )

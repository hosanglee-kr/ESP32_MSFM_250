import os
import pathlib
import argparse
import numpy as np
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
    output_path=None,
    feature_names=None,
    dt_from=None,
    dt_to=None,
    lid_list=None,
    label="SR_RELABELED",
    sample_weight=True,
    output_upload_dir="<NO_UPLOAD>",
):
    # Read preprocessed data
    try:
        df_list = []
        for input_file in os.listdir(input_data):
            tmp = pd.read_csv(os.path.join(input_data, input_file))
            df_list.append(tmp)
        df = pd.concat(df_list)
        print(f"Total {len(df_list)} files were read")
    except Exception as e:
        print(e)
        df = pd.read_csv(input_data, engine="python")

    # Preprocess
    df["DT"] = pd.to_datetime(df.DT)
    if feature_names is None:
        feature_names = df.drop(label, axis=1).columns.tolist()
    if dt_from is None:
        dt_from = df.DT.dropna().min()
    if dt_to is None:
        dt_to = df.DT.dropna().max()
    if lid_list is None:
        lid_list = df.LID.dropna().unique()

    print("Original Shape: ", end="")
    df = df.fillna({"SR_RELABELED": 0})
    df = df.drop_duplicates()

    # Set conditions
    condition = (df.DT >= dt_from) & (df.DT <= dt_to) & (df.LID.isin(lid_list))
    print(np.sum(condition))
    print("DATE FROM: ", end="")
    print(dt_from)
    print("DATE TO: ", end="")
    print(dt_to)
    print("LINES: ", end="")
    print("/".join([str(lid) for lid in lid_list]))

    # Create data
    df = df.loc[condition, feature_names + ["SR_RELABELED"]]

    # Set Sample Weight
    if sample_weight is True:
        df["SAMPLE_WEIGHT"] = np.where(
            df.SR_RELABELED == 0,
            1,
            np.sum(df.SR_RELABELED == 0) / np.sum(df.SR_RELABELED == 1),
        )[0]

    print("After Preprocessed: ", end="")
    print(df.shape)

    # Save data
    if output_path is not None:
        output_file_path = os.path.join(output_path, output_data + ".csv")
    else:
        output_file_path = output_data
    os.makedirs(os.path.dirname(output_file_path), exist_ok=True)
    df.to_csv(output_file_path, index=False)

    # os.makedirs(os.path.dirname(output_data), exist_ok=True)
    # df.to_csv(output_data, index=False)

    # upload data
    if output_upload_dir != "<NO_UPLOAD>":
        upload_data(output_file_path, output_upload_dir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_data")
    parser.add_argument("--output_data", default="outputs/output.csv")
    parser.add_argument("--output_path", default=None)
    parser.add_argument("--dt_from", default=None)
    parser.add_argument("--dt_to", default=None)
    parser.add_argument("--lid_list", nargs="*", default=None)
    parser.add_argument("--label", default="SR_RELABELED")
    parser.add_argument("--sample_weight", default=False)
    parser.add_argument(
        "--cfg_features_name",
        dest="cfg_features_name_json",
        default="",
    )
    parser.add_argument("--output_upload_dir", default="<NO_UPLOAD>")
    args = parser.parse_args()

    # Get default feature names
    if args.cfg_features_name_json == "":
        args.feature_names = None
    else:
        cfgs = read_json(args.cfg_features_name_json)
        args.feature_names = cfgs["feature_names"]

    main(
        input_data=args.input_data,
        output_data=args.output_data,
        output_path=args.output_path,
        feature_names=args.feature_names,
        dt_from=args.dt_from,
        dt_to=args.dt_to,
        lid_list=args.lid_list,
        label=args.label,
        sample_weight=args.sample_weight,
        output_upload_dir=args.output_upload_dir,
    )
